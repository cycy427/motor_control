import time
from copy import deepcopy
import threading

from cyclonedds.core import DDSException
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
import nubotddsmsg.hr as hrmsg

LEGCMDTOPIC = "/nubot/z1/legmotorcmds"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"

YKS_GLOBAL_INDEX = 0
EYOU_GLOBAL_INDEX = 4
CURRENT_MODE_CODE = 2

YKS_CURRENT_TARGET = 150.0
EYOU_CURRENT_TARGET = 250.0
POSITIVE_HOLD_DURATION = 2.0
ZERO_HOLD_DURATION = 1.0
NEGATIVE_HOLD_DURATION = 2.0
RETURN_ZERO_HOLD_DURATION = 1.0

SEND_INTERVAL = 0.02
PRINT_INTERVAL = 0.5
STOP_HOLD = 0.3


class Z1RemoteClient(threading.Thread):
	def __init__(self):
		super(Z1RemoteClient, self).__init__()
		self.daemon = True
		self._motorCmds = hrmsg.motorcmds(
			level=0,
			cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(12)],
		)
		self.motorCmds = hrmsg.motorcmds(
			level=0,
			cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(12)],
		)
		self._motorStates = hrmsg.motorstates(
			level=0,
			states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in range(12)],
		)

		participant = DomainParticipant(domain_id=0)
		cmdtopic_obj = Topic(participant, LEGCMDTOPIC, hrmsg.motorcmds)
		cmdqos = Qos(
			Policy.Reliability.BestEffort,
			Policy.Durability.Volatile,
			Policy.History.KeepLast(5),
		)
		publisher = Publisher(participant)
		self.writer = DataWriter(publisher, cmdtopic_obj, qos=cmdqos)

		stateqos = Qos(
			Policy.Reliability.BestEffort,
			Policy.Durability.Volatile,
			Policy.History.KeepLast(5),
		)
		statetopic_obj = Topic(participant, LEGSTATETOPIC, hrmsg.motorstates, qos=stateqos)
		self.reader = DataReader(participant, statetopic_obj)

		self.running = True
		self.reader_valid = False
		self._lockcmd = threading.RLock()
		self._lockstate = threading.RLock()

		self.start()

	def run(self):
		while self.running:
			msgs = []
			try:
				msgs = self.reader.take()
				self.reader_valid = bool(msgs)
			except DDSException as exc:
				print("[Reader] catch DDSException msg:", exc.msg)
			except TimeoutError:
				print("[Reader] take sample timeout")
			except Exception:
				print("[Reader] take sample error")

			if msgs:
				with self._lockstate:
					self._motorStates = msgs[-1]

			with self._lockcmd:
				try:
					self.writer.write(self._motorCmds)
				except DDSException as exc:
					print("[Writer] catch DDSException error. msg:", exc.msg)
				except Exception as exc:
					print("[Writer] write sample error. msg:", exc)

			time.sleep(0.002)

	def stop(self):
		self.running = False
		self.join(timeout=1.0)

	def setCommand(self):
		with self._lockcmd:
			self._motorCmds = deepcopy(self.motorCmds)

	def getStates(self):
		with self._lockstate:
			return deepcopy(self._motorStates)


def wait_for_valid_state(client, timeout=1.0):
	deadline = time.time() + timeout
	while time.time() < deadline:
		if client.reader_valid:
			return client.getStates()
		time.sleep(0.01)
	return client.getStates()


def clear_all_commands(client):
	for cmd in client.motorCmds.cmds:
		cmd.mode = 0
		cmd.pos = 0.0
		cmd.vel = 0.0
		cmd.tau = 0.0
		cmd.kp = 0.0
		cmd.kd = 0.0


def set_leg_current_command(client, global_index, current_target):
	if not 0 <= global_index < len(client.motorCmds.cmds):
		raise IndexError(f"Invalid leg global index: {global_index}")

	cmd = client.motorCmds.cmds[global_index]
	cmd.mode = CURRENT_MODE_CODE
	cmd.pos = 0.0
	cmd.vel = 0.0
	cmd.tau = current_target
	cmd.kp = 0.0
	cmd.kd = 0.0


def apply_dual_current_commands(client, yks_current_target, eyou_current_target):
	clear_all_commands(client)
	set_leg_current_command(client, YKS_GLOBAL_INDEX, yks_current_target)
	set_leg_current_command(client, EYOU_GLOBAL_INDEX, eyou_current_target)


def send_stop_commands(client):
	stop_end_time = time.time() + STOP_HOLD
	while time.time() < stop_end_time:
		clear_all_commands(client)
		client.setCommand()
		time.sleep(SEND_INTERVAL)


def print_motor_state(states, global_index, motor_name):
	if not 0 <= global_index < len(states.states):
		print(f"[State] {motor_name} global_index={global_index} out of range")
		return

	state = states.states[global_index]
	print(
		f"[State] {motor_name} global_index={global_index} mode={state.mode} "
		f"pos={state.pos:.4f} vel={state.vel:.4f} tau={state.tau:.4f} err={state.error}"
	)


def compute_segment_current(current_amplitude, start_time, now_time):
	elapsed = now_time - start_time
	cycle_duration = (
		POSITIVE_HOLD_DURATION
		+ ZERO_HOLD_DURATION
		+ NEGATIVE_HOLD_DURATION
		+ RETURN_ZERO_HOLD_DURATION
	)
	cycle_time = elapsed % cycle_duration

	if cycle_time < POSITIVE_HOLD_DURATION:
		return current_amplitude, "positive_current"

	cycle_time -= POSITIVE_HOLD_DURATION
	if cycle_time < ZERO_HOLD_DURATION:
		return 0.0, "hold_zero_after_positive"

	cycle_time -= ZERO_HOLD_DURATION
	if cycle_time < NEGATIVE_HOLD_DURATION:
		return -current_amplitude, "negative_current"

	return 0.0, "hold_zero_after_negative"


if __name__ == "__main__":
	client = Z1RemoteClient()
	start_time = time.time()
	print(
		f"[DDS Test] dual staged current control: YKS(id=1, global_index={YKS_GLOBAL_INDEX}, current={YKS_CURRENT_TARGET}), "
		f"EYOU(id=5, global_index={EYOU_GLOBAL_INDEX}, current={EYOU_CURRENT_TARGET})"
	)

	last_print = 0.0
	try:
		while True:
			now = time.time()
			current_yks_target, yks_phase = compute_segment_current(
				YKS_CURRENT_TARGET,
				start_time,
				now,
			)
			current_eyou_target, eyou_phase = compute_segment_current(
				EYOU_CURRENT_TARGET,
				start_time,
				now,
			)
			apply_dual_current_commands(client, current_yks_target, current_eyou_target)
			client.setCommand()

			if now - last_print >= PRINT_INTERVAL:
				states = client.getStates()
				print(
					f"[Target] YKS={current_yks_target:.4f}({yks_phase}) EYOU={current_eyou_target:.4f}({eyou_phase})"
				)
				print_motor_state(states, YKS_GLOBAL_INDEX, "YKS")
				print_motor_state(states, EYOU_GLOBAL_INDEX, "EYOU")
				last_print = now

			time.sleep(SEND_INTERVAL)
	except KeyboardInterrupt:
		print("\n[DDS Test] stop requested")
	finally:
		send_stop_commands(client)
		client.stop()
