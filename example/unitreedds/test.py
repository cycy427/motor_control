from unitree_pub import Z1UnitreePUBClient
import time
UNITREEPUBDATATOPIC = "/nubot/z1/unitreepubdata"

if __name__ == '__main__':
    i=0
    z1_unitree = Z1UnitreePUBClient(UNITREEPUBDATATOPIC)

    while True:
        i=i+1
        z1_unitree.Move(vx=0.1*i, vy=0.2*i, omega=0.3*i)
        print("Move to ", z1_unitree._unitreeStates.vx, z1_unitree._unitreeStates.vy, z1_unitree._unitreeStates.omega)
        time.sleep(0.1)
