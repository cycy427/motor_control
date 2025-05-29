from hcmd_pub import Z1HcmdPUBClient
import time
HCMDPUBDATATOPIC = "/nubot/z1/hcmdpubdata"

if __name__ == '__main__':
    i=0
    z1_hcmd = Z1HcmdPUBClient(HCMDPUBDATATOPIC)

    while True:
        i=i+1
        z1_hcmd.Move(0.1*i, 0.2*i, 0.3*i)
        print("Move to ", z1_hcmd._hcmdStates.vx, z1_hcmd._hcmdStates.vy, z1_hcmd._hcmdStates.omega)
        time.sleep(0.1)
