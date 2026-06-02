"""Diagnostic: does Dobot's pose Euler rotate with the user coordinate system?

Run this once outside the GUI to find out whether SetUser(N, {...,rz}) actually
rotates the (rx, ry, rz) values returned by GetPose, or only the (x, y, z).

Usage (PowerShell, with GUI closed so the robot socket is free):

    cd c:\\Users\\Administrator\\Desktop\\robot-control-code
    python diag_user_frame.py

Read the two GetPose lines it prints:
- If user=0 and user=1 give the SAME rx/ry/rz (only x/y/z differ): the Euler
  is in base frame regardless of user. Tell me, I'll switch to matrix math.
- If user=0 and user=1 give DIFFERENT rx/ry/rz (by ~92 around some axis):
  the user frame does rotate Euler. Then SetUser approach is viable.
"""

from time import sleep

import config
from Dobot.dobot import Dobot


# Rotation about base Z used to define user index 1. Set this to whatever angle
# you expect to "compensate" the probe's yaw (your TOOL_FRAME.rz is -92).
TEST_USER_RZ = -92.0


def main():
    dobot = Dobot(config.DOBOT_IP)
    dobot.connect()

    # Wait briefly so dashboard is reachable; we don't need full enable here.
    sleep(0.5)

    table = "{0,0,0,0,0," + repr(float(TEST_USER_RZ)) + "}"
    print("\n=== SetUser(1, " + table + ") (define) ===")
    print(dobot.dashboard.SetUser(1, table))

    print("\n--- Before activation (User(1) not yet called) ---")
    print("=== GetPose(user=0, tool=0) ===")
    print(dobot.dashboard.GetPose(user=0, tool=0))
    print("=== GetPose(user=1, tool=0) ===")
    print(dobot.dashboard.GetPose(user=1, tool=0))

    print("\n=== User(1) (activate as global) ===")
    print(dobot.dashboard.User(1))

    print("\n--- After activation ---")
    print("=== GetPose(user=0, tool=0) ===")
    print(dobot.dashboard.GetPose(user=0, tool=0))
    print("=== GetPose(user=1, tool=0) ===")
    print(dobot.dashboard.GetPose(user=1, tool=0))

    # Restore the global user and reset user 1 so the controller is clean.
    print("\n=== User(0) (restore global) ===")
    print(dobot.dashboard.User(0))
    print("=== SetUser(1, {0,0,0,0,0,0}) (reset) ===")
    print(dobot.dashboard.SetUser(1, "{0,0,0,0,0,0}"))

    try:
        dobot.dashboard.close()
    except Exception:
        pass
    try:
        dobot.feedFour.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
