from Dobot import Dobot
import threading
from time import sleep


# Robot run settings.
DOBOT_IP = "192.168.5.1"
SPEED_RATIO = 20
CIRCLE_RADIUS_MM = 100
ARC_BLEND_CP = 50


def has_robot_error(dobot, language="zh_cn"):
    # Read active robot alarms from the dashboard interface.
    error_info = dobot.dashboard.GetError(language)
    errors = error_info.get("errMsg", []) if error_info else []

    if not errors:
        print("Robot status normal, no error information")
        return False

    print(f"Found {len(errors)} robot error(s):")
    for index, error in enumerate(errors, 1):
        print(f"Error {index}:")
        print(f"  ID: {error.get('id', 'N/A')}")
        print(f"  Level: {error.get('level', 'N/A')}")
        print(f"  Description: {error.get('description', 'N/A')}")
        print(f"  Solution: {error.get('solution', 'N/A')}")
        print(f"  Mode: {error.get('mode', 'N/A')}")
        print(f"  Date: {error.get('date', 'N/A')}")
        print(f"  Time: {error.get('time', 'N/A')}")
    return True


def start():
    # Create and connect the Dobot control object.
    dobot = Dobot(DOBOT_IP)
    dobot.connect()

    # Do not enable the robot if it already has active alarms.
    print("Checking robot error information before enable...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before EnableRobot.")
        return

    # Enable the robot before sending any motion commands.
    enable_result = dobot.dashboard.EnableRobot()
    print("EnableRobot:", enable_result)
    if dobot.parseResultId(enable_result)[0] != 0:
        print("使能失败: 请检查机器人是否在 TCP/IP 模式、是否有报警/急停、以及 29999 端口连接")
        has_robot_error(dobot)
        return
    print("使能成功")

    # Set a conservative speed before running the demo path.
    speed_commands = [
        ("SpeedFactor", dobot.dashboard.SpeedFactor(SPEED_RATIO)),
        ("VelJ", dobot.dashboard.VelJ(SPEED_RATIO)),
        ("AccJ", dobot.dashboard.AccJ(SPEED_RATIO)),
    ]
    for name, result in speed_commands:
        print(f"{name}:", result)
        if dobot.parseResultId(result)[0] != 0:
            print(f"{name} set failed, stop demo")
            return
    print("Speed set to", SPEED_RATIO, "%")

    confirm = input("Input 1 to start motion, other input to exit: ").strip()
    if confirm != "1":
        print("Motion canceled")
        return

    # Check again right before motion in case a new alarm appeared.
    print("Checking robot error information before motion...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before motion.")
        return

    # Start feedback reading so WaitCommandDone can track command completion.
    feed_thread = threading.Thread(target=dobot.GetFeed)
    feed_thread.daemon = True
    feed_thread.start()

    sleep(1)

    # Use the current TCP pose as the circle center.
    center = dobot.GetCurrentPose()
    circle_points = dobot.GenerateXZArcPoints(center, CIRCLE_RADIUS_MM)

    print("圆心:", center)
    print("半径:", CIRCLE_RADIUS_MM, "mm")
    print("XZ 圆最低 Z:", center[2] - CIRCLE_RADIUS_MM, "最高 Z:", center[2] + CIRCLE_RADIUS_MM)

    # Move from the center to the circle edge, then complete the XZ circle with two arcs.
    start_point, top_point, left_point, bottom_point = circle_points
    dobot.RunPoint(start_point)
    dobot.RunArc(top_point, left_point, cp=ARC_BLEND_CP, wait=False)
    dobot.RunArc(bottom_point, start_point)


def main():
    start()
