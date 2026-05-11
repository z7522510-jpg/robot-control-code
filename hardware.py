import threading
from time import sleep

from Dobot import Dobot


def connect_robot(ip):
    dobot = Dobot(ip)
    dobot.connect()
    return dobot


def has_robot_error(dobot):
    error_result = dobot.dashboard.GetErrorID()
    print("GetErrorID:", error_result)

    result_ids = dobot.parseResultId(error_result)
    if not result_ids or result_ids[0] != 0:
        print("Failed to query robot error ID. Stop for safety.")
        return True

    error_ids = result_ids[1:]
    if not error_ids:
        print("Robot status normal, no active error ID")
        return False

    print(f"Found {len(error_ids)} active robot error ID(s):", error_ids)
    return True


def command_succeeded(dobot, command_name, result):
    print(f"{command_name}:", result)
    result_ids = dobot.parseResultId(result)
    if not result_ids or result_ids[0] != 0:
        print(f"{command_name} failed")
        return False
    return True


def enable_robot(dobot):
    print("Checking robot error information before enable...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before EnableRobot.")
        return False

    enable_result = dobot.dashboard.EnableRobot()
    if not command_succeeded(dobot, "EnableRobot", enable_result):
        print("Enable failed: check TCP/IP mode, alarms, E-stop, and port 29999 connection")
        has_robot_error(dobot)
        return False

    print("Enable succeeded")
    return True


def set_robot_speed(dobot, speed_ratio):
    speed_commands = (
        ("SpeedFactor", lambda: dobot.dashboard.SpeedFactor(speed_ratio)),
        ("VelJ", lambda: dobot.dashboard.VelJ(speed_ratio)),
        ("AccJ", lambda: dobot.dashboard.AccJ(speed_ratio)),
    )
    for name, send_command in speed_commands:
        if not command_succeeded(dobot, name, send_command()):
            print(f"{name} set failed, stop experiment")
            return False

    print("Speed set to", speed_ratio, "%")
    return True


def prepare_robot(dobot, speed_ratio):
    if not enable_robot(dobot):
        return False
    if not set_robot_speed(dobot, speed_ratio):
        return False
    return True


def start_feedback(dobot):
    feed_thread = threading.Thread(target=dobot.GetFeed)
    feed_thread.daemon = True
    feed_thread.start()
    return feed_thread


def move_linear_point(dobot, point, speed_ratio):
    move_result = dobot.dashboard.MovL(*point, 0, v=speed_ratio)
    print("MovL:", move_result)
    if not dobot.WaitCommandDone(move_result):
        raise RuntimeError("MovL failed or timed out")
    return True


def set_digital_output(dobot, do_index, value):
    result = dobot.dashboard.DO(do_index, value)
    print(f"DO({do_index},{value}):", result)
    return result


def send_do_pulse(dobot, do_index, pulse_seconds):
    do_on_result = set_digital_output(dobot, do_index, 1)
    sleep(pulse_seconds)
    do_off_result = set_digital_output(dobot, do_index, 0)
    return do_on_result, do_off_result


def turn_do_off(dobot, do_index):
    return set_digital_output(dobot, do_index, 0)


def stop_robot(dobot):
    result = dobot.dashboard.Stop()
    print("Stop:", result)
    return result


def return_to_pose(dobot, saved_pose, speed_ratio, do_indexes=None):
    print("Returning to saved start pose:", saved_pose)
    try:
        stop_robot(dobot)
        sleep(0.2)
    except Exception as error:
        print("Stop failed:", error)

    for do_index in do_indexes or []:
        try:
            turn_do_off(dobot, do_index)
        except Exception as error:
            print(f"DO({do_index},0) failed:", error)

    move_linear_point(dobot, saved_pose, speed_ratio)
    current_pose = dobot.GetCurrentPose()
    print("Returned current pose:", current_pose)
