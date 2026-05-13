import threading
import time
from time import sleep
from tracemalloc import stop

from .dobot import Dobot


def calculate_step_count(total_distance_mm, step_distance_mm):
    # Calculate how many steps are needed from the total distance and step size.
    step_count = int(total_distance_mm / step_distance_mm)
    total_distance = step_count * step_distance_mm
    if abs(total_distance - total_distance_mm) > 1e-9:
        raise ValueError("total_distance_mm must be divisible by step_distance_mm")
    return step_count


def build_xyz_step_target(start_pose, step_index, step_offset_mm):
    # Build a 3D step target. X/Y/Z move by the per-step offset multiplied by
    # step_index; Rx/Ry/Rz stay the same as the start pose.
    if len(start_pose) < 6:
        raise ValueError("start_pose must contain X, Y, Z, Rx, Ry, and Rz")
    if len(step_offset_mm) != 3:
        raise ValueError("step_offset_mm must contain dx, dy, and dz")

    target_pose = start_pose.copy()
    target_pose[0] = start_pose[0] + step_offset_mm[0] * step_index
    target_pose[1] = start_pose[1] + step_offset_mm[1] * step_index
    target_pose[2] = start_pose[2] + step_offset_mm[2] * step_index
    return target_pose


def run_xyz_step_cycle(
    dobot,
    start_pose,
    step_index,
    speed_ratio,
    step_offset_mm,
    step_wait_seconds,
    trigger_do_index,
    trigger_pulse_seconds,
    loop_start_time=None,
):
    # Record when DO turns on so the experiment can report trigger timing.
    on_time_ms = None
    if loop_start_time is not None:
        on_time_ms = int((time.perf_counter() - loop_start_time) * 1000)

    # Send one DO pulse to the external device: on, wait, then off.
    do_on_result, do_off_result = dobot.SendDOPulse(
        trigger_do_index,
        trigger_pulse_seconds,
    )

    # Record when DO turns off.
    off_time_ms = None
    if loop_start_time is not None:
        off_time_ms = int((time.perf_counter() - loop_start_time) * 1000)

    # Calculate this step's 3D target pose, then move there with a linear MovL.
    target_pose = build_xyz_step_target(
        start_pose,
        step_index,
        step_offset_mm,
    )
    dobot.MoveLinearPoint(target_pose, speed_ratio)
    sleep(step_wait_seconds)

    # Return this step's DO result and timing record for experiment.py.
    return {
        "step": step_index,
        "signal": f"DO({trigger_do_index},1)->DO({trigger_do_index},0)",
        "on_time_ms": on_time_ms,
        "off_time_ms": off_time_ms,
        "on_result": do_on_result,
        "off_result": do_off_result,
    }


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
    # Standard robot startup for this experiment: enable first, then set speed.
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
    # Blocking linear move. WaitCommandDone keeps the next command from
    # starting before this motion reaches its target.
    move_result = dobot.dashboard.MovL(*point, 0, v=speed_ratio)
    print("MovL:", move_result)
    if not dobot.WaitCommandDone(move_result):
        raise RuntimeError("MovL failed or timed out")
    return True


def move_relative_xyz(dobot, dx=0, dy=0, dz=0, speed_ratio=30):
    # Manual jog helper for notebook use. It changes only X/Y/Z and keeps the
    # current orientation fields untouched.
    current_pose = dobot.GetCurrentPose()
    target_pose = build_xyz_step_target(current_pose, 1, (dx, dy, dz))

    print("Current pose:", current_pose)
    print("Target pose:", target_pose)
    move_linear_point(dobot, target_pose, speed_ratio)
    return target_pose


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


def disconnect_robot(dobot):
    if dobot is None:
        return

    try:
        disable_result = dobot.dashboard.DisableRobot()
        print("DisableRobot:", disable_result)
    except Exception as error:
        print("DisableRobot failed:", error)

    try:
        if dobot.dashboard is not None:
            dobot.dashboard.close()
    except Exception as error:
        print("Dashboard close failed:", error)

    try:
        if dobot.feedFour is not None:
            dobot.feedFour.close()
    except Exception as error:
        print("Feedback close failed:", error)

    print("Dobot disconnected")


def return_to_pose(dobot, saved_pose, speed_ratio, do_indexes=None):
    # Recovery helper: stop queued motion, turn selected DO channels off, then
    # return to the saved original pose.
    print("Returning to saved start pose:", saved_pose)
    try:
        stop_robot(dobot)
        sleep(0.2)
    except Exception as error:
        print("Stop failed:", error)

    for do_index in do_indexes or []:
        try:
            turn_do_off(dobot, do_index)
            stop(.2)
        except Exception as error:
            print(f"DO({do_index},0) failed:", error)

    move_linear_point(dobot, saved_pose, speed_ratio)
    current_pose = dobot.GetCurrentPose()
    print("Returned current pose:", current_pose)
