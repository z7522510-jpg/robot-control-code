#initialize 
#calibration
#loop
#1. calculate next pose
#2. move to next pose
#3. wait
#uutill everything are done, change wavelength

import re
import time
from datetime import datetime
from pathlib import Path

import config
import matrix_calculation

from Dobot import get_robot_error, initialize_robot, stop_and_return
from Laser import connect_laser


CURRENT_POSE_DIR = Path(__file__).with_name("currentpose")
REPORT_POSE_USER_INDEX = 0


def calculate_matrix_circle_center_pose(radius_pose):
    center_position = matrix_calculation.offset_position_from_pose(
        radius_pose,
        [0, 0, config.CIRCLE_RADIUS_MM],
    )
    return [
        center_position[0],
        center_position[1],
        center_position[2],
        radius_pose[3],
        radius_pose[4],
        radius_pose[5],
    ]


def create_current_pose_report(start_time):
    CURRENT_POSE_DIR.mkdir(exist_ok=True)
    timestamp = start_time.strftime("%Y%m%d_%H%M%S")
    path = CURRENT_POSE_DIR / f"current_pose_{timestamp}.txt"

    with path.open("w", encoding="utf-8") as file:
        file.write("Circular move current pose report\n")
        file.write(f"Start time: {start_time.isoformat(timespec='seconds')}\n")
        file.write(f"TOOL_INDEX: {config.TOOL_INDEX}\n")
        file.write(f"TOOL_FRAME: {config.TOOL_FRAME}\n")
        file.write("Circle center mode: matrix calculation from real tool pose\n")
        file.write("Circle center offset in real tool frame: {0,0,CIRCLE_RADIUS_MM}\n")
        file.write(f"SUBCUTANEOUS_SCAN_DISTANCE_MM: {config.SUBCUTANEOUS_SCAN_DISTANCE_MM}\n")
        file.write(f"CIRCLE_RADIUS_MM: {config.CIRCLE_RADIUS_MM}\n")
        file.write(
            "Radius move delta along user Z: "
            f"{config.CIRCLE_RADIUS_MM - config.SUBCUTANEOUS_SCAN_DISTANCE_MM}\n"
        )
        file.write(f"CIRCLE_ARC_DEG: {config.CIRCLE_ARC_DEG}\n")
        file.write(f"CIRCLE_TOTAL_STEPS: {config.CIRCLE_TOTAL_STEPS}\n")
        file.write(f"CIRCLE_APPROACH_OFFSET_MM: {config.CIRCLE_APPROACH_OFFSET_MM}\n")
        file.write(
            "Reported current pose frame: "
            f"user={REPORT_POSE_USER_INDEX}, tool={config.TOOL_INDEX}, "
            f"tool_frame={config.TOOL_FRAME}\n"
        )
        file.write("\n")

    print("Current pose report:", path)
    return path


def get_config_tool_cartesian_pose(dobot):
    recv = dobot.dashboard.GetPose(
        user=REPORT_POSE_USER_INDEX,
        tool=config.TOOL_INDEX,
    )
    print(f"GetPose(user={REPORT_POSE_USER_INDEX}, tool={config.TOOL_INDEX}):", recv)
    return _pose_from_get_pose(recv)


def _pose_from_get_pose(recv):
    values = [float(num) for num in re.findall(r"-?\d+(?:\.\d+)?", recv)]
    if len(values) < 7 or int(values[0]) != 0:
        raise ValueError("GetPose failed: " + recv)
    return values[1:7]


def level_xz_plane(dobot):
    # Set rx=180, ry=0 (keep x/y/z and rz) so the probe points straight down and
    # the circular scan starts from a level orientation. Operates in the
    # configured tool frame (TOOL_INDEX), which run_experiment activates once.
    user = REPORT_POSE_USER_INDEX
    tool = config.TOOL_INDEX
    recv = dobot.dashboard.GetPose(user=user, tool=tool)
    print(f"GetPose(user={user}, tool={tool}):", recv)
    pose = _pose_from_get_pose(recv)

    pose[3] = 180.0
    pose[4] = 0.0
    move_result = dobot.dashboard.MovJ(
        *pose,
        0,
        user=user,
        tool=tool,
        a=config.CIRCLE_ACCELERATION_RATIO,
        v=config.CIRCLE_VELOCITY_RATIO,
        cp=config.CIRCLE_CP,
    )
    print("Level probe orientation (rx=180, ry=0):", move_result)
    if not dobot.WaitCommandDone(move_result):
        raise RuntimeError("Level probe orientation move failed or timed out")
    return pose


def initialize_devices():
    laser = connect_laser(config.LASER_DLL_PATH)
    try:
        laser.initialize_laser(config.LASER_WAVELENGTH_NM)
        dobot, feed_thread, _ = initialize_robot(
            config.DOBOT_IP,
            config.SPEED_RATIO,
        )
    except Exception:
        # Release the laser so the next attempt doesn't see "already connected" (err 17).
        try:
            laser.close()
        except Exception:
            pass
        raise
    return laser, dobot, feed_thread


def set_radius(dobot, report_path):
    # Step 2: move the real tool by the radius delta, then calculate the circle
    # center from the real tool pose. No virtual circle-center tool is created.
    radius_delta = config.CIRCLE_RADIUS_MM - config.SUBCUTANEOUS_SCAN_DISTANCE_MM
    if abs(radius_delta) < 1e-9:
        print("Radius move skipped: radius already matches subcutaneous distance")
    else:
        move_result = dobot.dashboard.RelMovLUser(
            0,
            0,
            radius_delta,
            0,
            0,
            0,
            user=config.CIRCLE_USER_INDEX,
            tool=config.TOOL_INDEX,
            v=config.CIRCLE_VELOCITY_RATIO,
        )
        print("RelMovLUser radius move:", move_result)
        if not dobot.WaitCommandDone(move_result):
            raise RuntimeError("RelMovLUser radius move failed or timed out")

    radius_pose = get_config_tool_cartesian_pose(dobot)
    print("Radius-adjusted real tool pose:", radius_pose)
    line = (
        f"{datetime.now().isoformat(timespec='seconds')} | "
        f"radius_adjusted_pose | current_pose={radius_pose}"
    )
    print(line)
    with report_path.open("a", encoding="utf-8") as file:
        file.write(line + "\n")

    center_pose = calculate_matrix_circle_center_pose(radius_pose)
    print("Matrix circle center pose:", center_pose)
    return radius_pose, center_pose


def run_step(
    dobot,
    pose,
    user,
    tool,
    acceleration,
    velocity,
    cp,
    stop_event=None,
    trigger_do_index=None,
    trigger_pulse_seconds=None,
):
    move_result = dobot.dashboard.MovJ(
        *pose,
        0,
        user=user,
        tool=tool,
        a=acceleration,
        v=velocity,
        cp=cp,
    )
    print("MovJ:", move_result)

    if trigger_do_index is not None and trigger_pulse_seconds is not None:
        pulse_ms = int(round(trigger_pulse_seconds * 1000))
        do_result = dobot.dashboard.DO(trigger_do_index, 1, pulse_ms)
        print(f"DO({trigger_do_index},1,{pulse_ms}ms):", do_result)

    if stop_event is None:
        done = dobot.WaitCommandDone(move_result)
    else:
        done = wait_command_done_or_stop(dobot, move_result, stop_event)
    if not done:
        raise RuntimeError("MovJ failed or timed out")

    return move_result


def wait_command_done_or_stop(dobot, move_result, stop_event, timeout=30):
    result_ids = dobot.parseResultId(move_result)
    print(result_ids)
    if len(result_ids) < 2 or result_ids[0] != 0:
        print("Command failed, skip waiting:", move_result)
        return False

    current_command_id = result_ids[1]
    print("Command ID:", current_command_id)
    start_time = time.perf_counter()
    last_print_time = start_time

    while True:
        if stop_event.is_set():
            print("Stop requested while waiting for motion")
            return False

        now = time.perf_counter()
        if dobot.feedData.robotMode == 5 and dobot.feedData.robotCurrentCommandID >= current_command_id:
            print("Motion done")
            return True

        # robotMode 9 = ROBOT_MODE_ERROR: the queue won't advance, so stop
        # waiting immediately and surface the real alarm instead of spinning.
        if dobot.feedData.robotMode == 9:
            print("Robot entered ERROR mode (9) while waiting. Active alarm:")
            get_robot_error(dobot)
            return False

        if now - last_print_time >= 1:
            print(
                "Waiting motion done: "
                f"mode={dobot.feedData.robotMode}, "
                f"currentCommandID={dobot.feedData.robotCurrentCommandID}, "
                f"targetCommandID={current_command_id}"
            )
            last_print_time = now

        if now - start_time >= timeout:
            print(
                "Waiting motion done timeout: "
                f"mode={dobot.feedData.robotMode}, "
                f"currentCommandID={dobot.feedData.robotCurrentCommandID}, "
                f"targetCommandID={current_command_id}"
            )
            return False

        time.sleep(0.005)


def ask_subcutaneous_scan_distance():
    distance = float(
        input(
            "Subcutaneous scan distance mm "
            f"[{config.SUBCUTANEOUS_SCAN_DISTANCE_MM}]: "
        ) or config.SUBCUTANEOUS_SCAN_DISTANCE_MM
    )
    if distance <= 0:
        raise ValueError("Subcutaneous scan distance must be greater than 0")

    config.SUBCUTANEOUS_SCAN_DISTANCE_MM = distance
    print("SUBCUTANEOUS_SCAN_DISTANCE_MM =", config.SUBCUTANEOUS_SCAN_DISTANCE_MM)
    return distance


def ask_circle_radius():
    radius = float(input(f"Circle radius mm [{config.CIRCLE_RADIUS_MM}]: ") or config.CIRCLE_RADIUS_MM)
    if radius <= 0:
        raise ValueError("Circle radius must be greater than 0")

    config.CIRCLE_RADIUS_MM = radius
    print("CIRCLE_RADIUS_MM =", config.CIRCLE_RADIUS_MM)
    return radius


def ask_circle_total_steps():
    total_steps = int(input(f"Circle total steps [{config.CIRCLE_TOTAL_STEPS}]: ") or config.CIRCLE_TOTAL_STEPS)
    if total_steps <= 0:
        raise ValueError("Circle total steps must be greater than 0")

    config.CIRCLE_TOTAL_STEPS = total_steps
    print("CIRCLE_TOTAL_STEPS =", config.CIRCLE_TOTAL_STEPS)
    return total_steps


def ask_circle_approach_offset():
    offset = float(
        input(
            "Approach offset mm (Y) "
            f"[{config.CIRCLE_APPROACH_OFFSET_MM}]: "
        ) or config.CIRCLE_APPROACH_OFFSET_MM
    )
    config.CIRCLE_APPROACH_OFFSET_MM = offset
    print("CIRCLE_APPROACH_OFFSET_MM =", config.CIRCLE_APPROACH_OFFSET_MM)
    return offset


def generate_matrix_circle_poses(
    center_pose,
    total_steps,
    arc_deg,
    start_pose,
    axis="rx",
):
    return matrix_calculation.generate_real_tool_circle_poses(
        start_pose,
        center_pose[:3],
        total_steps=total_steps,
        arc_deg=arc_deg,
        axis=axis,
    )


def run_experiment():
    ask_subcutaneous_scan_distance()
    ask_circle_radius()
    total_steps = ask_circle_total_steps()
    ask_circle_approach_offset()

    start_time = datetime.now()
    report_path = create_current_pose_report(start_time)
    laser, dobot, feed_thread = initialize_devices()

    user = config.CIRCLE_USER_INDEX
    tool = config.TOOL_INDEX
    acceleration = config.CIRCLE_ACCELERATION_RATIO
    velocity = config.CIRCLE_VELOCITY_RATIO
    cp = config.CIRCLE_CP

    try:
        dobot.SetTool(tool, config.TOOL_FRAME)
        dobot.ActivateTool(tool)

        real_start_pose = get_config_tool_cartesian_pose(dobot)
        print("Real tool start pose:", real_start_pose)
        line = (
            f"{datetime.now().isoformat(timespec='seconds')} | "
            f"real_tool_start_pose | current_pose={real_start_pose}"
        )
        print(line)
        with report_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

        level_xz_plane(dobot)
        radius_pose, center_pose = set_radius(dobot, report_path)
    except Exception:
        try:
            laser.close()
        except Exception:
            pass
        raise

    center_pose = list(center_pose)
    radius_pose = list(radius_pose)

    poses = generate_matrix_circle_poses(
        center_pose,
        total_steps=total_steps,
        arc_deg=config.CIRCLE_ARC_DEG,
        start_pose=radius_pose,
        axis=getattr(config, "CIRCLE_ROTATION_AXIS", "rx"),
    )

    try:
        if get_robot_error(dobot):
            stop_and_return(dobot, radius_pose, config.SPEED_RATIO)
            return laser, dobot, feed_thread, real_start_pose, poses

        start_pose = poses[0]

        # Step 1: linear sidestep along user Y so the subsequent rotation
        # passes through clear space, not over the subject.
        print(f"Sidestep Y by -{config.CIRCLE_APPROACH_OFFSET_MM} mm")
        sidestep_result = dobot.dashboard.RelMovLUser(
            0, -config.CIRCLE_APPROACH_OFFSET_MM, 0, 0, 0, 0,
            user=user,
            tool=tool,
            v=velocity,
        )
        print("Sidestep:", sidestep_result)
        if not dobot.WaitCommandDone(sidestep_result):
            raise RuntimeError("Sidestep before start pose failed or timed out")

        # Step 2: MovJ to start_pose from the sidestep position.
        print("Move to start pose:", start_pose)
        move_result = dobot.dashboard.MovJ(
            *start_pose,
            0,
            user=user,
            tool=tool,
            a=acceleration,
            v=velocity,
            cp=cp,
        )
        print("MovJ start:", move_result)
        if config.TRIGGER_DO_INDEX is not None and config.TRIGGER_PULSE_SECONDS is not None:
            pulse_ms = int(round(config.TRIGGER_PULSE_SECONDS * 1000))
            do_result = dobot.dashboard.DO(config.TRIGGER_DO_INDEX, 1, pulse_ms)
            print(f"DO({config.TRIGGER_DO_INDEX},1,{pulse_ms}ms):", do_result)
        if not dobot.WaitCommandDone(move_result):
            raise RuntimeError("Move to start pose failed or timed out")
        current_pose = get_config_tool_cartesian_pose(dobot)
        line = (
            f"{datetime.now().isoformat(timespec='seconds')} | "
            f"start pose | current_pose={current_pose} | target_pose={start_pose}"
        )
        print(line)
        with report_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

        for index, pose in enumerate(poses[1:], start=2):
            if get_robot_error(dobot):
                stop_and_return(dobot, radius_pose, config.SPEED_RATIO)
                return laser, dobot, feed_thread, real_start_pose, poses

            print(f"Circle point {index}/{len(poses)}:", pose)
            run_step(
                dobot,
                pose,
                user=user,
                tool=tool,
                acceleration=acceleration,
                velocity=velocity,
                cp=cp,
                trigger_do_index=config.TRIGGER_DO_INDEX,
                trigger_pulse_seconds=config.TRIGGER_PULSE_SECONDS,
            )
            current_pose = get_config_tool_cartesian_pose(dobot)
            line = (
                f"{datetime.now().isoformat(timespec='seconds')} | "
                f"circle point {index}/{len(poses)} | "
                f"current_pose={current_pose} | target_pose={pose}"
            )
            print(line)
            with report_path.open("a", encoding="utf-8") as file:
                file.write(line + "\n")
        # Same approach as entering start: sidestep Y first, then MovJ.
        print(f"Sidestep Y by {config.CIRCLE_APPROACH_OFFSET_MM} mm")
        sidestep_result = dobot.dashboard.RelMovLUser(
            0, config.CIRCLE_APPROACH_OFFSET_MM, 0, 0, 0, 0,
            user=user,
            tool=tool,
            v=velocity,
        )
        print("Sidestep:", sidestep_result)
        if not dobot.WaitCommandDone(sidestep_result):
            raise RuntimeError("Sidestep before center pose failed or timed out")

        print("Move back to radius-adjusted real tool pose:", radius_pose)
        move_result = dobot.dashboard.MovJ(
            *radius_pose,
            0,
            user=user,
            tool=tool,
            a=acceleration,
            v=velocity,
            cp=cp,
        )
        print("MovJ back to center:", move_result)
        if not dobot.WaitCommandDone(move_result):
            raise RuntimeError("Move back to radius-adjusted pose failed or timed out")
        current_pose = get_config_tool_cartesian_pose(dobot)
        line = (
            f"{datetime.now().isoformat(timespec='seconds')} | "
            "return radius_adjusted_pose | "
            f"current_pose={current_pose} | target_pose={radius_pose}"
        )
        print(line)
        with report_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

    finally:
        with report_path.open("a", encoding="utf-8") as file:
            file.write(f"\nFinish time: {datetime.now().isoformat(timespec='seconds')}\n")

    return laser, dobot, feed_thread, real_start_pose, poses


if __name__ == "__main__":
    laser = None
    try:
        laser = run_experiment()[0]
    finally:
        if laser is not None:
            laser.close()
