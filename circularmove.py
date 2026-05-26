#initialize 
#calibration
#loop
#1. calculate next pose
#2. move to next pose
#3. wait
#uutill everything are done, change wavelength

import re
from datetime import datetime
from pathlib import Path

import config

from Dobot import get_robot_error, initialize_robot, stop_and_return
from Laser import connect_laser


CURRENT_POSE_DIR = Path(__file__).with_name("currentpose")
REPORT_POSE_USER_INDEX = 0


def get_circle_center_tool_frame():
    values = _tool_frame_values(config.TOOL_FRAME)
    if len(values) != 6:
        raise ValueError("TOOL_FRAME must have 6 values: x, y, z, rx, ry, rz")

    values[2] += config.CIRCLE_RADIUS_MM
    return "{" + ",".join(str(value) for value in values) + "}"


def create_current_pose_report(start_time):
    CURRENT_POSE_DIR.mkdir(exist_ok=True)
    timestamp = start_time.strftime("%Y%m%d_%H%M%S")
    path = CURRENT_POSE_DIR / f"current_pose_{timestamp}.txt"

    with path.open("w", encoding="utf-8") as file:
        file.write("Circular move current pose report\n")
        file.write(f"Start time: {start_time.isoformat(timespec='seconds')}\n")
        file.write(f"TOOL_INDEX: {config.TOOL_INDEX}\n")
        file.write(f"TOOL_FRAME: {config.TOOL_FRAME}\n")
        file.write(f"CIRCLE_TOOL_INDEX: {config.CIRCLE_TOOL_INDEX}\n")
        file.write(f"Circle center tool frame: {get_circle_center_tool_frame()}\n")
        file.write(f"SUBCUTANEOUS_SCAN_DISTANCE_MM: {config.SUBCUTANEOUS_SCAN_DISTANCE_MM}\n")
        file.write(f"CIRCLE_RADIUS_MM: {config.CIRCLE_RADIUS_MM}\n")
        file.write(
            "Radius move delta along user Z: "
            f"{config.CIRCLE_RADIUS_MM - config.SUBCUTANEOUS_SCAN_DISTANCE_MM}\n"
        )
        file.write(f"CIRCLE_ARC_DEG: {config.CIRCLE_ARC_DEG}\n")
        file.write(f"CIRCLE_TOTAL_STEPS: {config.CIRCLE_TOTAL_STEPS}\n")
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
    values = [float(num) for num in re.findall(r"-?\d+(?:\.\d+)?", recv)]
    if len(values) >= 7 and int(values[0]) == 0:
        return values[1:7]
    if len(values) >= 6:
        return values[:6]
    raise ValueError(f"GetPose(user={REPORT_POSE_USER_INDEX}, tool={config.TOOL_INDEX}) failed: " + recv)


def report_current_pose(dobot, report_path, label, target_pose=None):
    current_pose = get_config_tool_cartesian_pose(dobot)
    line = f"{datetime.now().isoformat(timespec='seconds')} | {label} | current_pose={current_pose}"
    if target_pose is not None:
        line += f" | target_pose={target_pose}"

    print(line)
    with report_path.open("a", encoding="utf-8") as file:
        file.write(line + "\n")

    return current_pose


def change_radius(dobot):
    radius_delta = config.CIRCLE_RADIUS_MM - config.SUBCUTANEOUS_SCAN_DISTANCE_MM
    if abs(radius_delta) < 1e-9:
        print("Radius move skipped: radius already matches subcutaneous distance")
        return None

    dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
    dobot.ActivateTool(config.TOOL_INDEX)
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

    return move_result


def get_circle_tool_cartesian_pose(dobot):
    recv = dobot.dashboard.GetPose(
        user=config.CIRCLE_USER_INDEX,
        tool=config.CIRCLE_TOOL_INDEX,
    )
    print(f"GetPose(user={config.CIRCLE_USER_INDEX}, tool={config.CIRCLE_TOOL_INDEX}):", recv)
    values = [float(num) for num in re.findall(r"-?\d+(?:\.\d+)?", recv)]
    if len(values) >= 7 and int(values[0]) == 0:
        return values[1:7]
    if len(values) >= 6:
        return values[:6]
    raise ValueError(
        f"GetPose(user={config.CIRCLE_USER_INDEX}, tool={config.CIRCLE_TOOL_INDEX}) failed: " + recv
    )


def initialize_devices(report_path):
    # Step 1: connect laser + robot, activate the configured tool, and record
    # the real tool start pose. No circular-motion setup or arm move yet.
    laser = connect_laser(config.LASER_DLL_PATH)
    try:
        laser.initialize_laser(config.LASER_WAVELENGTH_NM)
        dobot, feed_thread, _ = initialize_robot(
            config.DOBOT_IP,
            config.SPEED_RATIO,
        )

        dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
        dobot.ActivateTool(config.TOOL_INDEX)
        real_start_pose = get_config_tool_cartesian_pose(dobot)
        print("Real tool start pose:", real_start_pose)
        report_current_pose(dobot, report_path, "real_tool_start_pose", real_start_pose)
    except Exception:
        # Release the laser so the next attempt doesn't see "already connected" (err 17).
        try:
            laser.close()
        except Exception:
            pass
        raise
    return laser, dobot, feed_thread, real_start_pose


def set_radius(dobot, report_path):
    # Step 2: move the arm by the radius delta, then build the virtual
    # circle-center tool frame and read the circle center pose.
    change_radius(dobot)
    radius_pose = get_config_tool_cartesian_pose(dobot)
    print("Radius-adjusted real tool pose:", radius_pose)
    report_current_pose(dobot, report_path, "radius_adjusted_pose", radius_pose)

    circle_tool_frame = get_circle_center_tool_frame()
    dobot.SetTool(config.CIRCLE_TOOL_INDEX, circle_tool_frame)
    dobot.ActivateTool(config.CIRCLE_TOOL_INDEX)
    center_pose = get_circle_tool_cartesian_pose(dobot)
    print("Circle center pose:", center_pose)
    return radius_pose, center_pose, circle_tool_frame


def initialize(report_path):
    laser, dobot, feed_thread, real_start_pose = initialize_devices(report_path)
    try:
        radius_pose, center_pose, circle_tool_frame = set_radius(dobot, report_path)
    except Exception:
        try:
            laser.close()
        except Exception:
            pass
        raise
    return laser, dobot, feed_thread, real_start_pose, radius_pose, center_pose, circle_tool_frame


def _tool_frame_values(tool_frame):
    return [
        float(value.strip())
        for value in tool_frame.strip("{}").split(",")
        if value.strip()
    ]


def run_step(dobot, pose, user, tool, acceleration, velocity, cp, circle_tool_frame):
    dobot.SetTool(tool, circle_tool_frame)
    dobot.ActivateTool(tool)
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
    if not dobot.WaitCommandDone(move_result):
        raise RuntimeError("MovJ failed or timed out")

    return move_result


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


def generate_tool_center_circle_poses(
    center_pose,
    total_steps,
    arc_deg,
):
    center_x = center_pose[0]
    center_y = center_pose[1]
    center_z = center_pose[2]
    center_rx = center_pose[3]
    center_ry = center_pose[4]
    center_rz = center_pose[5]

    poses = []
    start_offset = -arc_deg / 2
    angle_step = arc_deg / total_steps

    for step_index in range(total_steps + 1):
        ry = center_ry + start_offset + step_index * angle_step
        poses.append([center_x, center_y, center_z, center_rx, ry, center_rz])

    return poses


def run_experiment():
    ask_subcutaneous_scan_distance()
    ask_circle_radius()
    total_steps = ask_circle_total_steps()

    start_time = datetime.now()
    report_path = create_current_pose_report(start_time)
    (
        laser,
        dobot,
        feed_thread,
        real_start_pose,
        radius_pose,
        center_pose,
        circle_tool_frame,
    ) = initialize(report_path)

    user = config.CIRCLE_USER_INDEX
    tool = config.CIRCLE_TOOL_INDEX
    acceleration = config.CIRCLE_ACCELERATION_RATIO
    velocity = config.CIRCLE_VELOCITY_RATIO
    cp = config.CIRCLE_CP

    poses = generate_tool_center_circle_poses(
        center_pose,
        total_steps=total_steps,
        arc_deg=config.CIRCLE_ARC_DEG,
    )

    if get_robot_error(dobot):
        stop_and_return(dobot, center_pose, config.SPEED_RATIO)
        return laser, dobot, feed_thread, real_start_pose, poses

    try:
        if get_robot_error(dobot):
            stop_and_return(dobot, center_pose, config.SPEED_RATIO)
            return laser, dobot, feed_thread, real_start_pose, poses

        start_pose = poses[0]
        print("Move to start pose:", start_pose)
        run_step(
            dobot,
            start_pose,
            user=user,
            tool=tool,
            acceleration=acceleration,
            velocity=velocity,
            cp=cp,
            circle_tool_frame=circle_tool_frame,
        )
        report_current_pose(dobot, report_path, "start pose", start_pose)

        for index, pose in enumerate(poses[1:], start=2):
            if get_robot_error(dobot):
                stop_and_return(dobot, center_pose, config.SPEED_RATIO)
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
                circle_tool_frame=circle_tool_frame,
            )
            report_current_pose(dobot, report_path, f"circle point {index}/{len(poses)}", pose)

        print("Return to radius-adjusted vertical pose:", radius_pose)
        dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
        dobot.ActivateTool(config.TOOL_INDEX)
        move_result = dobot.dashboard.MovJ(
            *radius_pose,
            0,
            user=config.CIRCLE_USER_INDEX,
            tool=config.TOOL_INDEX,
            a=acceleration,
            v=velocity,
            cp=cp,
        )
        print("MovJ return radius pose:", move_result)
        if not dobot.WaitCommandDone(move_result):
            raise RuntimeError("Return to radius-adjusted pose failed or timed out")
        report_current_pose(dobot, report_path, "return radius_adjusted_pose", radius_pose)

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
