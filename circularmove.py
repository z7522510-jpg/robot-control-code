#initialize 
#calibration
#loop
#1. calculate next pose
#2. move to next pose
#3. wait
#uutill everything are done, change wavelength

import math
import re
from datetime import datetime
from pathlib import Path
from time import sleep

import config

from Dobot import get_robot_error, initialize_robot, stop_and_return
from Laser import connect_laser


CURRENT_POSE_DIR = Path(__file__).with_name("currentpose")
REPORT_POSE_USER_INDEX = 0
REPORT_POSE_TOOL_INDEX = 0


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
        file.write(f"Circle center tool frame: {get_circle_center_tool_frame()}\n")
        file.write(f"CIRCLE_RADIUS_MM: {config.CIRCLE_RADIUS_MM}\n")
        file.write(f"CIRCLE_END_DEG: {config.CIRCLE_END_DEG}\n")
        file.write(f"CIRCLE_TOTAL_STEPS: {config.CIRCLE_TOTAL_STEPS}\n")
        file.write(
            "Reported current pose frame: "
            f"user={REPORT_POSE_USER_INDEX}, tool={REPORT_POSE_TOOL_INDEX}\n"
        )
        file.write("\n")

    print("Current pose report:", path)
    return path


def get_default_tool_cartesian_pose(dobot):
    recv = dobot.dashboard.GetPose(
        user=REPORT_POSE_USER_INDEX,
        tool=REPORT_POSE_TOOL_INDEX,
    )
    print("GetPose(user=0, tool=0):", recv)
    values = [float(num) for num in re.findall(r"-?\d+(?:\.\d+)?", recv)]
    if len(values) >= 7 and int(values[0]) == 0:
        return values[1:7]
    if len(values) >= 6:
        return values[:6]
    raise ValueError("GetPose(user=0, tool=0) failed: " + recv)


def report_current_pose(dobot, report_path, label, target_pose=None):
    current_pose = get_default_tool_cartesian_pose(dobot)
    line = f"{datetime.now().isoformat(timespec='seconds')} | {label} | current_pose={current_pose}"
    if target_pose is not None:
        line += f" | target_pose={target_pose}"

    print(line)
    with report_path.open("a", encoding="utf-8") as file:
        file.write(line + "\n")

    return current_pose


def initialize():
    laser = connect_laser(config.LASER_DLL_PATH)
    try:
        laser.initialize_laser(config.LASER_WAVELENGTH_NM)
        dobot, feed_thread, _ = initialize_robot(
            config.DOBOT_IP,
            config.SPEED_RATIO,
        )

        # For circular motion, the active TCP is a virtual point below the
        # original tool end. Holding this point fixed lets the real tool end
        # swing around it as rx/ry/rz changes.
        circle_tool_frame = get_circle_center_tool_frame()
        dobot.SetTool(config.TOOL_INDEX, circle_tool_frame)
        dobot.ActivateTool(config.TOOL_INDEX)

        saved_start_pose = dobot.GetCurrentPose()
        print("Saved start pose:", saved_start_pose)

        initial_pose = get_initial_pose(dobot, saved_start_pose)
    except Exception:
        # Release the laser so the next attempt doesn't see "already connected" (err 17).
        try:
            laser.close()
        except Exception:
            pass
        raise
    return laser, dobot, feed_thread, saved_start_pose, initial_pose


def get_initial_pose(dobot, saved_start_pose):
    target_pose = list(saved_start_pose)
    target_pose[3] = config.CIRCLE_RX_DEG
    target_pose[4] = config.CIRCLE_START_RY_DEG
    target_pose[5] = config.CIRCLE_RZ_DEG
    run_step(
        dobot,
        target_pose,
        config.CIRCLE_USER_INDEX,
        config.TOOL_INDEX,
        config.CIRCLE_ACCELERATION_RATIO,
        config.CIRCLE_VELOCITY_RATIO,
        config.CIRCLE_CP,
    )

    initial_pose = dobot.GetCurrentPose()
    print("Initial pose:", initial_pose)
    return initial_pose


def _tool_frame_values(tool_frame):
    return [
        float(value.strip())
        for value in tool_frame.strip("{}").split(",")
        if value.strip()
    ]


def ask_tool_coordinates():
    current_values = _tool_frame_values(config.TOOL_FRAME)
    labels = ["x", "y", "z", "rx", "ry", "rz"]
    values = []

    print("Input tool coordinates: x, y, z, rx, ry, rz")
    for label, current_value in zip(labels, current_values):
        value = input(f"{label} [{current_value}]: ").strip()
        values.append(float(value) if value else current_value)

    if len(values) != 6:
        raise ValueError("Tool coordinates must have 6 values: x, y, z, rx, ry, rz")

    config.TOOL_FRAME = "{" + ",".join(str(value) for value in values) + "}"
    print("TOOL_FRAME =", config.TOOL_FRAME)
    return config.TOOL_FRAME


def calibration(dobot):
    tool_frame = ask_tool_coordinates()
    set_tool_result = dobot.SetTool(config.TOOL_INDEX, tool_frame)
    activate_result = dobot.ActivateTool(config.TOOL_INDEX)

    print("SetTool result:", set_tool_result)
    print("ActivateTool result:", activate_result)
    return set_tool_result, activate_result


def run_step(dobot, pose, user, tool, acceleration, velocity, cp):
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


def ask_circle_end_angle():
    end_angle = float(input(f"Circle end angle degrees [{config.CIRCLE_END_DEG}]: ") or config.CIRCLE_END_DEG)
    if end_angle <= 0:
        raise ValueError("Circle end angle must be greater than 0")

    config.CIRCLE_END_DEG = end_angle
    print("CIRCLE_END_DEG =", config.CIRCLE_END_DEG)
    return end_angle


def generate_xz_circle_poses(
    initial_pose,
    radius,
    angle_step_deg,
    end_angle_deg,
    rx,
    start_ry,
    rz,
):
    initial_x = initial_pose[0]
    fixed_y = initial_pose[1]
    initial_z = initial_pose[2]

    poses = []
    current_angle = 0
    angle_ry = start_ry
    end_angle = math.radians(end_angle_deg)
    angle_step = math.radians(angle_step_deg)

    while current_angle < end_angle:
        x = initial_x + radius * math.sin(current_angle)
        z = initial_z + radius * math.cos(current_angle) - radius
        poses.append([x, fixed_y, z, rx, angle_ry, rz])

        angle_ry += angle_step_deg
        current_angle += angle_step

    return poses


def generate_tool_center_circle_poses(
    initial_pose,
    angle_step_deg,
    end_angle_deg,
    rx,
    start_ry,
    rz,
):
    center_x = initial_pose[0]
    center_y = initial_pose[1]
    center_z = initial_pose[2]

    poses = []
    current_angle = 0

    while current_angle < end_angle_deg:
        ry = start_ry + current_angle
        poses.append([center_x, center_y, center_z, rx, ry, rz])
        current_angle += angle_step_deg

    return poses


def run_experiment():
    # Ask circle params before initialize(): the radius is baked into the
    # virtual circle-center tool frame that initialize() sets up.
    ask_circle_radius()
    end_angle_deg = ask_circle_end_angle()
    total_steps = ask_circle_total_steps()
    angle_step_deg = end_angle_deg / total_steps

    start_time = datetime.now()
    report_path = create_current_pose_report(start_time)
    laser, dobot, feed_thread, saved_start_pose, initial_pose = initialize()
    report_current_pose(dobot, report_path, "initial_pose", initial_pose)

    user = config.CIRCLE_USER_INDEX
    tool = config.TOOL_INDEX
    acceleration = config.CIRCLE_ACCELERATION_RATIO
    velocity = config.CIRCLE_VELOCITY_RATIO
    cp = config.CIRCLE_CP

    poses = generate_tool_center_circle_poses(
        initial_pose,
        angle_step_deg=angle_step_deg,
        end_angle_deg=end_angle_deg,
        rx=config.CIRCLE_RX_DEG,
        start_ry=config.CIRCLE_START_RY_DEG,
        rz=config.CIRCLE_RZ_DEG,
    )

    if get_robot_error(dobot):
        stop_and_return(dobot, saved_start_pose, config.SPEED_RATIO)
        return laser, dobot, feed_thread, saved_start_pose, poses

    # Virtual circle-center tool frame is already set and active from initialize().

    for loop_index in range(1, config.LOOP_REPEAT_COUNT + 1):
        if get_robot_error(dobot):
            stop_and_return(dobot, saved_start_pose, config.SPEED_RATIO)
            return laser, dobot, feed_thread, saved_start_pose, poses

        print(f"Loop {loop_index}/{config.LOOP_REPEAT_COUNT}")

        for index, pose in enumerate(poses, start=1):
            if get_robot_error(dobot):
                stop_and_return(dobot, saved_start_pose, config.SPEED_RATIO)
                return laser, dobot, feed_thread, saved_start_pose, poses

            print(f"Circle point {index}/{len(poses)}:", pose)
            run_step(
                dobot,
                pose,
                user=user,
                tool=tool,
                acceleration=acceleration,
                velocity=velocity,
                cp=cp,
            )
            report_current_pose(
                dobot,
                report_path,
                f"loop {loop_index} circle point {index}/{len(poses)}",
                pose,
            )

        if get_robot_error(dobot):
            stop_and_return(dobot, saved_start_pose, config.SPEED_RATIO)
            return laser, dobot, feed_thread, saved_start_pose, poses

        run_step(
            dobot,
            initial_pose,
            user=user,
            tool=tool,
            acceleration=acceleration,
            velocity=velocity,
            cp=cp,
        )
        report_current_pose(dobot, report_path, f"loop {loop_index} return initial", initial_pose)
        sleep(2)

    with report_path.open("a", encoding="utf-8") as file:
        file.write(f"\nFinish time: {datetime.now().isoformat(timespec='seconds')}\n")

    return laser, dobot, feed_thread, saved_start_pose, poses


if __name__ == "__main__":
    laser = None
    try:
        laser = run_experiment()[0]
    finally:
        if laser is not None:
            laser.close()
