#initialize 
#calibration
#loop
#1. calculate next pose
#2. move to next pose
#3. wait
#uutill everything are done, change wavelength

import math
import re

import config

from Dobot import get_robot_error, initialize_robot


def initialize():
    return initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )


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


def get_pose(dobot, user=0, tool=0):
    recv = dobot.dashboard.GetPose(user=user, tool=tool)
    print("GetPose:", recv)
    values = [float(num) for num in re.findall(r"-?\d+(?:\.\d+)?", recv)]
    if len(values) >= 7 and int(values[0]) == 0:
        return values[1:7]
    raise ValueError("GetPose failed: " + recv)


def generate_xz_circle_poses(
    initial_pose,
    radius=350,
    angle_step_deg=5,
    end_angle_deg=90,
    rx=180,
    start_ry=0,
    rz=0,
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


def run_circular_move(dobot):
    user = getattr(config, "CIRCLE_USER_INDEX", 0)
    tool = getattr(config, "CIRCLE_TOOL_INDEX", 0)
    acceleration = getattr(config, "CIRCLE_ACCELERATION_RATIO", 20)
    velocity = getattr(config, "CIRCLE_VELOCITY_RATIO", 20)
    cp = getattr(config, "CIRCLE_CP", 100)

    initial_pose = getattr(config, "CIRCLE_INITIAL_POSE", None)
    if initial_pose:
        move_result = dobot.dashboard.MovJ(
            *initial_pose,
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

    initial_pose = get_pose(dobot, user=user, tool=tool)
    print("Initial pose:", initial_pose)

    poses = generate_xz_circle_poses(
        initial_pose,
        radius=getattr(config, "CIRCLE_RADIUS_MM", 350),
        angle_step_deg=getattr(config, "CIRCLE_STEP_DEG", 5),
        end_angle_deg=getattr(config, "CIRCLE_END_DEG", 90),
        rx=getattr(config, "CIRCLE_RX_DEG", 180),
        start_ry=getattr(config, "CIRCLE_START_RY_DEG", 0),
        rz=getattr(config, "CIRCLE_RZ_DEG", 0),
    )

    for index, pose in enumerate(poses, start=1):
        if get_robot_error(dobot):
            raise RuntimeError("Dobot has active errors. Stop circular move.")

        print(f"Circle point {index}/{len(poses)}:", pose)
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

    return poses


def run_experiment():
    dobot, feed_thread, saved_start_pose = initialize()
    poses = run_circular_move(dobot)
    return dobot, feed_thread, saved_start_pose, poses


if __name__ == "__main__":
    run_experiment()
