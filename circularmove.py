#initialize 
#calibration
#loop
#1. calculate next pose
#2. move to next pose
#3. wait
#uutill everything are done, change wavelength

import math
from time import sleep

import config

from Dobot import get_robot_error, initialize_robot
from Laser import connect_laser
from experiment1 import ask_loop_wavelengths, ask_pulse, stop_laser_and_return


def initialize():
    laser = connect_laser(config.LASER_DLL_PATH)
    laser.initialize_laser(config.LASER_WAVELENGTH_NM)

    dobot, feed_thread, saved_start_pose = initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )

    initial_pose = get_initial_pose(dobot, saved_start_pose)
    return laser, dobot, feed_thread, saved_start_pose, initial_pose


def get_initial_pose(dobot, saved_start_pose):
    target_pose = list(saved_start_pose)
    target_pose[3] = config.CIRCLE_RX_DEG
    target_pose[4] = config.CIRCLE_START_RY_DEG
    run_step(
        dobot,
        target_pose,
        config.CIRCLE_USER_INDEX,
        config.CIRCLE_TOOL_INDEX,
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


def run_experiment():
    laser, dobot, feed_thread, saved_start_pose, initial_pose = initialize()

    user = config.CIRCLE_USER_INDEX
    tool = config.CIRCLE_TOOL_INDEX
    acceleration = config.CIRCLE_ACCELERATION_RATIO
    velocity = config.CIRCLE_VELOCITY_RATIO
    cp = config.CIRCLE_CP

    radius = ask_circle_radius()
    end_angle_deg = ask_circle_end_angle()
    total_steps = ask_circle_total_steps()
    ask_pulse()
    loop_wavelengths = ask_loop_wavelengths()
    angle_step_deg = end_angle_deg / total_steps

    poses = generate_xz_circle_poses(
        initial_pose,
        radius=radius,
        angle_step_deg=angle_step_deg,
        end_angle_deg=end_angle_deg,
        rx=config.CIRCLE_RX_DEG,
        start_ry=config.CIRCLE_START_RY_DEG,
        rz=config.CIRCLE_RZ_DEG,
    )

    try:
        if get_robot_error(dobot):
            stop_laser_and_return(laser, dobot, saved_start_pose)
            return laser, dobot, feed_thread, saved_start_pose

        # Set tool coordinates.
        set_tool_result = dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
        activate_result = dobot.ActivateTool(config.TOOL_INDEX)
        print("SetTool result:", set_tool_result)
        print("ActivateTool result:", activate_result)

        laser.run()
        print("Laser RUN")
        sleep(5)

        for loop_index, wavelength in enumerate(loop_wavelengths, start=1):
            if get_robot_error(dobot):
                stop_laser_and_return(laser, dobot, saved_start_pose)
                return laser, dobot, feed_thread, saved_start_pose

            print(f"Loop {loop_index}/{len(loop_wavelengths)}")
            laser.set_wavelength(wavelength)
            sleep(2)

            for index, pose in enumerate(poses, start=1):
                if get_robot_error(dobot):
                    stop_laser_and_return(laser, dobot, saved_start_pose)
                    return laser, dobot, feed_thread, saved_start_pose

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

            if get_robot_error(dobot):
                stop_laser_and_return(laser, dobot, saved_start_pose)
                return laser, dobot, feed_thread, saved_start_pose

            run_step(
                dobot,
                initial_pose,
                user=user,
                tool=tool,
                acceleration=acceleration,
                velocity=velocity,
                cp=cp,
            )
            sleep(2)

    finally:
        laser.stop_safely()

    return laser, dobot, feed_thread, saved_start_pose, poses


if __name__ == "__main__":
    run_experiment()
