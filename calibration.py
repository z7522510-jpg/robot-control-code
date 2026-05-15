# Dobot tool calibration check (same logic as calibration.ipynb).
# Pose format: [x, y, z, rx, ry, rz].

import config

from Dobot import get_robot_error, initialize_robot


# Initialize Dobot.
def initialize():
    dobot, feed_thread, saved_start_pose = initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )

    return dobot, feed_thread, saved_start_pose


def run_experiment():
    # 1. Initialize robot; abort if it has active errors.
    dobot, feed_thread, saved_start_pose = initialize()

    if get_robot_error(dobot):
        raise RuntimeError("Dobot has active errors. Stop before calibration.")

    # Pose with the default tool (tool 0, flange). Robot does not move.
    before_pose = dobot.GetCurrentPose()
    print("Before pose (tool 0):", before_pose)

    # 2. Define tool frame, then activate it as the global tool.
    # SetTool only writes the definition; ActivateTool makes GetPose use it.
    # No motion happens, so the delta below reflects only the tool offset.
    set_tool_result = dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
    activate_result = dobot.ActivateTool(config.TOOL_INDEX)

    after_pose = dobot.GetCurrentPose()
    print("SetTool result:", set_tool_result)
    print("ActivateTool result:", activate_result)
    print("After pose (tool %d):" % config.TOOL_INDEX, after_pose)

    # 3. Compare before/after to see how the tool frame shifted the pose.
    delta_pose = [after - before for before, after in zip(before_pose, after_pose)]

    print("Before pose:", before_pose)
    print("After pose:", after_pose)
    print("Delta pose:", delta_pose)

    # Restore default tool so later scripts are not affected.
    dobot.ActivateTool(0)

    report = {
        "tool_index": config.TOOL_INDEX,
        "tool_frame": config.TOOL_FRAME,
        "before_pose": before_pose,
        "after_pose": after_pose,
        "delta_pose": delta_pose,
    }
    print("Report:", report)
    return dobot, feed_thread, report


if __name__ == "__main__":
    run_experiment()
