import config

from Dobot import get_robot_error, initialize_robot


# Pose format: [x, y, z, rx, ry, rz]
WANTED_POSE = [
    100,
    100,
    0,
    0,
    0,
    0,
]


# Initialize Dobot.
def initialize():
    dobot, feed_thread, saved_start_pose = initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )

    return dobot, feed_thread, saved_start_pose


def run_experiment():
    dobot, feed_thread, saved_start_pose = initialize()

    if get_robot_error(dobot):
        return dobot, feed_thread, saved_start_pose

    # Set tool coordinates.
    dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)

    dobot.MoveLinearPoint(WANTED_POSE, config.SPEED_RATIO)

    return dobot, feed_thread, saved_start_pose


if __name__ == "__main__":
    run_experiment()
