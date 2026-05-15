import config

from Dobot import initialize_robot
from Laser import connect_laser


# Pose format: [x, y, z, rx, ry, rz]
WANTED_POSE = [
    100,
    100,
    0,
    0,
    0,
    0,
]


# Initialize laser and Dobot.
def initialize():
    laser = connect_laser(config.LASER_DLL_PATH)
    laser.initialize_laser(config.LASER_WAVELENGTH_NM)

    dobot, feed_thread, saved_start_pose = initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )

    return laser, dobot, feed_thread, saved_start_pose


def run_experiment():
    laser, dobot, feed_thread, saved_start_pose = initialize()

    dobot.MoveLinearPoint(WANTED_POSE, config.SPEED_RATIO)

    return laser, dobot, feed_thread, saved_start_pose


if __name__ == "__main__":
    run_experiment()
