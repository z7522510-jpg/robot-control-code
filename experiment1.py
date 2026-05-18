import config
from time import sleep

from Dobot import (
    calculate_step_count,
    get_robot_error,
    initialize_robot,
    run_step,
    stop_and_return,
    turn_do_off,
)
from Laser import connect_laser


# Initialize laser and Dobot.
def initialize():
    laser = connect_laser(config.LASER_DLL_PATH)
    laser.initialize_laser(config.LASER_WAVELENGTH_NM)

    dobot, feed_thread, saved_start_pose = initialize_robot(
        config.DOBOT_IP,
        config.SPEED_RATIO,
    )

    return laser, dobot, feed_thread, saved_start_pose


def ask_robot_step():
    step_distance = float(input(f"Step distance mm [{config.STEP_DISTANCE_MM}]: ") or config.STEP_DISTANCE_MM)
    direction = input("Direction x+/x-/y+/y-/z+/z- [x-]: ").strip().lower() or "x-"

    directions = {
        "x+": [step_distance, 0, 0],
        "x-": [-step_distance, 0, 0],
        "y+": [0, step_distance, 0],
        "y-": [0, -step_distance, 0],
        "z+": [0, 0, step_distance],
        "z-": [0, 0, -step_distance],
    }
    if direction not in directions:
        raise ValueError("Direction must be one of: x+, x-, y+, y-, z+, z-")

    config.STEP_DISTANCE_MM = step_distance
    config.STEP_OFFSET_MM = directions[direction]
    print("STEP_DISTANCE_MM =", config.STEP_DISTANCE_MM)
    print("STEP_OFFSET_MM =", config.STEP_OFFSET_MM)
    return config.STEP_DISTANCE_MM, config.STEP_OFFSET_MM


def ask_pulse():
    do_index = int(input(f"Trigger DO index [{config.TRIGGER_DO_INDEX}]: ") or config.TRIGGER_DO_INDEX)
    pulse_seconds = float(input(f"Trigger pulse seconds [{config.TRIGGER_PULSE_SECONDS}]: ") or config.TRIGGER_PULSE_SECONDS)

    if do_index <= 0:
        raise ValueError("Trigger DO index must be greater than 0")
    if pulse_seconds <= 0:
        raise ValueError("Trigger pulse seconds must be greater than 0")

    config.TRIGGER_DO_INDEX = do_index
    config.TRIGGER_PULSE_SECONDS = pulse_seconds
    print("TRIGGER_DO_INDEX =", config.TRIGGER_DO_INDEX)
    print("TRIGGER_PULSE_SECONDS =", config.TRIGGER_PULSE_SECONDS)
    return config.TRIGGER_DO_INDEX, config.TRIGGER_PULSE_SECONDS


def ask_loop_wavelengths():
    loop_count = int(input(f"Loop repeat count [{config.LOOP_REPEAT_COUNT}]: ") or config.LOOP_REPEAT_COUNT)
    if loop_count <= 0:
        raise ValueError("Loop repeat count must be greater than 0")

    wavelengths = []
    for loop_index in range(1, loop_count + 1):
        wavelength = float(input(f"Loop {loop_index} wavelength nm [{config.LASER_WAVELENGTH_NM}]: ") or config.LASER_WAVELENGTH_NM)
        if wavelength <= 0:
            raise ValueError("Laser wavelength must be greater than 0")
        wavelengths.append(wavelength)

    config.LOOP_REPEAT_COUNT = loop_count
    return wavelengths


def stop_laser_and_return(laser, dobot, saved_start_pose):
    laser.stop_safely()
    stop_and_return(
        dobot,
        saved_start_pose,
        config.SPEED_RATIO,
        do_index=config.TRIGGER_DO_INDEX,
    )


def run_experiment():
    laser, dobot, feed_thread, saved_start_pose = initialize()
    ask_robot_step()
    ask_pulse()
    loop_wavelengths = ask_loop_wavelengths()

    step_count = calculate_step_count(config.TOTAL_DISTANCE_MM, config.STEP_DISTANCE_MM)
    print("Step count:", step_count)

    try:
        if get_robot_error(dobot):
            stop_laser_and_return(laser, dobot, saved_start_pose)
            return laser, dobot, feed_thread, saved_start_pose

        # Set tool coordinates.
        dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)

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

            for step_index in range(1, step_count + 1):
                if get_robot_error(dobot):
                    stop_laser_and_return(laser, dobot, saved_start_pose)
                    return laser, dobot, feed_thread, saved_start_pose

                run_step(
                    dobot,
                    step_index,
                    config.STEP_OFFSET_MM,
                    config.SPEED_RATIO,
                    config.STEP_WAIT_SECONDS,
                    config.TRIGGER_DO_INDEX,
                    config.TRIGGER_PULSE_SECONDS,
                )

            # One error check after the whole scan loop (covers the last step
            # too); the per-step check is at the top of the next iteration.
            if get_robot_error(dobot):
                stop_laser_and_return(laser, dobot, saved_start_pose)
                return laser, dobot, feed_thread, saved_start_pose

            turn_do_off(dobot, config.TRIGGER_DO_INDEX)
            dobot.MoveLinearPoint(saved_start_pose, config.SPEED_RATIO)
            sleep(2)

    finally:
        laser.stop_safely()

    return laser, dobot, feed_thread, saved_start_pose


if __name__ == "__main__":
    run_experiment()
