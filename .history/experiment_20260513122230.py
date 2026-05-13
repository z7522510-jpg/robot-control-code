import threading
import time
import math
from time import sleep

import config
from Dobot import (
    build_xyz_step_target,
    calculate_step_count,
    connect_robot,
    has_robot_error,
    prepare_robot,
    return_to_pose,
    run_xyz_step_cycle,
    start_feedback,
)
from Laser import connect_laser


def empty_result():
    return {"do_records": [], "endpoint_records": []}


def elapsed_ms(start_time):
    return int((time.perf_counter() - start_time) * 1000)


def format_ms(milliseconds):
    return f"{milliseconds / 1000:.3f}s"


class LoopTimer:
    def __init__(self):
        self.stop_event = None
        self.thread = None

    def start(self, loop_index, loop_start_time):
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            args=(loop_index, loop_start_time),
        )
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        if self.stop_event is not None:
            self.stop_event.set()
        if self.thread is not None:
            self.thread.join()
        self.stop_event = None
        self.thread = None

    def _run(self, loop_index, start_time):
        next_second = 1
        while not self.stop_event.wait(0.01):
            current_ms = elapsed_ms(start_time)
            if current_ms >= next_second * 1000:
                print(f"[Loop {loop_index}] timer: {format_ms(current_ms)}")
                next_second += 1


def validate_step_offset():
    if len(config.STEP_OFFSET_MM) != 3:
        raise ValueError("STEP_OFFSET_MM must be [dx, dy, dz]")
    if get_step_distance_mm(validate=False) <= 1e-12:
        raise ValueError("STEP_OFFSET_MM must include movement in at least one XYZ direction")


def get_step_distance_mm(validate=True):
    if validate and len(config.STEP_OFFSET_MM) != 3:
        raise ValueError("STEP_OFFSET_MM must be [dx, dy, dz]")
    return math.sqrt(sum(offset * offset for offset in config.STEP_OFFSET_MM))


def initialize_laser_from_config(laser):
    # Laser warm-up and wavelength setup live in Laser; experiment only passes
    # the current config values.
    laser.initialize_laser(
        config.LASER_FIRE_MODE,
        config.LASER_SYNC_MODE,
        config.LASER_SYNC_DELAY,
        config.LASER_WAVELENGTH_NM,
    )


def prompt_positive_number(prompt, default, value_type=float):
    value_text = input(f"{prompt} [{default}]: ").strip()
    if not value_text:
        return default

    value = value_type(value_text)
    if value <= 0:
        raise ValueError(f"{prompt} must be greater than 0")
    return value


def prompt_experiment_plan():
    # Ask once before motion starts. Each loop can use a different wavelength.
    loop_count = prompt_positive_number(
        "Input loop repeat count",
        config.LOOP_REPEAT_COUNT,
        int,
    )

    loop_wavelengths = []
    for loop_index in range(1, loop_count + 1):
        default_wavelength = (
            config.LASER_WAVELENGTH_NM
            if loop_index == 1
            else loop_wavelengths[-1]
        )
        wavelength = prompt_positive_number(
            f"Input laser wavelength for loop {loop_index} nm",
            default_wavelength,
            float,
        )
        loop_wavelengths.append(wavelength)

    print("Planned loop wavelengths:")
    for loop_index, wavelength in enumerate(loop_wavelengths, start=1):
        print(f"Loop {loop_index}: {wavelength} nm")

    return loop_count, loop_wavelengths


def run_dobot_step(dobot, start_pose, loop_index, step_index, loop_start_time, do_records):
    target_pose = build_xyz_step_target(
        start_pose,
        step_index,
        config.STEP_OFFSET_MM,
    )
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"Move target XYZ=({target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f})"
    )

    cycle_record = run_xyz_step_cycle(
        dobot,
        start_pose,
        step_index,
        config.SPEED_RATIO,
        config.STEP_OFFSET_MM,
        config.STEP_WAIT_SECONDS,
        config.TRIGGER_DO_INDEX,
        config.TRIGGER_PULSE_SECONDS,
        loop_start_time=loop_start_time,
    )

    do_records.append({
        "loop": loop_index,
        **cycle_record,
    })
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"DO pulse ON at {format_ms(cycle_record['on_time_ms'])}: {cycle_record['on_result']}"
    )
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"DO pulse OFF at {format_ms(cycle_record['off_time_ms'])}: {cycle_record['off_result']}"
    )


def record_endpoint(dobot, loop_index, endpoint_records, wavelength_nm):
    # Record endpoint before returning to the original pose.
    endpoint_pose = dobot.GetCurrentPose()
    endpoint_records.append({
        "loop": loop_index,
        "wavelength_nm": wavelength_nm,
        "endpoint_pose": endpoint_pose,
    })
    print(f"Loop {loop_index} endpoint pose:", endpoint_pose)
    return endpoint_pose


def print_records(do_records, endpoint_records):
    print("Final DO/time records:")
    for record in do_records:
        print(
            f"Loop {record['loop']} Step {record['step']}: "
            f"{record['signal']}, "
            f"on={record['on_time_ms']}ms, off={record['off_time_ms']}ms, "
            f"on_result={record['on_result']}, off_result={record['off_result']}"
        )
    print("Final endpoint records:")
    for record in endpoint_records:
        print(f"Loop {record['loop']}: endpoint_pose={record['endpoint_pose']}")


def run_experiment(require_confirm=True):
    # Connect both devices first; cleanup in finally keeps laser off even if
    # the robot loop is interrupted.
    dobot = connect_robot(config.DOBOT_IP)
    laser = connect_laser(config.LASER_DLL_PATH)

    do_records = []
    endpoint_records = []
    saved_start_pose = None
    loop_timer = LoopTimer()

    try:
        validate_step_offset()

        if not prepare_robot(dobot, config.SPEED_RATIO):
            return empty_result()

        initialize_laser_from_config(laser)

        # Non-interactive callers use config defaults; interactive runs can
        # override loop count and wavelength plan before motion starts.
        loop_count = config.LOOP_REPEAT_COUNT
        loop_wavelengths = [config.LASER_WAVELENGTH_NM] * loop_count

        if require_confirm:
            loop_count, loop_wavelengths = prompt_experiment_plan()
            confirm = input("Input 1 to start motion, other input to exit: ").strip()
            if confirm != "1":
                print("Motion canceled")
                return empty_result()

        print("Checking robot error information before motion...")
        if has_robot_error(dobot):
            print("Robot has active errors. Stop before motion.")
            return empty_result()

        start_feedback(dobot)
        sleep(1)

        step_distance_mm = get_step_distance_mm()
        step_count = calculate_step_count(config.TOTAL_DISTANCE_MM, step_distance_mm)
        saved_start_pose = dobot.GetCurrentPose()
        print("Saved start pose:", saved_start_pose)
        print("Step offset XYZ:", config.STEP_OFFSET_MM, "mm")
        print("Step distance:", step_distance_mm, "mm")
        print("Total distance per loop:", config.TOTAL_DISTANCE_MM, "mm")
        print("Step count per loop:", step_count)

        laser.run()
        print("Laser RUN for experiment")

        for loop_index in range(1, loop_count + 1):
            # Each loop starts from the current pose, moves step-by-step,
            # records the endpoint, then returns to the original saved pose.
            loop_wavelength = loop_wavelengths[loop_index - 1]
            print(f"Loop {loop_index}/{loop_count} start")
            print(f"Loop {loop_index} wavelength:", loop_wavelength, "nm")
            laser.set_wavelength(loop_wavelength)
            sleep(2)  # Short wait after wavelength change before starting motion; laser is already running, so no long warm-up needed.
            start_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} start current pose:", start_pose)
            loop_start_time = None
            if loop_index == 1:
                sleep(6)  # Longer wait before the first loop to allow for final confirmation and setup

            for step_index in range(1, step_count + 1):
                if loop_start_time is None:
                    # Start laser exactly once per loop. It stays running while
                    # Dobot repeats pulse -> move -> wait.
                    loop_start_time = time.perf_counter()
                    loop_timer.start(loop_index, loop_start_time)
                    print(f"[Loop {loop_index}] loop timer start at {format_ms(elapsed_ms(loop_start_time))}")

                run_dobot_step(dobot, start_pose, loop_index, step_index, loop_start_time, do_records)

            record_endpoint(dobot, loop_index, endpoint_records, loop_wavelength)

            print(f"Loop {loop_index} reached {config.TOTAL_DISTANCE_MM}mm. Returning to start pose.")
            return_start_ms = elapsed_ms(loop_start_time)
            dobot.MoveLinearPoint(saved_start_pose, config.SPEED_RATIO)
            return_end_ms = elapsed_ms(loop_start_time)
            print(
                f"Loop {loop_index} returned to start at {format_ms(return_end_ms)}, "
                f"return duration={return_end_ms - return_start_ms}ms"
            )
            end_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} end current pose:", end_pose)

            loop_timer.stop()
            print(f"Loop {loop_index}/{loop_count} finished")
    except KeyboardInterrupt:
        # Manual interruption should leave the setup in a known safe state:
        # timer stopped, laser stopped, robot returned if possible.
        print("Ctrl+C detected. Returning to saved start pose.")
        loop_timer.stop()
        laser.stop_safely()
        if saved_start_pose is not None:
            return_to_pose(
                dobot,
                saved_start_pose,
                config.SPEED_RATIO,
                do_indexes=[config.TRIGGER_DO_INDEX],
            )
    finally:
        loop_timer.stop()
        laser.close()

    print_records(do_records, endpoint_records)
    return {"do_records": do_records, "endpoint_records": endpoint_records}
