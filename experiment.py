import threading
import time
from time import sleep

import config
from Dobot import (
    connect_robot,
    has_robot_error,
    move_linear_point,
    prepare_robot,
    return_to_pose,
    send_do_pulse,
    start_feedback,
)
from Laser import connect_laser


def empty_result():
    return {"do_records": [], "endpoint_records": []}


def elapsed_ms(start_time):
    return int((time.perf_counter() - start_time) * 1000)


def format_ms(milliseconds):
    return f"{milliseconds / 1000:.3f}s"


def run_timer_log(loop_index, start_time, stop_event):
    next_second = 1
    while not stop_event.wait(0.01):
        current_ms = elapsed_ms(start_time)
        if current_ms >= next_second * 1000:
            print(f"[Loop {loop_index}] timer: {format_ms(current_ms)}")
            next_second += 1


def start_loop_timer(loop_index, loop_start_time):
    stop_event = threading.Event()
    timer_thread = threading.Thread(
        target=run_timer_log,
        args=(loop_index, loop_start_time, stop_event),
    )
    timer_thread.daemon = True
    timer_thread.start()
    return stop_event, timer_thread


def stop_loop_timer(stop_event, timer_thread):
    if stop_event is not None:
        stop_event.set()
    if timer_thread is not None:
        timer_thread.join()


def calculate_step_count():
    step_count = int(config.TOTAL_DISTANCE_MM / config.STEP_DISTANCE_MM)
    total_distance = step_count * config.STEP_DISTANCE_MM
    if abs(total_distance - config.TOTAL_DISTANCE_MM) > 1e-9:
        raise ValueError("TOTAL_DISTANCE_MM must be divisible by STEP_DISTANCE_MM")
    return step_count


def build_step_target(start_pose, step_index):
    target_pose = start_pose.copy()
    target_pose[1] = start_pose[1] - config.STEP_DISTANCE_MM * step_index
    return target_pose


def initialize_laser_from_config(laser):
    laser.initialize_laser(
        config.LASER_FIRE_MODE,
        config.LASER_SYNC_MODE,
        config.LASER_SYNC_DELAY,
        config.LASER_WAVELENGTH_NM,
    )


def send_trigger_pulse(dobot, loop_index, step_index, loop_start_time, do_records):
    on_time_ms = elapsed_ms(loop_start_time)
    do_on_result, do_off_result = send_do_pulse(
        dobot,
        config.TRIGGER_DO_INDEX,
        config.TRIGGER_PULSE_SECONDS,
    )
    off_time_ms = elapsed_ms(loop_start_time)

    do_records.append({
        "loop": loop_index,
        "step": step_index,
        "signal": f"DO({config.TRIGGER_DO_INDEX},1)->DO({config.TRIGGER_DO_INDEX},0)",
        "on_time_ms": on_time_ms,
        "off_time_ms": off_time_ms,
        "on_result": do_on_result,
        "off_result": do_off_result,
    })
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"DO pulse ON at {format_ms(on_time_ms)}: {do_on_result}"
    )
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"DO pulse OFF at {format_ms(off_time_ms)}: {do_off_result}"
    )


def run_dobot_step(dobot, start_pose, loop_index, step_index, loop_start_time, do_records):
    send_trigger_pulse(dobot, loop_index, step_index, loop_start_time, do_records)

    target_pose = build_step_target(start_pose, step_index)

    move_start_ms = elapsed_ms(loop_start_time)
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"Move start at {format_ms(move_start_ms)}, target Y={target_pose[1]:.3f}"
    )
    move_linear_point(dobot, target_pose, config.SPEED_RATIO)
    move_end_ms = elapsed_ms(loop_start_time)
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"Move end at {format_ms(move_end_ms)}, duration={move_end_ms - move_start_ms}ms"
    )

    sleep(config.STEP_WAIT_SECONDS)
    print(
        f"[Loop {loop_index} Step {step_index}] "
        f"Wait {int(config.STEP_WAIT_SECONDS * 1000)}ms done at "
        f"{format_ms(elapsed_ms(loop_start_time))}"
    )


def record_endpoint(dobot, loop_index, endpoint_records):
    endpoint_pose = dobot.GetCurrentPose()
    endpoint_records.append({
        "loop": loop_index,
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
    dobot = connect_robot(config.DOBOT_IP)
    laser = connect_laser(config.LASER_DLL_PATH)

    do_records = []
    endpoint_records = []
    saved_start_pose = None
    timer_stop_event = None
    timer_thread = None

    try:
        if not prepare_robot(dobot, config.SPEED_RATIO):
            return empty_result()

        initialize_laser_from_config(laser)

        if require_confirm:
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

        step_count = calculate_step_count()
        saved_start_pose = dobot.GetCurrentPose()
        print("Saved start pose:", saved_start_pose)
        print("Direction: -Y")
        print("Step distance:", config.STEP_DISTANCE_MM, "mm")
        print("Total distance per loop:", config.TOTAL_DISTANCE_MM, "mm")
        print("Step count per loop:", step_count)

        for loop_index in range(1, config.LOOP_REPEAT_COUNT + 1):
            print(f"Loop {loop_index}/{config.LOOP_REPEAT_COUNT} start")
            start_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} start current pose:", start_pose)
            loop_start_time = None
            timer_stop_event = None
            timer_thread = None
            laser_running = False

            for step_index in range(1, step_count + 1):
                if loop_start_time is None:
                    loop_start_time = time.perf_counter()
                    timer_stop_event, timer_thread = start_loop_timer(loop_index, loop_start_time)
                    laser.run()
                    laser_running = True
                    print(f"[Loop {loop_index}] laser RUN at {format_ms(elapsed_ms(loop_start_time))}")

                run_dobot_step(dobot, start_pose, loop_index, step_index, loop_start_time, do_records)

            record_endpoint(dobot, loop_index, endpoint_records)

            if laser_running:
                laser.stop()
                laser_running = False
                print(f"[Loop {loop_index}] laser STOP at {format_ms(elapsed_ms(loop_start_time))}")

            print(f"Loop {loop_index} reached {config.TOTAL_DISTANCE_MM}mm. Returning to start pose.")
            return_start_ms = elapsed_ms(loop_start_time)
            move_linear_point(dobot, saved_start_pose, config.SPEED_RATIO)
            return_end_ms = elapsed_ms(loop_start_time)
            print(
                f"Loop {loop_index} returned to start at {format_ms(return_end_ms)}, "
                f"return duration={return_end_ms - return_start_ms}ms"
            )
            end_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} end current pose:", end_pose)

            stop_loop_timer(timer_stop_event, timer_thread)
            print(f"Loop {loop_index}/{config.LOOP_REPEAT_COUNT} finished")
    except KeyboardInterrupt:
        print("Ctrl+C detected. Returning to saved start pose.")
        stop_loop_timer(timer_stop_event, timer_thread)
        laser.stop_safely()
        if saved_start_pose is not None:
            return_to_pose(
                dobot,
                saved_start_pose,
                config.SPEED_RATIO,
                do_indexes=[config.TRIGGER_DO_INDEX],
            )
    finally:
        stop_loop_timer(timer_stop_event, timer_thread)
        laser.close()

    print_records(do_records, endpoint_records)
    return {"do_records": do_records, "endpoint_records": endpoint_records}
