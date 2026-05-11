import threading
import time
from time import sleep

import config
from hardware import (
    connect_robot,
    has_robot_error,
    move_linear_point,
    prepare_robot,
    return_to_pose,
    send_do_pulse,
    start_feedback,
)


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


def print_records(do_records):
    print("Final DO/time records:")
    for record in do_records:
        print(
            f"Loop {record['loop']} Step {record['step']}: "
            f"{record['signal']}, "
            f"on={record['on_time_ms']}ms, off={record['off_time_ms']}ms, "
            f"on_result={record['on_result']}, off_result={record['off_result']}"
        )


def run_experiment(require_confirm=True):
    dobot = connect_robot(config.DOBOT_IP)

    if not prepare_robot(dobot, config.SPEED_RATIO):
        return []

    if require_confirm:
        confirm = input("Input 1 to start motion, other input to exit: ").strip()
        if confirm != "1":
            print("Motion canceled")
            return []

    print("Checking robot error information before motion...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before motion.")
        return []

    start_feedback(dobot)
    sleep(1)

    step_count = calculate_step_count()
    do_records = []

    saved_start_pose = dobot.GetCurrentPose()
    print("Saved start pose:", saved_start_pose)
    print("Direction: -Y")
    print("Step distance:", config.STEP_DISTANCE_MM, "mm")
    print("Total distance per loop:", config.TOTAL_DISTANCE_MM, "mm")
    print("Step count per loop:", step_count)

    timer_stop_event = None
    timer_thread = None
    try:
        for loop_index in range(1, config.LOOP_REPEAT_COUNT + 1):
            print(f"Loop {loop_index}/{config.LOOP_REPEAT_COUNT} start")
            start_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} start current pose:", start_pose)
            loop_start_time = None
            timer_stop_event = threading.Event()
            timer_thread = None

            for step_index in range(1, step_count + 1):
                if loop_start_time is None:
                    loop_start_time = time.perf_counter()
                    timer_thread = threading.Thread(
                        target=run_timer_log,
                        args=(loop_index, loop_start_time, timer_stop_event),
                    )
                    timer_thread.daemon = True
                    timer_thread.start()

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

            print(f"Loop {loop_index} reached {config.TOTAL_DISTANCE_MM}mm. Returning to start pose.")
            return_start_ms = elapsed_ms(loop_start_time)
            move_linear_point(dobot, start_pose, config.SPEED_RATIO)
            return_end_ms = elapsed_ms(loop_start_time)
            print(
                f"Loop {loop_index} returned to start at {format_ms(return_end_ms)}, "
                f"return duration={return_end_ms - return_start_ms}ms"
            )
            end_pose = dobot.GetCurrentPose()
            print(f"Loop {loop_index} end current pose:", end_pose)

            timer_stop_event.set()
            if timer_thread is not None:
                timer_thread.join()
            print(f"Loop {loop_index}/{config.LOOP_REPEAT_COUNT} finished")
    except KeyboardInterrupt:
        print("Ctrl+C detected. Returning to saved start pose.")
        if timer_stop_event is not None:
            timer_stop_event.set()
        if timer_thread is not None:
            timer_thread.join()
        return_to_pose(
            dobot,
            saved_start_pose,
            config.SPEED_RATIO,
            do_indexes=[config.TRIGGER_DO_INDEX],
        )

    print_records(do_records)
    return do_records
