from Dobot import Dobot
import threading
import time
from time import sleep


# Robot run settings.
DOBOT_IP = "192.168.5.1"
SPEED_RATIO = 30
DO_INDEX = 1
DO_PULSE_SECONDS = 0.001
STEP_DISTANCE_MM = .5
TOTAL_DISTANCE_MM = 150
LOOP_REPEAT_COUNT = 1
STEP_WAIT_SECONDS = 0.150


def has_robot_error(dobot):
    # Use the dashboard TCP command instead of the HTTP alarm API.
    error_result = dobot.dashboard.GetErrorID()
    print("GetErrorID:", error_result)

    result_ids = dobot.parseResultId(error_result)
    if not result_ids or result_ids[0] != 0:
        print("Failed to query robot error ID. Stop for safety.")
        return True

    error_ids = result_ids[1:]
    if not error_ids:
        print("Robot status normal, no active error ID")
        return False

    print(f"Found {len(error_ids)} active robot error ID(s):", error_ids)
    return True


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


def run_linear_point(dobot, point):
    move_result = dobot.dashboard.MovL(*point, 0, v=SPEED_RATIO)
    print("MovL:", move_result)
    if not dobot.WaitCommandDone(move_result):
        raise RuntimeError("MovL failed or timed out")


def send_do_signal(dobot, loop_index, step_index, loop_start_time, do_records):
    on_time_ms = elapsed_ms(loop_start_time)
    do_on_result = dobot.dashboard.DO(DO_INDEX, 1)
    sleep(DO_PULSE_SECONDS)
    off_time_ms = elapsed_ms(loop_start_time)
    do_off_result = dobot.dashboard.DO(DO_INDEX, 0)
    do_records.append({
        "loop": loop_index,
        "step": step_index,
        "signal": f"DO({DO_INDEX},1)->DO({DO_INDEX},0)",
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


def return_to_saved_pose(dobot, saved_pose):
    print("Returning to saved start pose:", saved_pose)
    try:
        stop_result = dobot.dashboard.Stop()
        print("Stop:", stop_result)
        sleep(0.2)
    except Exception as error:
        print("Stop failed:", error)

    try:
        do_off_result = dobot.dashboard.DO(DO_INDEX, 0)
        print(f"DO({DO_INDEX},0):", do_off_result)
    except Exception as error:
        print("DO off failed:", error)

    run_linear_point(dobot, saved_pose)
    current_pose = dobot.GetCurrentPose()
    print("Returned current pose:", current_pose)


def start():
    # Create and connect the Dobot control object.
    dobot = Dobot(DOBOT_IP)
    dobot.connect()

    # Do not enable the robot if it already has active alarms.
    print("Checking robot error information before enable...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before EnableRobot.")
        return

    # Enable the robot before sending any motion commands.
    enable_result = dobot.dashboard.EnableRobot()
    print("EnableRobot:", enable_result)
    if dobot.parseResultId(enable_result)[0] != 0:
        print("使能失败: 请检查机器人是否在 TCP/IP 模式、是否有报警/急停、以及 29999 端口连接")
        has_robot_error(dobot)
        return
    print("使能成功")

    # Set a conservative speed before running the demo path.
    speed_commands = [
        ("SpeedFactor", dobot.dashboard.SpeedFactor(SPEED_RATIO)),
        ("VelJ", dobot.dashboard.VelJ(SPEED_RATIO)),
        ("AccJ", dobot.dashboard.AccJ(SPEED_RATIO)),
    ]
    for name, result in speed_commands:
        print(f"{name}:", result)
        if dobot.parseResultId(result)[0] != 0:
            print(f"{name} set failed, stop demo")
            return
    print("Speed set to", SPEED_RATIO, "%")

    confirm = input("Input 1 to start motion, other input to exit: ").strip()
    if confirm != "1":
        print("Motion canceled")
        return

    # Check again right before motion in case a new alarm appeared.
    print("Checking robot error information before motion...")
    if has_robot_error(dobot):
        print("Robot has active errors. Stop before motion.")
        return

    # Start feedback reading so WaitCommandDone can track command completion.
    feed_thread = threading.Thread(target=dobot.GetFeed)
    feed_thread.daemon = True
    feed_thread.start()

    sleep(1)

    step_count = int(TOTAL_DISTANCE_MM / STEP_DISTANCE_MM)
    if abs(step_count * STEP_DISTANCE_MM - TOTAL_DISTANCE_MM) > 1e-9:
        raise ValueError("TOTAL_DISTANCE_MM must be divisible by STEP_DISTANCE_MM")
    do_records = []

    saved_start_pose = dobot.GetCurrentPose()
    print("Saved start pose:", saved_start_pose)
    print("Direction: -Y")
    print("Step distance:", STEP_DISTANCE_MM, "mm")
    print("Total distance per loop:", TOTAL_DISTANCE_MM, "mm")
    print("Step count per loop:", step_count)

    timer_stop_event = None
    timer_thread = None
    try:
        for loop_index in range(1, LOOP_REPEAT_COUNT + 1):
            print(f"Loop {loop_index}/{LOOP_REPEAT_COUNT} start")
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

                send_do_signal(dobot, loop_index, step_index, loop_start_time, do_records)

                target_pose = start_pose.copy()
                target_pose[1] = start_pose[1] - STEP_DISTANCE_MM * step_index

                move_start_ms = elapsed_ms(loop_start_time)
                print(
                    f"[Loop {loop_index} Step {step_index}] "
                    f"Move start at {format_ms(move_start_ms)}, target Y={target_pose[1]:.3f}"
                )
                run_linear_point(dobot, target_pose)
                move_end_ms = elapsed_ms(loop_start_time)
                print(
                    f"[Loop {loop_index} Step {step_index}] "
                    f"Move end at {format_ms(move_end_ms)}, duration={move_end_ms - move_start_ms}ms"
                )

                sleep(STEP_WAIT_SECONDS)
                print(
                    f"[Loop {loop_index} Step {step_index}] "
                    f"Wait {int(STEP_WAIT_SECONDS * 1000)}ms done at {format_ms(elapsed_ms(loop_start_time))}"
                )

            print(f"Loop {loop_index} reached {TOTAL_DISTANCE_MM}mm. Returning to start pose.")
            return_start_ms = elapsed_ms(loop_start_time)
            run_linear_point(dobot, start_pose)
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
            print(f"Loop {loop_index}/{LOOP_REPEAT_COUNT} finished")
    except KeyboardInterrupt:
        print("Ctrl+C detected. Returning to saved start pose.")
        if timer_stop_event is not None:
            timer_stop_event.set()
        if timer_thread is not None:
            timer_thread.join()
        return_to_saved_pose(dobot, saved_start_pose)

    print("Final DO/time records:")
    for record in do_records:
        print(
            f"Loop {record['loop']} Step {record['step']}: "
            f"{record['signal']}, "
            f"on={record['on_time_ms']}ms, off={record['off_time_ms']}ms, "
            f"on_result={record['on_result']}, off_result={record['off_result']}"
        )


def main():
    start()
