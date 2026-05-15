from .dobot import Dobot

SetTool = Dobot.SetTool

from .dobot_control import (
    calculate_step_count,
    connect_robot,
    disconnect_robot,
    enable_robot,
    get_robot_error,
    has_robot_error,
    initialize_robot,
    move_linear_point,
    move_relative_xyz,
    prepare_robot,
    return_to_pose,
    run_step,
    send_do_pulse,
    set_robot_speed,
    start_feedback,
    stop_and_return,
    turn_do_off,
)

__all__ = [
    "Dobot",
    "SetTool",
    "calculate_step_count",
    "connect_robot",
    "disconnect_robot",
    "enable_robot",
    "get_robot_error",
    "has_robot_error",
    "initialize_robot",
    "move_linear_point",
    "move_relative_xyz",
    "prepare_robot",
    "return_to_pose",
    "run_step",
    "send_do_pulse",
    "set_robot_speed",
    "start_feedback",
    "stop_and_return",
    "turn_do_off",
]
