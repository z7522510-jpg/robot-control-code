import math


def _wrap_degrees(degrees):
    wrapped = (degrees + 180.0) % 360.0 - 180.0
    if abs(wrapped) < 1e-9:
        return 0.0
    return wrapped


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _normalize_axis(axis):
    normalized = str(axis).strip().lower()
    if normalized in ("x", "rx"):
        return "x"
    if normalized in ("y", "ry"):
        return "y"
    raise ValueError("axis must be 'rx'/'x' or 'ry'/'y'")


def _position_values(position_or_pose, name):
    if len(position_or_pose) == 3:
        return [float(value) for value in position_or_pose]
    if len(position_or_pose) == 6:
        return [float(value) for value in position_or_pose[:3]]
    raise ValueError(f"{name} must have 3 position values or 6 pose values")


def _mat3_mul(left, right):
    return [
        [
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def _mat3_vec_mul(matrix, vector):
    return [
        sum(matrix[row][index] * vector[index] for index in range(3))
        for row in range(3)
    ]


def _matrix_from_rotation_translation(rotation, translation):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_x(degrees):
    angle = math.radians(degrees)
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return [
        [1.0, 0.0, 0.0],
        [0.0, cos_angle, -sin_angle],
        [0.0, sin_angle, cos_angle],
    ]


def rot_y(degrees):
    angle = math.radians(degrees)
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return [
        [cos_angle, 0.0, sin_angle],
        [0.0, 1.0, 0.0],
        [-sin_angle, 0.0, cos_angle],
    ]


def rot_z(degrees):
    angle = math.radians(degrees)
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return [
        [cos_angle, -sin_angle, 0.0],
        [sin_angle, cos_angle, 0.0],
        [0.0, 0.0, 1.0],
    ]


def user_axis_rotation(axis, degrees):
    axis = _normalize_axis(axis)
    if axis == "x":
        return rot_x(degrees)
    return rot_y(degrees)


def pose_to_matrix(pose):
    if len(pose) != 6:
        raise ValueError("pose must have 6 values: x, y, z, rx, ry, rz")

    x, y, z, rx, ry, rz = [float(value) for value in pose]

    # Dobot pose assumption: R = Rz @ Ry @ Rx.
    rotation = _mat3_mul(_mat3_mul(rot_z(rz), rot_y(ry)), rot_x(rx))
    return _matrix_from_rotation_translation(rotation, [x, y, z])


def matrix_to_pose(matrix):
    rotation = [
        [float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2])],
        [float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2])],
        [float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2])],
    ]
    x = float(matrix[0][3])
    y = float(matrix[1][3])
    z = float(matrix[2][3])

    # Inverse of R = Rz @ Ry @ Rx.
    ry_rad = math.asin(_clamp(-rotation[2][0], -1.0, 1.0))
    cos_ry = math.cos(ry_rad)

    if abs(cos_ry) > 1e-9:
        rx_rad = math.atan2(rotation[2][1], rotation[2][2])
        rz_rad = math.atan2(rotation[1][0], rotation[0][0])
    else:
        rx_rad = 0.0
        rz_rad = math.atan2(-rotation[0][1], rotation[1][1])

    return [
        x,
        y,
        z,
        _wrap_degrees(math.degrees(rx_rad)),
        _wrap_degrees(math.degrees(ry_rad)),
        _wrap_degrees(math.degrees(rz_rad)),
    ]


def rotate_pose_around_user_axis(start_pose, center_position, angle_deg, axis="rx"):
    start_matrix = pose_to_matrix(start_pose)
    start_position = [start_matrix[0][3], start_matrix[1][3], start_matrix[2][3]]
    start_rotation = [
        [start_matrix[0][0], start_matrix[0][1], start_matrix[0][2]],
        [start_matrix[1][0], start_matrix[1][1], start_matrix[1][2]],
        [start_matrix[2][0], start_matrix[2][1], start_matrix[2][2]],
    ]
    center = _position_values(center_position, "center_position")
    offset = [
        start_position[0] - center[0],
        start_position[1] - center[1],
        start_position[2] - center[2],
    ]

    axis_rotation = user_axis_rotation(axis, angle_deg)
    rotated_offset = _mat3_vec_mul(axis_rotation, offset)
    target_position = [
        center[0] + rotated_offset[0],
        center[1] + rotated_offset[1],
        center[2] + rotated_offset[2],
    ]
    target_rotation = _mat3_mul(axis_rotation, start_rotation)
    return matrix_to_pose(_matrix_from_rotation_translation(target_rotation, target_position))


def offset_position_from_pose(pose, local_offset):
    if len(pose) != 6:
        raise ValueError("pose must have 6 values: x, y, z, rx, ry, rz")
    if len(local_offset) != 3:
        raise ValueError("local_offset must have 3 values: x, y, z")

    matrix = pose_to_matrix(pose)
    position = [matrix[0][3], matrix[1][3], matrix[2][3]]
    rotation = [
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]],
    ]
    offset = [float(value) for value in local_offset]
    user_offset = _mat3_vec_mul(rotation, offset)

    return [
        position[0] + user_offset[0],
        position[1] + user_offset[1],
        position[2] + user_offset[2],
    ]


def generate_real_tool_circle_poses(
    start_pose,
    center_position,
    total_steps,
    arc_deg,
    axis="rx",
):
    if len(start_pose) != 6:
        raise ValueError("start_pose must have 6 values: x, y, z, rx, ry, rz")
    if total_steps <= 0:
        raise ValueError("total_steps must be greater than 0")

    center = _position_values(center_position, "center_position")
    poses = []
    start_offset = arc_deg / 2
    angle_step = arc_deg / total_steps

    for step_index in range(total_steps + 1):
        angle = start_offset - step_index * angle_step
        poses.append(
            rotate_pose_around_user_axis(
                start_pose,
                center,
                angle,
                axis=axis,
            )
        )

    return poses


def generate_tool_center_circle_poses(
    center_pose,
    total_steps,
    arc_deg,
    axis="rx",
):
    if len(center_pose) != 6:
        raise ValueError("center_pose must have 6 values: x, y, z, rx, ry, rz")
    if total_steps <= 0:
        raise ValueError("total_steps must be greater than 0")

    center_matrix = pose_to_matrix(center_pose)
    center_position = [center_matrix[0][3], center_matrix[1][3], center_matrix[2][3]]
    center_rotation = [
        [center_matrix[0][0], center_matrix[0][1], center_matrix[0][2]],
        [center_matrix[1][0], center_matrix[1][1], center_matrix[1][2]],
        [center_matrix[2][0], center_matrix[2][1], center_matrix[2][2]],
    ]

    poses = []
    start_offset = arc_deg / 2
    angle_step = arc_deg / total_steps

    for step_index in range(total_steps + 1):
        angle = start_offset - step_index * angle_step
        axis_rotation = user_axis_rotation(axis, angle)

        # Left multiply: rotate around the fixed X/Y axis of the user frame.
        target_rotation = _mat3_mul(axis_rotation, center_rotation)
        target_matrix = _matrix_from_rotation_translation(
            target_rotation,
            center_position,
        )
        poses.append(matrix_to_pose(target_matrix))

    return poses


generate_user_axis_circle_poses = generate_tool_center_circle_poses
