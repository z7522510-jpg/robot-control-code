DOBOT_IP = "192.168.5.1"
SPEED_RATIO = 75

LASER_DLL_PATH = r"C:\Users\Administrator\Desktop\robot-control-code\Laser\REMOTECONTROL.dll"
LASER_WAVELENGTH_NM = 670

TRIGGER_DO_INDEX = 1
TRIGGER_PULSE_SECONDS = 0.03

STEP_DISTANCE_MM = .5
# Per-step scan movement vector in XYZ, in millimeters: [dx, dy, dz].
# Use one non-zero value for axis-aligned scans, or multiple values for angled scans.
# Examples:
#   X positive: [STEP_DISTANCE_MM, 0, 0]
#   Y negative: [0, -STEP_DISTANCE_MM, 0]
#   Z positive: [0, 0, STEP_DISTANCE_MM]
#   45 degrees in XY: [0.3536, -0.3536, 0]  # length is about 0.5 mm
STEP_OFFSET_MM = [-STEP_DISTANCE_MM, 0, 0]
STEP_SPEED_MM_S = 10
TOTAL_DISTANCE_MM  = 4
LOOP_REPEAT_COUNT = 1
STEP_WAIT_SECONDS = 0.150

# Tool frame for SetTool, format: "{x,y,z,rx,ry,rz}".
TOOL_INDEX = 1
TOOL_FRAME = "{0,0,260,0,0,0}"

# Circular move.
CIRCLE_USER_INDEX = 0
CIRCLE_TOOL_INDEX = 0
CIRCLE_ACCELERATION_RATIO = 20
CIRCLE_VELOCITY_RATIO = 20
CIRCLE_CP = 100
CIRCLE_INITIAL_POSE = None

CIRCLE_RADIUS_MM = 350
CIRCLE_STEP_DEG = 5
CIRCLE_END_DEG = 90
CIRCLE_RX_DEG = 180
CIRCLE_START_RY_DEG = 0
CIRCLE_RZ_DEG = 0
