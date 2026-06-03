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

# Real tool frame for normal activation and current-pose reports,
# format: "{x,y,z,rx,ry,rz}".
TOOL_INDEX = 1
TOOL_FRAME = "{1.34,3.09056,218.6788,0,0,86}"

# Circular move.
CIRCLE_USER_INDEX = 0
CIRCLE_ACCELERATION_RATIO = 20
CIRCLE_VELOCITY_RATIO = 20
CIRCLE_CP = 100
CIRCLE_INITIAL_POSE = None
CIRCLE_ROTATION_AXIS = "ry"

# The circle center is first defined SUBCUTANEOUS_SCAN_DISTANCE_MM below the
# current probe/tool end along user Z. CIRCLE_RADIUS_MM then sets the final
# probe-to-center distance; the probe moves by radius - subcutaneous distance.
SUBCUTANEOUS_SCAN_DISTANCE_MM = 350
CIRCLE_RADIUS_MM = 350
CIRCLE_ARC_DEG = 120
CIRCLE_TOTAL_STEPS = 18
CIRCLE_RX_DEG = 180

# Lateral sidestep along user Y applied before entering the scan start pose
# and before returning to the circle-center pose. The robot translates by this
# many mm first (predictable linear motion), then MovJs to the target so the
# rotation does not pass through obstacles (e.g. the subject's fingers) sitting
# along the original Y position. Sign chooses which side to step toward.
CIRCLE_APPROACH_OFFSET_MM = 50
