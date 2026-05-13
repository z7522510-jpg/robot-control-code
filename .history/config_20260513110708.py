DOBOT_IP = "192.168.5.1"
SPEED_RATIO = 30

LASER_DLL_PATH = r"C:\Users\Administrator\Desktop\robot-control-code\Laser\REMOTECONTROL.dll"
LASER_FIRE_MODE = "Continuous"
LASER_SYNC_MODE = "INTERNAL"
LASER_SYNC_DELAY = "-80"
LASER_WAVELENGTH_NM = 670

TRIGGER_DO_INDEX = 1
TRIGGER_PULSE_SECONDS = 0.001

STEP_DISTANCE_MM = 1
# Per-step scan movement vector in XYZ, in millimeters: [dx, dy, dz].
# Use one non-zero value for axis-aligned scans, or multiple values for angled scans.
# Examples:
#   X positive: [STEP_DISTANCE_MM, 0, 0]
#   Y negative: [0, -STEP_DISTANCE_MM, 0]
#   Z positive: [0, 0, STEP_DISTANCE_MM]
#   45 degrees in XY: [0.3536, -0.3536, 0]  # length is about 0.5 mm
STEP_OFFSET_MM = [-STEP_DISTANCE_MM, 0, 0]
TOTAL_DISTANCE_MM  = 50
LOOP_REPEAT_COUNT = .5
STEP_WAIT_SECONDS = 0.150
