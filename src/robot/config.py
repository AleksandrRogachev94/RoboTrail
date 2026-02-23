"""Robot calibration constants and pin configuration."""

# =============================================================================
# Physical Dimensions (measure your robot)
# =============================================================================
WHEEL_DIAMETER_CM = 4  # Outer diameter of drive wheels
TRACK_WIDTH_CM = 14.5  # Distance between wheel centers (measure on new chassis)

# =============================================================================
# DC Motor Pin Configuration (BCM GPIO numbers) - N20 + TB6612FNG
# =============================================================================
# Right motor (TB6612FNG channel A)
DC_RIGHT_PWM = 12
DC_RIGHT_IN1 = 20
DC_RIGHT_IN2 = 21

# Left motor (TB6612FNG channel B) - IN1/IN2 swapped to fix direction
DC_LEFT_PWM = 16
DC_LEFT_IN1 = 25  # Swapped from 24
DC_LEFT_IN2 = 24  # Swapped from 25

# PWM frequency for DC motors (Hz)
DC_PWM_FREQ = 1000

# =============================================================================
# DC Motor Movement Constants
# =============================================================================
TICKS_PER_CM = 45  # Calibrate with real measurement
MAX_FORWARD_VELOCITY = 700  # ticks/sec max wheel speed
DT = 0.02  # 50Hz control loop

# Forward motion profile
RAMP_DISTANCE_CM = 8.0  # Ramp up/down over this distance
MIN_SPEED_FACTOR = 0.3  # Minimum speed during ramp (30% of target)

# Turn profile
RAMP_ANGLE_DEG = 20.0  # Ramp up/down over this angle
MAX_TURN_VELOCITY = 500  # ticks/sec max wheel speed during turns
MIN_TURN_FACTOR = 0.2  # Minimum speed during turn ramp (20%)
TURN_PID_FINE_GAIN = 0.1  # 10% of normal heading PID for fine correction

# =============================================================================
# PID Control Constants (Tuned Ziegler-Nichols)
# =============================================================================
# Velocity PID (per motor) - tuned via pid_tune.py
PID_KP = 0.3
PID_KI = 0.1
PID_KD = 0.0

# Feedforward: PWM = OFFSET + SLOPE * velocity
# Calibrated via calibrate_feedforward.py
FEEDFORWARD_OFFSET = 25.0  # Dead zone PWM %
FEEDFORWARD_SLOPE = 0.0679  # Measured from 50%→90% PWM tests

# Heading PID (robot level) - TODO: Tune for your robot
HEADING_PID_KP = 30.0  # degrees error → velocity differential
HEADING_PID_KI = 0.5
HEADING_PID_KD = 0.0

# =============================================================================
# Servo/ToF Calibration
# =============================================================================
# Offset to correct servo 0° to true forward (run calibrate_servo_angle.py)
SERVO_ANGLE_OFFSET = -5.0  # degrees (calibrated via calibrate_servo_angle.py)
TOF_OFFSET_X = 7.0  # cm (forward from wheel axis)
TOF_OFFSET_Y = 0.0  # cm (lateral center)

# =============================================================================
# Occupancy Grid
# =============================================================================
GRID_RESOLUTION = 2.0  # cm per cell
GRID_SIZE = 500  # cells per side (500 × 500 = 10m × 10m at 2cm resolution)
GRID_ORIGIN = 250  # Robot starts at cell (250, 250) = center of grid

# Log-odds update parameters
L_OCC = 0.85  # Added when ray HITS a cell (occupied evidence)
L_FREE = 0.4  # Subtracted when ray PASSES THROUGH a cell (free evidence)
L_MAX = 5.0  # Clamp max (prevents over-confidence)
L_MIN = -5.0  # Clamp min
