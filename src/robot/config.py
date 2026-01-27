"""Robot calibration constants and pin configuration.

Update STEPS_PER_CM and STEPS_PER_DEGREE after running calibration.
"""

# =============================================================================
# Physical Dimensions (measure your robot)
# =============================================================================
WHEEL_DIAMETER_MM = 40  # Outer diameter of drive wheels WITH TRACKS
TRACK_WIDTH_MM = 145.0  # Distance between wheel centers (for pivot turns)

# =============================================================================
# Stepper Motor Pin Configuration (BCM GPIO numbers) - 28BYJ-48 + ULN2003
# =============================================================================
LEFT_STEPPER_PINS = [17, 27, 22, 23]  # IN1, IN2, IN3, IN4
RIGHT_STEPPER_PINS = [5, 6, 13, 19]  # IN1, IN2, IN3, IN4

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
# Calibrated Movement Constants
# Run calibrate_motors.py to measure these values for your robot.
# =============================================================================

# Steps required to move 1 cm forward
STEPS_PER_CM = 365.7

# Steps required to rotate 1 degree (pivot turn)
STEPS_PER_DEGREE = 62.0  # TODO: Calibrate with actual measurement

# =============================================================================
# Motor Timing
# =============================================================================
DEFAULT_STEP_DELAY = 0.001  # Seconds between steps (1ms = smooth movement)

# =============================================================================
# PID Control Constants (Tuned Ziegler-Nichols)
# =============================================================================
# Velocity PID (per motor) - tuned via pid_tune.py
PID_KP = 0.15
PID_KI = 0.1
PID_KD = 0.0

# Feedforward: PWM = OFFSET + SLOPE * velocity
# Measure by finding PWM at dead zone (~40%) and max velocity (~1400 ticks/sec at 100%)
FEEDFORWARD_OFFSET = 40.0  # Dead zone PWM %
FEEDFORWARD_SLOPE = 0.043  # (100-40) / 1400 ≈ 0.043

# Heading PID (robot level) - TODO: Tune for your robot
HEADING_PID_KP = 5.0  # degrees error → velocity differential
HEADING_PID_KI = 0.5
HEADING_PID_KD = 0.0

# =============================================================================
# Servo/ToF Calibration
# =============================================================================
# Offset to correct servo 0° to true forward (run calibrate_servo_angle.py)
SERVO_ANGLE_OFFSET = 0.0  # degrees
