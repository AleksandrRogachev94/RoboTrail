"""Robot calibration constants and pin configuration.

Update STEPS_PER_CM and STEPS_PER_DEGREE after running calibration.
"""

# =============================================================================
# Physical Dimensions (measure your robot)
# =============================================================================
WHEEL_DIAMETER_MM = 40  # Outer diameter of drive wheels WITH TRACKS
TRACK_WIDTH_MM = 130.0  # Distance between wheel centers (for pivot turns)

# =============================================================================
# Motor Pin Configuration (BCM GPIO numbers)
# =============================================================================
LEFT_MOTOR_PINS = [17, 27, 22, 23]  # IN1, IN2, IN3, IN4
RIGHT_MOTOR_PINS = [5, 6, 13, 19]  # IN1, IN2, IN3, IN4

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
