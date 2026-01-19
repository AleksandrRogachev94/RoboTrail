import time

from sensors.servo import HWServo

# Test
servo = HWServo(channel=2, reversed=True)
print("Center (0°)")
servo.set_angle(0)
time.sleep(1)
print("Left (-90°)")
servo.set_angle(-90)
time.sleep(1)
print("Right (90)")
servo.set_angle(90)
time.sleep(1)
print("Back to center")
servo.set_angle(0)
# time.sleep(0.5)
# servo.stop()
print("Done!")
