from robot.pid import PID

pid = PID(kp=0.5, ki=0.1, kd=0.05)
dt = 0.02  # 50 Hz
target = 100  # target velocity
actual = 0  # starting velocity
for i in range(100):
    error = target - actual
    output = pid.update(error, dt)

    # Simulate: PWM roughly affects velocity (fake physics)
    actual += output * 0.1  # pretend motor responds

    print(
        f"t={i * dt:.2f}s  target={target}  actual={actual:.1f}  error={error:.1f}  output={output:.1f}"
    )
