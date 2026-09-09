# Learning Path

The full stage-by-stage build log for RoboTrail — see [README.md](README.md) for the project overview and demo. This doc is the original from-scratch walkthrough: hardware, wiring, and every SLAM concept in the order they were actually learned and implemented, from open-loop dead reckoning up through particle filters and visual SLAM.

> **Note for AI:** Follow [AI_GUIDE.md](AI_GUIDE.md) when assisting with this project. Prioritize teaching over implementing.

## Hardware

### Core Components

| Component           | Model              | Purpose                              |
| ------------------- | ------------------ | ------------------------------------ |
| **Computer**        | Raspberry Pi 5     | Main processing                      |
| **Chassis**         | SMARS (3D printed) | Robot frame                          |
| **Motors**          | N20 + TB6612FNG    | DC motors with encoders for odometry |
| **Distance Sensor** | VL53L1X ToF        | Laser ranging up to 4m (the "lidar") |
| **IMU**             | MPU-6050           | Gyroscope for rotation tracking      |
| **Servo**           | SG90               | Pan the ToF sensor for 180° scans    |

### Power System

| Component          | Spec                                |
| ------------------ | ----------------------------------- |
| **Battery**        | 7.4V 5200mAh 2S LiPo                |
| **Buck Converter** | XL4015 5A (with voltmeter)          |
| **Safety**         | Inline blade fuse (5A)              |
| **Connector**      | Deans/T-plug                        |
| **Monitoring**     | INA219 (I2C voltage/current sensor) |

### Optional Extras

- **SSD1306 OLED** (0.96" I2C) — Display status, battery %, sensor data
- **KY-008 Laser** — Visual debugging (shows where ToF is pointing)

## Learning Path

**SLAM = Simultaneous Localization and Mapping**

SLAM answers two questions at once: _"Where am I?"_ (localization) and _"What does this place look like?"_ (mapping).

**Your Background:** You are comfortable with math/probability (matrices, Bayes rule) but "rusty."
**Our Approach:** We will start with a **deterministic** version (Stage 4) to build intuition for the mechanics. Once that works, we will leverage your math background to upgrade to **Probabilistic SLAM** (Stage 4.5/5), implementing proper uncertainty handling (Bayesian filters) which is the "real" way robotics is done.

**Stages 1-3** teach you the building blocks. **Stage 4** is where you actually do SLAM. **Stage 5** adds intelligence on top.

---

### Stage 1: Open-Loop Movement (Foundation)

**Goal:** Make the robot drive in a perfect 1-meter square.

**What you'll learn:** How stepper motors work, basic motor control, differential turning, the concept of dead reckoning.

- Write Python to control steppers via ULN2003
- Tune `STEPS_PER_CM` until forward distance is accurate
- Tune `STEPS_PER_DEGREE` for precise 90° turns (differential drive: one wheel forward, one backward)
- Combine straight + turn movements to close the square perfectly
- No sensors yet—pure dead reckoning (counting steps to estimate distance and rotation)

**Why it matters:** Dead reckoning is the foundation of knowing "how far did I move?" and "how much did I turn?" Even with sensors, you need this as a starting estimate.

---

### Stage 2: Virtual Scanner (Sensing)

**Goal:** Create a visual radar plot of the room.

**What you'll learn:** How ToF ranging works, polar-to-cartesian conversion, basic visualization.

- Mount VL53L1X on servo
- Sweep 180° taking measurements every 10°
- Plot with matplotlib—should show room outline

**Why it matters:** This is the "mapping" half of SLAM. You're collecting range data that becomes walls on your map.

---

### Stage 3: DC Motors & PID Control

**Goal:** Upgrade to DC motors with encoder feedback and PID control for accurate movement.

**What you'll learn:** Encoder-based odometry, PID velocity control, IMU sensor fusion, control loop architecture.

#### Hardware Changes

| Component                   | Purpose                                        |
| --------------------------- | ---------------------------------------------- |
| **N20 motors with encoder** | 6V, 150RPM, AB hall encoder (~20 cm/sec at 5V) |
| **TB6612FNG driver**        | Efficient H-bridge for PWM motor control       |

#### Pi 5 Encoder Setup

Use kernel-level encoder counting (Python polling is too slow):

```bash
# /boot/firmware/config.txt
dtoverlay=rotary-encoder,pin_a=17,pin_b=18,relative_axis=1  # Left motor
dtoverlay=rotary-encoder,pin_a=22,pin_b=23,relative_axis=1  # Right motor
```

Read encoder position from `/dev/input/eventX` using the `evdev` library.

#### Control Architecture

For stop-and-scan SLAM, a simple **single-threaded blocking** approach works best:

```python
class MotionController:
    def move(self, distance_cm, heading):
        """Blocking: PID runs inside until distance reached."""
        target_ticks = distance_cm * TICKS_PER_CM
        start = self.encoder.total

        while (self.encoder.total - start) < target_ticks:
            # Heading PID
            correction = self.heading_pid.update(heading - self.imu.heading)

            # Velocity PIDs
            left_pwm = self.left_pid.update(self.left_enc.velocity, speed - correction)
            right_pwm = self.right_pid.update(self.right_enc.velocity, speed + correction)
            self.motors.set(left_pwm, right_pwm)

            time.sleep(0.01)  # 100 Hz

        self.motors.stop()
        # Returns when done

def slam_loop():
    motion = MotionController()
    while exploring:
        motion.move(30, heading)     # Blocking - PID runs inside
        scan = scanner.sweep_180()   # Robot stopped
        pose = icp_correct(scan)     # SLAM processing
        update_map(scan, pose)
        heading = plan_next()
```

#### Three PIDs Working Together

| PID                | Input              | Output              | Purpose               |
| ------------------ | ------------------ | ------------------- | --------------------- |
| **Left velocity**  | Left encoder rate  | Left motor PWM      | Track target velocity |
| **Right velocity** | Right encoder rate | Right motor PWM     | Track target velocity |
| **Heading**        | IMU gyro heading   | Velocity difference | Keep robot straight   |

**Key difference from steppers:** Encoders tell you how far you _actually_ moved, not how far you _commanded_. This is true closed-loop control.

**Why it matters for SLAM:** Accurate odometry = better pose prediction = better scan matching = better maps.

#### Arc Movement for Path Following

The robot cannot reliably rotate in place (friction causes stuttering), so we use **arc motion** where both wheels move forward at different speeds.

**Path pipeline:**

1. **A\*** on occupancy grid → raw path
2. **Gradient descent smoothing** (Sebastian Thrun's method) → smooth waypoints
3. **Arc-to-waypoint** geometry → compute arc from current pose to next waypoint
4. **Execute arc()** → differential wheel velocities

**Key formula** - Arc to reach point (x', y') from pose (x, y, θ):

```python
# Transform target to robot frame
local_x = dx * cos(θ) + dy * sin(θ)  # forward
local_y = -dx * sin(θ) + dy * cos(θ)  # left

# Arc radius (positive = left turn)
radius = (local_x² + local_y²) / (2 * local_y)
```

**Wheel velocities for arc:**

```python
v_left  = v_center * (1 - 1/radius * track_width/2)
v_right = v_center * (1 + 1/radius * track_width/2)
```

---

### Stage 4: Occupancy Grid (This is SLAM!)

**Goal:** Build a persistent map while tracking your position.

**What you'll learn:** Occupancy grids, coordinate transforms, scan matching, the predict-correct loop.

#### The Predict-Correct Loop

SLAM continuously runs this cycle:

```
┌─────────────────────────────────────────────────────────────┐
│  1. PREDICT: "Based on my motor commands, I should be here" │
│     └─ Use: Stepper counts + Gyro heading                   │
│                          ↓                                  │
│  2. SCAN: "What do I see around me?"                        │
│     └─ Use: ToF sensor swept by servo (18 points @ 10°)     │
│                          ↓                                  │
│  3. CORRECT: "Does this scan match what the map says?"      │
│     └─ If wall in map is 2cm left of where I expected →     │
│        I must be 2cm right of where I thought               │
│                          ↓                                  │
│  4. UPDATE: Add new walls to map, mark free space           │
│                          ↓                                  │
│  5. PLAN: Where should I go next? (find unexplored area)    │
│                          ↓                                  │
│  6. MOVE: Execute movement → back to step 1                 │
└─────────────────────────────────────────────────────────────┘
```

#### How Each Sensor Is Used

| Sensor              | Role in SLAM                              |
| ------------------- | ----------------------------------------- |
| **Encoders**        | PREDICT: "Wheels moved 200 ticks = ~10cm" |
| **Gyro (MPU-6050)** | PREDICT: "I rotated 45° during that turn" |
| **ToF (VL53L1X)**   | SCAN: "Wall is 87cm away at angle 30°"    |
| **Servo**           | SCAN: Points the ToF at different angles  |

#### How Scanning Works

```python
def scan():
    points = []
    for angle in range(0, 181, 10):  # 0°, 10°, 20°... 180°
        servo.angle = angle
        time.sleep(0.1)  # Let servo settle
        distance = tof.read()
        points.append((angle, distance))
    return points  # 19 points forming a semicircle view
```

Each scan gives you ~19 points. Convert to map coordinates:

```python
for angle, distance in points:
    # Convert polar (angle, distance) to cartesian (x, y)
    world_angle = robot_heading + angle  # Absolute angle
    x = robot_x + distance * cos(world_angle)
    y = robot_y + distance * sin(world_angle)
    map[x][y] = WALL  # Mark as occupied
```

#### How Correction Works (Simplified)

```python
# After scanning, compare to existing map
expected_wall_distance = map.ray_cast(robot_x, robot_y, angle)
actual_wall_distance = tof.read()

error = actual_wall_distance - expected_wall_distance
# If error > 0: wall is further than expected → I'm closer to it
# If error < 0: wall is closer than expected → I'm further from it

robot_x += error * cos(angle)  # Nudge position estimate
robot_y += error * sin(angle)
```

This is a simplified version. Real SLAM uses algorithms like:

- **ICP** (Iterative Closest Point) — matches scan points to map
- **Particle Filter** — maintains multiple position hypotheses
- **Kalman Filter** — probabilistic state estimation

For learning, start with the simple version above!

#### Handling Scan Gaps

With 10° angular resolution, walls will have gaps between points. Start simple—multiple scans from different positions naturally fill gaps as evidence accumulates. If scan matching becomes jittery, add **distance-threshold interpolation** (fill between similar-distance readings). If A\* paths clip walls, add **inflation** (mark cells near obstacles as blocked).

#### How the Robot Explores

**Frontier-Based Exploration:**

1. Find cells in the map that are "unknown" (never scanned)
2. Find "frontiers" = boundaries between known-free and unknown
3. Drive toward the nearest frontier
4. Repeat until no frontiers remain (room fully mapped)

```
Map:  ? = unknown, . = free, # = wall

  ? ? ? ? ? ?
  ? . . . ? ?    Robot sees frontier at top-right
  ? . @ . ? ?    @ = robot position
  ? . . . # ?    Drives toward "?" to explore
  ? # # # # ?
```

#### Path Planning with A\*

Once you know _where_ to go (frontier), you need to plan _how_ to get there while avoiding obstacles.

**A\* Algorithm:** Finds the shortest path on your occupancy grid.

```python
def a_star(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(grid, current):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score, neighbor))

    return None  # No path found
```

**Heuristic:** Use Euclidean or Manhattan distance to the goal. A\* guarantees the shortest path if the heuristic never overestimates.

#### What You'll Build

- `occupancy_grid.py` — 2D numpy array representing the map
- `scanner.py` — Sweep servo, collect ToF readings
- `localizer.py` — Predict position, correct with scan matching
- `path_planner.py` — A\* implementation for obstacle avoidance
- `explorer.py` — Find frontiers, use A\* to plan paths
- `main.py` — The loop that ties it all together

---

### Stage 5: Extended Kalman Filter (Probabilistic)

**Goal:** Replace deterministic position tracking with probabilistic state estimation.

**What you'll learn:** Gaussian uncertainty, covariance matrices, the predict-update cycle, sensor fusion via Kalman gain.

**Why EKF?** Stage 4 uses simple correction ("nudge position by error"). This works but doesn't track _how confident_ you are. EKF maintains a probability distribution over your position—uncertainty grows when moving, shrinks when sensing.

#### The EKF Loop

```python
# State: [x, y, θ] with covariance P (3×3 matrix)

def predict(motor_command):
    # Update state estimate
    x += distance * cos(θ)
    y += distance * sin(θ)
    θ += gyro_reading

    # Uncertainty GROWS (process noise Q)
    P = F @ P @ F.T + Q

def update(scan):
    for measurement in scan:
        expected = ray_cast(x, y, θ + angle)
        innovation = measurement - expected

        # Kalman gain: balance prediction vs measurement trust
        K = P @ H.T @ inv(H @ P @ H.T + R)

        # Correct state
        state += K @ innovation

        # Uncertainty SHRINKS
        P = (I - K @ H) @ P
```

#### Key Concepts

| Concept                   | Meaning                                        |
| ------------------------- | ---------------------------------------------- |
| **State vector**          | `[x, y, θ]` — what we're estimating            |
| **Covariance (P)**        | How uncertain we are about each state variable |
| **Process noise (Q)**     | How much uncertainty motion adds               |
| **Measurement noise (R)** | How noisy the ToF sensor is                    |
| **Kalman gain (K)**       | How much to trust sensor vs prediction         |
| **Jacobian (F, H)**       | Linearization for nonlinear models             |

#### What You'll Build

- `ekf.py` — EKF state estimation class
- Modify `localizer.py` to use EKF instead of simple correction
- Tune Q and R matrices for your hardware

---

### Stage 6: Particle Filter (Multi-Hypothesis)

**Goal:** Handle ambiguous situations where EKF fails (symmetry, kidnapping, global localization).

**What you'll learn:** Monte Carlo methods, importance sampling, resampling, representing arbitrary probability distributions.

**Why Particle Filter?** EKF assumes a single Gaussian belief. If the robot could be in multiple places (identical hallways, placed in unknown location), EKF can't represent this. Particles can.

#### The Particle Filter Loop

```python
# Particles: N samples, each is (x, y, θ, weight)
particles = initialize_uniform(N=100)

def predict(motor_command):
    for p in particles:
        # Move each particle with noise
        p.x += distance * cos(p.θ) + random.gauss(0, σ_motion)
        p.y += distance * sin(p.θ) + random.gauss(0, σ_motion)
        p.θ += gyro_reading + random.gauss(0, σ_rotation)

def update(scan):
    for p in particles:
        # Score: how well does this particle explain the scan?
        p.weight = 1.0
        for angle, distance in scan:
            expected = ray_cast(p.x, p.y, p.θ + angle)
            error = (distance - expected) ** 2
            p.weight *= exp(-error / (2 * σ_sensor²))

    # Normalize weights
    total = sum(p.weight for p in particles)
    for p in particles:
        p.weight /= total

    # Resample: keep good particles, discard bad ones
    particles = resample(particles)

def get_estimate():
    # Weighted average of all particles
    return (sum(p.x * p.weight), sum(p.y * p.weight), ...)
```

#### Key Concepts

| Concept         | Meaning                                         |
| --------------- | ----------------------------------------------- |
| **Particle**    | One hypothesis of robot pose                    |
| **Weight**      | How likely this hypothesis is given sensor data |
| **Resampling**  | Clone good particles, delete bad ones           |
| **Multimodal**  | Can represent multiple possible locations       |
| **Convergence** | Particles cluster at true location over time    |

#### When to Use Each

| Scenario                  | Use                                |
| ------------------------- | ---------------------------------- |
| Known start, building map | EKF (efficient, accurate)          |
| Unknown start, known map  | Particle (global localization)     |
| Kidnapped robot recovery  | Particle (redistribute hypotheses) |
| Symmetric environments    | Particle (multiple hypotheses)     |

#### What You'll Build

- `particle_filter.py` — Particle filter class
- Modify `localizer.py` to switch between EKF and particle mode
- Implement resampling strategies (systematic, low-variance)

#### A Note on Loop Closure

Neither EKF nor particle filters solve **loop closure** — detecting when you've returned to a previously visited area. Small scan-matching errors accumulate over time, so your map may not "close" properly:

```
Start → explore → return to start
        ↓
Map shows you 10cm away from where you actually started!
```

**Detection:** Compare current scan to stored scan from that area. High similarity = loop detected.

**Correction:** Requires **graph-based SLAM** (e.g., g2o, GTSAM) to optimize the entire trajectory. This is an advanced topic beyond the current stages but worth knowing exists.

For room-sized exploration, accumulated error is usually small enough that loop closure isn't critical. For larger spaces or repeated runs, it becomes essential.

---

### Stage 7: Semantic Localization (Advanced)

**Goal:** Recognize _where_ you are using visual landmarks.

**What you'll learn:** Object detection, landmark-based localization, loop closure.

- Run YOLO on Pi Camera
- When detecting objects (chair, door), save their location to the map
- Use landmarks for relocalization if robot gets lost

**Why it matters:** Pure geometric SLAM can get confused in similar-looking corridors. Recognizing "I see the red chair" gives absolute position references.

---

### Stage 8: Visual SLAM (Feature-Based)

**Goal:** Full camera-based SLAM using visual features instead of (or alongside) ToF.

**What you'll learn:** Feature extraction (ORB, SIFT), descriptor matching, visual odometry, bundle adjustment, loop closure detection.

**The idea:** Extract distinctive points from camera images, track them across frames to estimate motion, and build a 3D map of feature locations. This is how systems like ORB-SLAM work.

**Why it's the final stage:** Combines everything—probabilistic state estimation, sensor fusion, and now replaces the ToF "lidar" with pure vision.

---

## I2C Wiring

I2C is a 2-wire bus that lets multiple devices share the same connection. Each device has a unique address.

### Physical Connection

```
Raspberry Pi 5          All I2C Devices (shared bus)
─────────────           ─────────────────────────────
GPIO 2 (SDA) ──────────┬── VL53L1X SDA
                       ├── MPU-6050 SDA
                       ├── INA219 SDA
                       └── SSD1306 SDA

GPIO 3 (SCL) ──────────┬── VL53L1X SCL
                       ├── MPU-6050 SCL
                       ├── INA219 SCL
                       └── SSD1306 SCL

3.3V ──────────────────┬── VL53L1X VCC
                       ├── MPU-6050 VCC
                       ├── INA219 VCC
                       └── SSD1306 VCC

GND ───────────────────┬── VL53L1X GND
                       ├── MPU-6050 GND
                       ├── INA219 GND
                       └── SSD1306 GND
```

### I2C Addresses (Default)

| Device   | Address |
| -------- | ------- |
| VL53L1X  | 0x29    |
| MPU-6050 | 0x68    |
| INA219   | 0x40    |
| SSD1306  | 0x3C    |

All different, so no conflicts! Run `i2cdetect -y 1` on the Pi to see connected devices.

## Main Loop Strategy: Stop-and-Scan

```
1. Move    → Drive forward using dead reckoning + gyro correction
2. Stop    → Every 50cm, halt completely
3. Scan    → Sweep servo 180° to gather ToF measurements
4. Update  → Add walls to occupancy grid, correct position
5. Plan    → Find unexplored areas, drive toward them
```

## Key Concepts

| Concept             | What It Means                                         |
| ------------------- | ----------------------------------------------------- |
| **Dead Reckoning**  | Estimating position by counting motor steps           |
| **Sensor Fusion**   | Combining multiple sensors for better accuracy        |
| **PID Controller**  | Feedback loop to minimize error (drift correction)    |
| **Occupancy Grid**  | 2D array representing free/occupied space             |
| **ZUPT**            | Zero-Velocity Update—recalibrate gyro when stationary |
| **Sensor Sampling** | Read gyro/encoders at 50Hz+ to capture all motion     |

> [!IMPORTANT] > **Gyro and Encoder Sampling Rate**
>
> Gyros output _instantaneous angular rate_ (deg/sec), not accumulated rotation. To know total rotation, you must integrate: `total = ∫ rate(t) dt`. If you sample too slowly, you miss rotation that happened between samples (**aliasing**).
>
> **Example:** Robot turns for 1 second, then goes straight for 1 second.
>
> - Sample at 50Hz: Captures turn → integrates to 20° ✅
> - Sample at end only: Reads 0°/s × 2s = 0° ❌
>
> **This applies to ALL algorithms** (dead reckoning, PID, EKF, particle filter). The estimation algorithm can run slowly, but sensor integration must be continuous.
>
> Similarly, DC motor encoders output pulses that must be counted at high rate (hardware interrupt or kernel driver) or pulses are lost.

## Coordinate Systems

SLAM requires converting between three coordinate frames:

```
1. SENSOR FRAME     2. ROBOT FRAME       3. WORLD FRAME
   (ToF reading)       (chassis)            (the map)

      ↑ 0°                ↑ Forward            ↑ North (Y+)
      │                   │                    │
  ←───┼───→           ←───●───→            ←───┼───→ East (X+)
      │               Robot center             │
   Servo angle        (origin)              Map origin
```

### When to Convert

| From → To      | When                      | Formula                               |
| -------------- | ------------------------- | ------------------------------------- |
| Sensor → Robot | After each ToF reading    | Add sensor offset `(x_tof, y_tof)`    |
| Robot → World  | When adding points to map | Rotate by heading, add robot position |

### Sensor Position Matters

The ToF sensor is mounted ~7cm in front of robot center. **This offset must be corrected**, otherwise scans from different headings won't align:

```
Facing North: ToF is 7cm closer to north wall → measures 1.93m instead of 2.0m
Facing East:  ToF offset is now horizontal → measures 2.0m correctly

Without correction: same wall appears at y=1.93m and y=2.0m (7cm error!)
```

### Transformation Code

```python
# Sensor offsets (meters from robot center)
TOF_OFFSET_X = 0.0    # Centered left-right
TOF_OFFSET_Y = 0.07   # 7cm forward

def scan_point_to_world(distance, servo_angle, robot_x, robot_y, robot_heading):
    """Convert a single ToF reading to world coordinates."""
    # Sensor frame → Robot frame
    px = distance * cos(servo_angle) + TOF_OFFSET_X
    py = distance * sin(servo_angle) + TOF_OFFSET_Y

    # Robot frame → World frame (rotate + translate)
    world_x = robot_x + px * cos(robot_heading) - py * sin(robot_heading)
    world_y = robot_y + px * sin(robot_heading) + py * cos(robot_heading)
    return world_x, world_y
```

## Servo PWM on Pi 5

Software PWM causes jitter under CPU load. Use **hardware PWM** for reliable servo control:

```bash
# /boot/firmware/config.txt
dtoverlay=pwm-2chan
```

Servo must be on **GPIO 12 or 18** (PWM0) or **GPIO 13 or 19** (PWM1). Use `rpi-hardware-pwm` library:

```python
from rpi_hardware_pwm import HardwarePWM
pwm = HardwarePWM(pwm_channel=0, hz=50, chip=2)  # chip=2 for Pi 5
pwm.start(7.5)  # 7.5% = center (1.5ms pulse)
# 5% = -90°, 10% = +90°
```

**Alternative:** PCA9685 I²C servo driver ($3-5) handles timing externally—16 channels, no GPIO constraints.

## Wiring Overview

### Option A: DC Motors (N20 + TB6612FNG)

| Connection        | GPIO/Pin                       |
| ----------------- | ------------------------------ |
| **TB6612FNG**     |                                |
| VM                | 6V motor supply                |
| VCC               | 3.3V                           |
| GND               | Common ground                  |
| STBY              | 3.3V (tie high)                |
| PWMA              | GPIO 12 (right speed, ~1kHz)   |
| PWMB              | GPIO 16 (left speed, ~1kHz)    |
| AIN1 / AIN2       | GPIO 20 / 21 (right direction) |
| BIN1 / BIN2       | GPIO 25 / 24 (left direction)  |
| A01 / A02         | Right motor (red/white)        |
| B01 / B02         | Left motor (red/white)         |
| **Encoders**      |                                |
| Left encoder A/B  | GPIO 17 / 27                   |
| Right encoder A/B | GPIO 5 / 6                     |
| Encoder VCC       | 3.3V                           |
| Encoder GND       | GND                            |

> [!NOTE]
> Motors use **software PWM** (lgpio) at ~1kHz to avoid conflict with the servo's hardware PWM on GPIO 18 (50Hz). BIN1/BIN2 are swapped (25/24) to correct left motor direction.

### Option B: Stepper Motors (28BYJ-48 + ULN2003)

| Connection  | GPIO                |
| ----------- | ------------------- |
| Left motor  | GPIO 17, 27, 22, 23 |
| Right motor | GPIO 5, 6, 13, 19   |

### Common Wiring

```
Raspberry Pi 5
├── I2C Bus (GPIO 2/3)
│   ├── VL53L1X (ToF sensor)
│   ├── MPU-6050 (IMU)
│   ├── INA219 (battery monitor)
│   └── SSD1306 (OLED display)
├── Servo (GPIO 18, hardware PWM)
├── KY-008 Laser (optional)
└── Power: 5V from XL4015 buck converter
```

## Resources

- [SMARS Robot](https://www.smarsfan.com/) — Chassis design and mods
- [VL53L1X Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [MPU-6050 Guide](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)

## License

MIT
