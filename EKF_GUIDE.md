# Extended Kalman Filter Guide

This guide maps EKF concepts to the RoboTrail robot, distilled from our learning discussion.

> [!IMPORTANT] > **Sensor Sampling vs EKF Update Rate**
>
> Gyros output instantaneous rate (deg/sec), not accumulated rotation. You must sample at 50Hz+ and integrate continuously to capture all motion. The EKF predict/update can run slowly, but **sensor integration must be fast** or you'll miss rotation (aliasing). Same applies to DC motor encoders.

## Why EKF (Not Linear KF)?

Your motion model has **trigonometry**:

```
x_new = x + distance × cos(θ)
y_new = y + distance × sin(θ)
```

The matrix entries depend on the current state θ—that's what makes it **nonlinear**. EKF handles this by recomputing a linearization (Jacobian) at each step.

---

## State, Input, and Measurements

| Element            | What It Is           | Your Hardware                      |
| ------------------ | -------------------- | ---------------------------------- |
| **State x**        | What we estimate     | `[x, y, θ]` (position + heading)   |
| **Input u**        | What we command      | Stepper counts → distance/rotation |
| **Measurements z** | What sensors observe | ToF distances                      |

### Sensor Roles

| Sensor                   | Role                      | Reasoning                         |
| ------------------------ | ------------------------- | --------------------------------- |
| **Steppers**             | Input (predict step)      | You command them directly         |
| **Gyro**                 | Input (predict step)      | Reliable, fast—use directly for θ |
| **Encoders** (DC motors) | Input (predict step)      | Treat like stepper counts         |
| **ToF**                  | Measurement (update step) | Corrects position estimate        |

> **Key insight**: Gyro as input is simpler than as measurement. The gyro reading directly updates θ in the predict step—no extra Kalman update needed.

---

## The Two Steps

### 1. PREDICT (Motion Update)

Apply physics, then grow uncertainty:

```python
def predict(state, distance, gyro_rotation):
    x, y, theta = state

    # Apply motion model
    x_new = x + distance * cos(theta)
    y_new = y + distance * sin(theta)
    theta_new = theta + gyro_rotation

    # Compute Jacobian F at new state
    F = [[1, 0, -distance * sin(theta_new)],
         [0, 1,  distance * cos(theta_new)],
         [0, 0,  1]]

    # Grow uncertainty
    P_new = F @ P @ F.T + Q

    return [x_new, y_new, theta_new], P_new
```

### 2. UPDATE (Measurement Correction)

Compare expected vs actual sensor reading, then correct:

```python
def update(state, P, tof_distance, servo_angle, map):
    # What should ToF read given our state?
    expected = ray_cast(map, state, servo_angle)
    innovation = tof_distance - expected

    # How does expected reading change with state? (Jacobian)
    H = numerical_jacobian(state, servo_angle, map)  # 1×3

    # Kalman gain: balance prediction vs measurement trust
    R = [[sigma_tof**2]]  # ToF noise
    K = P @ H.T @ inv(H @ P @ H.T + R)

    # Correct state and shrink uncertainty
    state_new = state + K @ innovation
    P_new = (I - K @ H) @ P

    return state_new, P_new
```

---

## Key Matrices

| Matrix | Size                  | Meaning              | How to Set                              |
| ------ | --------------------- | -------------------- | --------------------------------------- |
| **P**  | 3×3                   | Current uncertainty  | Starts large, shrinks with measurements |
| **Q**  | 3×3                   | Motion uncertainty   | Scale by distance moved + angle rotated |
| **R**  | 1×1 (per ToF reading) | Sensor noise         | ~2cm for VL53L1X                        |
| **F**  | 3×3                   | Motion Jacobian      | Recompute each predict step             |
| **H**  | 1×3 (per ToF reading) | Measurement Jacobian | Recompute each update step              |
| **K**  | 3×1                   | Kalman Gain          | Computed from P, H, R                   |

### Setting Q (Process Noise)

Q scales with how much you moved:

```python
def compute_Q(distance, rotation):
    sigma_trans = 0.01   # 1cm per cm moved
    sigma_rot = 0.02     # ~1° per 10° turned
    sigma_drift = 0.001  # Gyro drift per step

    return np.diag([
        (sigma_trans * abs(distance))**2,
        (sigma_trans * abs(distance))**2,
        (sigma_rot * abs(rotation) + sigma_drift)**2
    ])
```

---

## The H Jacobian: Two Approaches

### Approach A: Scan Matching (Recommended)

Since Stage 4 already uses scan matching, reuse it:

```python
# Scan matcher outputs pose correction (you already have this from Stage 4)
dx, dy, dtheta, covariance = scan_match(scan, map, predicted_pose)

# Use correction as measurement
z = [dx, dy, dtheta]
H = np.eye(3)  # Identity! No Jacobian needed.
R = covariance  # From scan matcher (or tuned constant)
```

**Why recommended**: H is identity (simple!), reuses Stage 4 code, handles data association robustly.

### Approach B: Raw ToF (Educational)

For learning the full EKF, you can use raw ToF readings:

```python
def numerical_H(state, servo_angle, map, eps=1e-4):
    x, y, theta = state
    z0 = ray_cast(map, x, y, theta + servo_angle)

    # Central differences for accuracy
    dz_dx = (ray_cast(map, x+eps, y, theta+servo_angle) -
             ray_cast(map, x-eps, y, theta+servo_angle)) / (2*eps)
    dz_dy = (ray_cast(map, x, y+eps, theta+servo_angle) -
             ray_cast(map, x, y-eps, theta+servo_angle)) / (2*eps)
    dz_dt = (ray_cast(map, x, y, theta+eps+servo_angle) -
             ray_cast(map, x, y, theta-eps+servo_angle)) / (2*eps)

    return [[dz_dx, dz_dy, dz_dt]]
```

**Tradeoff**: More complex (many ray casts for Jacobian), fragile if position estimate is wrong (bad data association).

---

## Processing Multiple ToF Readings

Two valid approaches:

### Sequential (Simpler)

```python
for angle, distance in tof_scan:
    state, P = update(state, P, distance, angle, map)
```

### Batch (More Efficient)

Stack all readings into one update with a 19×3 H matrix and 19×19 R matrix.

Both converge to the same result—use sequential for learning.

---

## Update Frequency

| Phase                   | When to Run EKF                                         |
| ----------------------- | ------------------------------------------------------- |
| **Stop-and-scan**       | Predict after each move, update with full scan          |
| **Continuous movement** | Predict at 10-50Hz, update whenever ToF has new reading |

---

## When Does EKF Actually Matter?

**Honest assessment for your project:**

| Factor         | Your Robot         | Impact                                     |
| -------------- | ------------------ | ------------------------------------------ |
| Speed          | 2 cm/s             | Scan matching easily corrects small errors |
| Sensors        | 2 (gyro, steppers) | Little to fuse                             |
| Runtime        | Minutes            | Drift doesn't compound much                |
| Start position | Known (0,0,0)      | No global localization needed              |

**Result**: For stop-and-scan at 2 cm/s, Stage 4's simple correction may produce maps nearly as good as EKF. The value of EKF for your project is primarily **educational**.

**When EKF becomes essential**:

- Faster movement (need between-scan tracking)
- More sensors (need proper fusion)
- Longer runs (uncertainty tracking helps detect drift)
- Reliability requirements (graceful degradation)

---

## Variants to Explore

Once basic EKF works:

1. **Gyro as measurement** instead of input—adds redundancy
2. **Velocity in state** `[x, y, θ, v, ω]`—for continuous movement smoothing
3. **Particle filter**—for global localization or symmetric environments
4. **Outlier rejection**—skip measurements with high Mahalanobis distance

---

## Recommended Implementation Order

1. **Predict only**: Just the motion model, no measurements. Watch uncertainty grow.
2. **Fake measurements**: Hardcode expected values to test update step.
3. **Real ToF**: Add actual sensor readings with numerical Jacobian.
4. **Tune Q, R**: Adjust until position estimate is smooth and accurate.
5. **Try scan matching**: Compare robustness vs raw ToF approach.

---

## Common Pitfalls

| Problem                  | Cause                                       | Fix                         |
| ------------------------ | ------------------------------------------- | --------------------------- |
| Position "jumps" wildly  | R too small (over-trusting noisy sensor)    | Increase R                  |
| Corrections are too slow | R too large (ignoring sensor)               | Decrease R                  |
| Diverges over time       | Q too small                                 | Increase Q                  |
| Very noisy estimate      | Q too large                                 | Decrease Q                  |
| Map walls don't align    | Wrong sensor offset or coordinate transform | Check `scan_point_to_world` |

---

## Quick Reference

```
EKF LOOP:
─────────
1. PREDICT
   state = f(state, motor_input, gyro)  ← Nonlinear physics
   F = Jacobian of f                     ← Linearization
   P = F·P·Fᵀ + Q                        ← Uncertainty grows

2. UPDATE (for each ToF reading)
   expected = ray_cast(state, angle, map)
   innovation = actual - expected
   H = Jacobian of ray_cast              ← Linearization
   K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹              ← Kalman Gain
   state = state + K·innovation          ← Correction
   P = (I - K·H)·P                       ← Uncertainty shrinks

3. Update map with corrected state
4. Plan next move
5. Go to 1
```
