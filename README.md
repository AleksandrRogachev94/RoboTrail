# RoboTrail 🤖

A 3D-printed Raspberry Pi robot that maps rooms on its own — no pre-built SLAM library, just a ToF sensor on a servo, some encoders, an IMU, and a lot of linear algebra.

<video src="https://github.com/user-attachments/assets/6f5a3e32-c5a6-42ae-8448-716495ea4b60" controls autoplay loop muted></video>

*The robot exploring a room: driving, stopping to scan with the ToF sensor, building the occupancy grid, and streaming its camera feed the whole time.*

## What this is

Most cheap SLAM demos lean entirely on a camera. That falls apart on plain walls or when the wheels slip a little. RoboTrail instead fuses several cheap sensors — wheel encoders, a gyro, and a time-of-flight distance sensor on a pan servo — so it can localize and map a room even when the visuals give it nothing to work with.

It's also a learning project. Every stage was built to understand *why* SLAM works, not just to get a demo running — dead reckoning first, then scan matching, then a proper probabilistic backend. The full stage-by-stage build log (hardware, wiring, and the SLAM concepts in the order they were learned) is in [LEARNING_PATH.md](LEARNING_PATH.md); the guides linked below go deep on individual pieces.

> **AI assistants:** see [AI_GUIDE.md](AI_GUIDE.md) — the point of this repo is to learn robotics, so teach concepts rather than just writing the implementation.

## Hardware

| Part | What | Why |
| --- | --- | --- |
| Raspberry Pi 5 | Brains | Runs everything, streams the camera over the web UI |
| SMARS chassis | 3D-printed, tank treads | Cheap, easy to modify |
| N20 motors + TB6612FNG | Drive with encoders | Closed-loop odometry instead of guessing |
| VL53L1X | Time-of-flight sensor | The "lidar" — 4m range |
| SG90 servo | Pans the ToF sensor | Turns one point sensor into a 180° scan |
| MPU-6050 | Gyro | Tracks heading, corrects drift between scans |
| Pi Camera v3 | Camera | Live video feed while it drives |
| 2S LiPo + XL4015 + INA219 | Power | 7.4V pack, buck-converted, with voltage/current monitoring |

Full wiring, I2C addresses, and the chassis dimensions are in [CHASSIS.md](CHASSIS.md).

## How it maps a room

The core loop is "stop and scan": drive a bit, stop, sweep the servo for a fresh set of range readings, fold that into the map, decide where to explore next, repeat.

- **Odometry** — encoder ticks + gyro heading give a rough estimate of where the robot is after each move.
- **Scan matching (ICP)** — each new sweep gets matched against the existing map to correct the drift that odometry alone can't catch. See [ICM_SCAN_MATCHING.md](ICM_SCAN_MATCHING.md).
- **Pose graph / Graph SLAM** — instead of committing every correction immediately, poses and constraints are kept in a graph and optimized together, so a bad scan match doesn't permanently wreck the map. See [GRAPH_SLAM_GUIDE.md](GRAPH_SLAM_GUIDE.md).
- **EKF** — position is tracked as a distribution, not a single point, so the robot has a real sense of how confident it is. See [EKF_GUIDE.md](EKF_GUIDE.md).
- **Occupancy grid + frontier exploration + A\*** — the map is a plain 2D grid; the robot picks the nearest unexplored boundary and A\* plans a path to it, avoiding walls.

## Running it

```bash
pip install -r requirements.txt

# on the Pi, needs GPIO/camera access:
sudo python3 src/web_server.py
```

Then open `http://<pi-ip>` in a browser for the live camera feed and map. Most of the individual modules (`occupancy_grid.py`, `path_planner.py`, `icp.py`, etc.) also run standalone for testing — see the `*_test.py` files next to each one.

## License

MIT
