"""SLAM system with background thread for web control.

Runs a stop-and-scan loop in a background thread:
  - Wait for target
  - Move to target (blocking)
  - Scan + ICP correct + update map
  - Repeat
"""

import math
import threading
import time
import traceback

import lgpio
import numpy as np

from icp import icp
from occupancy_grid import OccupancyGrid
from robot.config import TOF_OFFSET_X, TOF_OFFSET_Y
from robot.drive_dc import RobotDC
from scanner import Scanner


class SlamSystem:
    def __init__(self, use_icp=True):
        self.use_icp = use_icp

        # Shared state (GIL-safe for simple reads/writes)
        self.state = "IDLE"  # IDLE, MOVING, SCANNING, ERROR
        self.pose = (0.0, 0.0, 0.0)  # x, y, heading_deg
        self.target = None  # (x, y) or None
        self.message = "Initializing..."
        self.map_version = 0
        self.icp_result = None  # dict with last ICP result info

        # Hardware
        self.chip = None
        self.robot = None
        self.scanner = None
        self.grid = OccupancyGrid()

        self._running = False
        self._thread = None

    def start(self):
        """Start background control loop."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop and cleanup."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.robot:
            self.robot.close()
        if self.chip:
            lgpio.gpiochip_close(self.chip)

    def set_target(self, x, y):
        """Set navigation target. Returns False if busy."""
        if self.state != "IDLE":
            return False
        self.target = (x, y)
        self.message = f"Target set: ({x:.1f}, {y:.1f})"
        return True

    def get_map_data(self):
        """Return cropped map data for the frontend.

        Returns dict with:
          grid: 2D list of probabilities (0=free, 1=occupied)
          bounds: [x_min, x_max, y_min, y_max] in cm
          rows, cols: dimensions
        """
        prob_map = self.grid.get_probability_map()
        r0, r1, c0, c1 = self.grid._get_data_bounds(padding=10)
        cropped = prob_map[r0 : r1 + 1, c0 : c1 + 1]

        x_min, y_min = self.grid.grid_to_world(r0, c0)
        x_max, y_max = self.grid.grid_to_world(r1, c1)

        return {
            "grid": np.round(cropped, 2).tolist(),
            "bounds": [x_min, x_max, y_min, y_max],
            "rows": cropped.shape[0],
            "cols": cropped.shape[1],
            "version": self.map_version,
        }

    # ── Background loop ───────────────────────────────────────────────

    def _loop(self):
        """Main background loop."""
        self._init_hardware()

        while self._running:
            if self.target:
                target = self.target
                self.target = None
                self._move_scan_update(target)
            time.sleep(0.1)

    def _init_hardware(self):
        """Initialize hardware, take initial scan."""
        try:
            print("Initializing hardware...")
            self.chip = lgpio.gpiochip_open(4)
            self.robot = RobotDC(self.chip)
            self.scanner = Scanner()

            # Initial scan at origin
            self._scan_and_update()
            self.state = "IDLE"
            self.message = "Ready"
            print("Hardware ready.")

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Init error: {e}"
            traceback.print_exc()

    def _move_scan_update(self, target):
        """Move to target, then scan and update map."""
        tx, ty = target

        # 1. Move
        self.state = "MOVING"
        self.message = "Calibrating gyro..."
        try:
            # ZUPT: re-zero gyro bias while stopped to combat heading drift
            self.robot.imu.calibrate_gyro(samples=100)

            self.message = f"Moving to ({tx:.1f}, {ty:.1f})..."
            print(f"Moving to ({tx:.1f}, {ty:.1f})...")
            self.robot.move_to(tx, ty)
            self.pose = self.robot.get_pose()
        except Exception as e:
            self.state = "ERROR"
            self.message = f"Move failed: {e}"
            traceback.print_exc()
            return

        # 2. Scan + update
        self._scan_and_update()

        self.state = "IDLE"
        self.message = "Ready"

    def _scan_and_update(self):
        """Scan, optionally ICP correct, update grid."""
        self.state = "SCANNING"
        self.message = "Scanning..."

        try:
            scan = self.scanner.scan()
            pose = self.robot.get_pose()

            # ICP correction: scan-to-map matching
            if self.use_icp and len(scan) > 5:
                map_points = self.grid.get_occupied_points()

                if len(map_points) > 10:
                    scan_world = self._scan_to_world(scan, pose)
                    R, t, _, ok = icp(scan_world, map_points, max_distance=20)

                    if ok:
                        corr_pos = R @ np.array([pose[0], pose[1]]) + t
                        corr_angle = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                        dx = corr_pos[0] - pose[0]
                        dy = corr_pos[1] - pose[1]
                        self.robot.x = corr_pos[0]
                        self.robot.y = corr_pos[1]
                        self.robot._heading += corr_angle
                        pose = self.robot.get_pose()
                        self.icp_result = {
                            "status": "converged",
                            "dx": round(dx, 1),
                            "dy": round(dy, 1),
                            "dtheta": round(corr_angle, 1),
                            "map_pts": len(map_points),
                        }
                        print(
                            f"ICP converged: dx={dx:.1f} dy={dy:.1f} dθ={corr_angle:.1f}° (map: {len(map_points)} pts)"
                        )
                    else:
                        self.icp_result = {
                            "status": "failed",
                            "map_pts": len(map_points),
                        }
                        print(f"ICP failed to converge (map: {len(map_points)} pts)")
                else:
                    self.icp_result = {"status": "building_map"}

            # Update grid
            self.grid.update(scan, pose)
            self.pose = pose
            self.map_version += 1

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Scan error: {e}"
            traceback.print_exc()

    def _scan_to_world(self, scan_sensor, pose):
        """Transform sensor-frame scan points to world frame."""
        x, y, heading_deg = pose
        theta = math.radians(heading_deg)
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])
        scan_robot = scan_sensor + [TOF_OFFSET_X, TOF_OFFSET_Y]
        return (R @ scan_robot.T).T + np.array([x, y])
