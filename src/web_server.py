"""Flask web server for SLAM robot control.

Run on Pi: sudo python3 web_server.py
Then open http://<pi-ip> in browser.
"""

import signal
import sys

from flask import Flask, jsonify, render_template, request

from slam_system import SlamSystem

app = Flask(__name__)
slam = SlamSystem()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def get_state():
    """Return pose, status, target."""
    return jsonify(
        {
            "state": slam.state,
            "pose": slam.pose,
            "target": slam.target,
            "message": slam.message,
            "map_version": slam.map_version,
            "icp": slam.icp_result,
            "path": slam.path_history,
            "planned": slam.planned_waypoints,
            "icp_corrections": slam.icp_corrections,
            "pid": slam.pid_summary,
            "frontier_target": slam.frontier_target,
            "frontier_clusters": [
                {"x": c["centroid_xy"][0], "y": c["centroid_xy"][1], "size": c["size"]}
                for c in slam.frontier_clusters
            ],
        }
    )


@app.route("/api/map")
def get_map():
    """Return cropped occupancy grid as JSON."""
    return jsonify(slam.get_map_data())


@app.route("/api/target", methods=["POST"])
def set_target():
    data = request.json
    x, y = float(data["x"]), float(data["y"])
    ok = slam.set_target(x, y)
    if ok:
        return jsonify({"status": "ok"})
    return jsonify({"status": "busy"}), 409


@app.route("/api/explore", methods=["POST"])
def start_explore():
    if slam.state != "IDLE":
        return jsonify({"status": "busy"}), 409
    import threading

    threading.Thread(target=slam.explore, daemon=True).start()
    return jsonify({"status": "ok"})


def shutdown(sig, frame):
    slam.stop()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    slam.start()
    print("Server starting on http://0.0.0.0:80")
    app.run(host="0.0.0.0", port=80, debug=False, use_reloader=False)
