"""Flask web server for SLAM robot control.

Run on Pi: sudo python3 web_server.py
Then open http://<pi-ip> in browser.
"""

import logging
import signal
import subprocess
import sys

from flask import Flask, Response, jsonify, render_template, request

from graph_slam_system import GraphSlamSystem

app = Flask(__name__)
slam = GraphSlamSystem()

CAMERA_CMD = [
    "rpicam-vid",
    "-t", "0",
    "--codec", "mjpeg",
    "--width", "1280",
    "--height", "720",
    "--framerate", "20",
    "--quality", "85",
    "--rotation", "180",
    "--inline",
    "-o", "-",
]


@app.route("/")
def index():
    return render_template("index.html")


def _mjpeg_frames():
    """Run rpicam-vid and split its MJPEG stdout into multipart frames."""
    proc = subprocess.Popen(CAMERA_CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    buf = b""
    try:
        while True:
            chunk = proc.stdout.read(4096)
            if not chunk:
                break
            buf += chunk
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9")
            if start != -1 and end != -1 and end > start:
                frame = buf[start : end + 2]
                buf = buf[end + 2 :]
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


@app.route("/video_feed")
def video_feed():
    return Response(_mjpeg_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/state")
def get_state():
    """Return pose, status, target."""
    current_pose = slam.robot.get_pose() if slam.robot else slam.pose
    return jsonify(
        {
            "state": slam.state,
            "pose": current_pose,
            "target": slam.target,
            "message": slam.message,
            "map_version": slam.map_version,
            "icp": slam.icp_result,
            "path": slam.path_history,
            "planned": slam.planned_waypoints,
            "icp_corrections": slam.icp_corrections,
            "pid": slam.pid_summary,
            "exploring": slam._exploring,
            "explore_goal": slam.explore_goal,
            "frontiers": slam.frontier_data,
            "graph": getattr(slam, "graph_info", None),
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
    ok = slam.explore()
    if ok:
        return jsonify({"status": "ok"})
    return jsonify({"status": "busy"}), 409


@app.route("/api/explore/stop", methods=["POST"])
def stop_explore():
    slam.stop_explore()
    return jsonify({"status": "ok"})


@app.route("/api/graph")
def get_graph():
    """Return full pose graph data for visualization."""
    if hasattr(slam, "get_graph_data"):
        return jsonify(slam.get_graph_data())
    return jsonify({"error": "Graph SLAM not active"}), 404


def shutdown(sig, frame):
    slam.stop()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    slam.auto_explore = True
    slam.start()
    # Suppress per-request access logs from werkzeug
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    print("Server starting on http://0.0.0.0:80")
    app.run(host="0.0.0.0", port=80, debug=False, use_reloader=False, threaded=True)
