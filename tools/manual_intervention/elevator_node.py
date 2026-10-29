"""Manual Gazebo elevator driver with confirmed pose updates and an operator page."""

import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import threading
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from elevator import Elevator


class ElevatorNode(Node):
    def __init__(self, world):
        super().__init__("manual_elevator")
        self.lift = Elevator()
        self.robot = None
        self.clock = None
        self.previous = None
        self.pending = []
        self.candidate = None
        self.confirmed = False
        self.error = "Starting Gazebo connection"
        self.requests = queue.Queue()
        self.latest = {}
        self.lock = threading.Lock()
        self.client = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self.pub = self.create_publisher(String, "/elevator/state", 10)
        self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.create_subscription(
            TFMessage, "/sim/world_pose", self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(String, "/elevator/command", self.on_command, 10)
        self.create_timer(0.05, self.tick)

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_pose(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == "go2":
                p = tf.transform.translation
                self.robot = (p.x, p.y, p.z)

    def on_command(self, msg):
        try:
            self.requests.put((json.loads(msg.data), None, None))
        except ValueError:
            self.get_logger().warning("Invalid elevator command JSON")

    def status(self):
        with self.lock:
            return dict(self.latest)

    def publish(self):
        state = {
            **self.lift.state(),
            "confirmed": self.confirmed,
            "robot": self.robot,
            "error": self.error,
        }
        with self.lock:
            self.latest = state
        self.pub.publish(String(data=json.dumps(state)))

    def tick(self):
        if self.pending:
            if not all(f.done() for f in self.pending):
                self.publish()
                return
            okay = all(
                f.exception() is None and f.result().success for f in self.pending
            )
            self.pending = []
            if okay:
                self.lift = self.candidate
                self.confirmed = True
                self.error = ""
            else:
                self.confirmed = False
                self.error = "Gazebo pose update failed"
        if self.clock is None or not self.client.service_is_ready():
            self.confirmed = False
            self.publish()
            return
        while not self.requests.empty():
            command, event, result = self.requests.get()
            try:
                if not isinstance(command, dict):
                    raise ValueError("Command must be an object")
                if not self.confirmed:
                    raise ValueError("Wait for confirmed elevator state")
                self.lift.command(
                    command.get("action"), command.get("floor"), self.robot
                )
                answer = {"ok": True}
            except ValueError as error:
                answer = {"ok": False, "error": str(error)}
            if result is not None:
                result.update(answer)
                event.set()
            if not answer["ok"]:
                self.get_logger().warning(answer["error"])
        dt = (
            0.0
            if self.previous is None
            else max(0.0, min(0.1, self.clock - self.previous))
        )
        self.previous = self.clock
        candidate = copy.deepcopy(self.lift)
        candidate.step(dt, self.robot)
        if candidate.entities() != self.lift.entities() or not self.confirmed:
            self.candidate = candidate
            for name, (x, y, z) in candidate.entities().items():
                request = SetEntityPose.Request()
                request.entity.name = name
                request.pose.position.x = float(x)
                request.pose.position.y = float(y)
                request.pose.position.z = float(z)
                request.pose.orientation.w = 1.0
                self.pending.append(self.client.call_async(request))
        else:
            self.lift = candidate
        self.publish()


def serve(node, port):
    page = Path(__file__).with_name("elevator.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, data, content="application/json"):
            self.send_response(status)
            self.send_header("Content-Type", content)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def test_proxy(self, action):
            try:
                req = Request("http://127.0.0.1:8110/" + action,
                              data=None if action == "status" else b"{}",
                              headers={"Content-Type": "application/json"})
                with urlopen(req, timeout=5) as response:
                    self.send(200, response.read())
            except HTTPError as error:
                self.send(error.code, error.read())
            except OSError:
                self.send(503, b'{"status":"unavailable","ok":false,"error":"Robot test service unavailable"}')

        def do_GET(self):
            if self.path == "/test/status":
                self.test_proxy("status")
            elif self.path == "/status":
                self.send(200, json.dumps(node.status()).encode())
            elif self.path == "/":
                self.send(200, page, "text/html; charset=utf-8")
            else:
                self.send(404, b"{}")

        def do_POST(self):
            if self.path in ("/test/start", "/test/cancel"):
                self.test_proxy(self.path.rsplit("/", 1)[1])
                return
            if self.path not in ("/command", "/robot-command"):
                self.send(404, b"{}")
                return
            if self.path == "/robot-command" and self.client_address[0] != "127.0.0.1":
                self.send(403, b'{"ok":false,"error":"Robot commands require loopback"}')
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("Invalid command size")
                command = json.loads(self.rfile.read(size))
                try:
                    with urlopen("http://127.0.0.1:8110/status", timeout=1) as response:
                        if self.path == "/command" and json.load(response).get("status") in ("running", "stopping"):
                            self.send(409, b'{"ok":false,"error":"Cancel Test before manual elevator commands"}')
                            return
                except OSError:
                    pass
                event = threading.Event()
                result = {}
                node.requests.put((command, event, result))
                if not event.wait(3):
                    self.send(
                        503, b'{"ok":false,"error":"Elevator driver unavailable"}'
                    )
                    return
                self.send(200 if result["ok"] else 409, json.dumps(result).encode())
            except ValueError as error:
                self.send(400, json.dumps({"ok": False, "error": str(error)}).encode())

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="elevator")
    parser.add_argument("--port", type=int, default=8092)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = ElevatorNode(args.world)
    server = serve(node, args.port)
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
