#!/usr/bin/env python3
"""
EsiBot Camera Stream Node
==========================
"""

import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Try to reach the ESP32-CAM network stack.
# If urllib itself is unavailable (headless sim host), fall back to sim mode.
try:
    import urllib.request as _probe   
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("urllib unavailable — running in SIMULATION/MOCK mode.")


class CameraStreamNode(Node):

    def __init__(self):
        super().__init__("esibot_camera_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("esp32_ip",        "192.168.1.80")
        self.declare_parameter("esp32_port",      80)
        self.declare_parameter("stream_path",     "/stream")
        self.declare_parameter("frame_width",     320)
        self.declare_parameter("frame_height",    240)
        self.declare_parameter("publish_rate",    10.0)
        self.declare_parameter("camera_frame",    "camera_optical_frame")
        self.declare_parameter("show_fps",        True)
        self.declare_parameter("reconnect_delay", 3.0)
        self.declare_parameter("sim_mode",        False)   

        self.esp32_ip        = self.get_parameter("esp32_ip").value
        self.esp32_port      = self.get_parameter("esp32_port").value
        self.stream_path     = self.get_parameter("stream_path").value
        self.frame_w         = self.get_parameter("frame_width").value
        self.frame_h         = self.get_parameter("frame_height").value
        self.publish_rate    = self.get_parameter("publish_rate").value
        self.camera_frame    = self.get_parameter("camera_frame").value
        self.show_fps        = self.get_parameter("show_fps").value
        self.reconnect_delay = self.get_parameter("reconnect_delay").value
        self.sim_mode        = self.get_parameter("sim_mode").value or not HARDWARE_AVAILABLE

        self.stream_url = (
            f"http://{self.esp32_ip}:{self.esp32_port}{self.stream_path}"
        )

        # ── Shared state ─────────────────────────────────────────────────────
        self.latest_frame = None
        self.frame_lock   = threading.Lock()
        self.frame_count  = 0
        self.fps          = 0.0
        self.fps_timer    = time.time()
        self.running      = True
        self._stream      = None   # persistent HTTP stream handle (live mode)
        self._buf         = bytes()

        if self.sim_mode:
            # Mirrors radar node: "No hardware detected. Publishing simulated…"
            self.get_logger().info(
                "No hardware detected. Publishing simulated camera frames."
            )

        # ── Publishers ───────────────────────────────────────────────────────
        self.image_pub  = self.create_publisher(Image,      "/camera/image_raw",   10)
        self.info_pub   = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.status_pub = self.create_publisher(String,     "/camera/status",      10)

        # ── Publish timer (ROS executor thread) ──────────────────────────────
        self.pub_timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)

        # ── Capture thread (dedicated — keeps executor thread unblocked) ─────
        self.capture_thread = threading.Thread(
            target=self.capture_loop, daemon=True
        )
        self.capture_thread.start()

        self.get_logger().info(
            f"esibot_camera — mode={'SIM' if self.sim_mode else 'LIVE'}"
            + (f", URL: {self.stream_url}" if not self.sim_mode else "")
        )

    # ── Capture loop (runs in dedicated thread) ───────────────────────────────

    def capture_loop(self):
        while self.running:
            try:
                # Single entry point — mirrors read_distance() in radar node.
                frame = self.capture_frame()

                if frame is not None:
                    if self.frame_count == 0:
                        self.get_logger().info(
                            f"First frame: {frame.shape}, "
                            f"mean={frame.mean():.1f}"
                        )
                    processed = self.process_frame(frame)
                    with self.frame_lock:
                        self.latest_frame = processed
                    self.frame_count += 1

            except Exception as e:
                self.get_logger().error(f"Capture error: {e}")
                self._publish_status(f"ERROR: {e}")
                self._stream = None   # force reconnect on next iteration
                time.sleep(self.reconnect_delay)

    # ── Hardware / simulation abstraction ────────────────────────────────────

    def capture_frame(self) -> np.ndarray | None:
        """
        Return a BGR frame as a numpy array.
        """
        if not self.sim_mode:
            return self._capture_from_esp32()
        else:
            # Mimic ~1-frame acquisition delay (radar node uses time.sleep(0.01))
            time.sleep(1.0 / self.publish_rate)
            return self._make_sim_frame()

    # ── Real hardware path ───────────────────────────────────────────────────

    def _capture_from_esp32(self) -> np.ndarray | None:
        """
        Pull one JPEG frame from the ESP32-CAM MJPEG stream.
        Reconnects automatically on loss of connection.
        Only called when sim_mode is False.
        """
        if self._stream is None:
            self._buf = bytes()
            try:
                self.get_logger().info(
                    f"Connecting to ESP32-CAM: {self.stream_url}"
                )
                self._stream = urllib.request.urlopen(
                    self.stream_url, timeout=10
                )
                self.get_logger().info("ESP32-CAM connected.")
                self._publish_status("CONNECTED")
            except urllib.error.URLError as e:
                self.get_logger().warn(f"ESP32 unreachable: {e.reason}")
                self._publish_status(f"DISCONNECTED: {e.reason}")
                self.get_logger().info(
                    f"Reconnecting in {self.reconnect_delay}s..."
                )
                self._stream = None
                time.sleep(self.reconnect_delay)
                return None

        chunk = self._stream.read(4096)
        if not chunk:
            self._stream = None
            return None
        self._buf += chunk

        # Locate JPEG frame boundaries in the MJPEG byte stream
        start = self._buf.find(b"\xff\xd8")   # JPEG SOI marker
        end   = self._buf.find(b"\xff\xd9")   # JPEG EOI marker

        if start != -1 and end != -1 and end > start:
            jpg           = self._buf[start : end + 2]
            self._buf     = self._buf[end + 2 :]   # keep remainder only
            arr           = np.frombuffer(jpg, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)

        return None

    # ── Simulation frame generator ────────────────────────────────────────────

    def _make_sim_frame(self) -> np.ndarray:
        """
        Synthetic BGR test frame: animated scrolling hue gradient + info bar.
        Gives a visually moving image so subscribers can confirm the pipeline
        is alive — mirrors the radar node's flat-wall + Gaussian-noise fake.
        """
        t     = time.time()
        frame = np.zeros((self.frame_h, self.frame_w, 3), dtype=np.uint8)

        # Scrolling hue gradient (advances ~30°/s)
        for x in range(self.frame_w):
            hue     = int((x / self.frame_w * 180 + t * 30) % 180)
            hsv_col = np.uint8([[[hue, 220, 200]]])
            bgr_col = cv2.cvtColor(hsv_col, cv2.COLOR_HSV2BGR)[0][0]
            frame[:, x] = bgr_col

        # Info overlay
        ts = time.strftime("%H:%M:%S")
        cv2.putText(
            frame, f"SIM {self.frame_w}x{self.frame_h} {ts}",
            (4, 14), cv2.FONT_HERSHEY_SIMPLEX,
            0.4, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return frame

    # ── Frame processing ─────────────────────────────────────────────────────

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Post-process a decoded BGR frame before publishing."""
        if frame.shape[:2] != (self.frame_h, self.frame_w):
            self.get_logger().warn(
                f"Frame size mismatch: got {frame.shape[1]}x{frame.shape[0]}, "
                f"resizing to {self.frame_w}x{self.frame_h}",
                throttle_duration_sec=5.0
            )
            frame = cv2.resize(frame, (self.frame_w, self.frame_h))

        # FPS / debug overlay — uncomment to enable
        # if self.show_fps:
        #     self._update_fps()
        #     cv2.putText(frame, f'FPS:{self.fps:.1f}',
        #         (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
        #         0.6, (0, 255, 0), 2, cv2.LINE_AA)
        #     cv2.putText(frame, f'{self.frame_w}x{self.frame_h}',
        #         (10, 45), cv2.FONT_HERSHEY_SIMPLEX,
        #         0.5, (0, 200, 255), 1, cv2.LINE_AA)
        #     cv2.putText(frame, 'ESP32-CAM->ROS2',
        #         (10, 68), cv2.FONT_HERSHEY_SIMPLEX,
        #         0.5, (255, 100, 0), 1, cv2.LINE_AA)
        #     ts = time.strftime('%H:%M:%S')
        #     cv2.putText(frame, ts,
        #         (self.frame_w - 80, 22), cv2.FONT_HERSHEY_SIMPLEX,
        #         0.5, (200, 200, 200), 1, cv2.LINE_AA)

        return frame

    def _update_fps(self):
        now     = time.time()
        elapsed = now - self.fps_timer
        if elapsed >= 1.0:
            self.fps         = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_timer   = now

    # ── Publish frame + camera_info (called by ROS timer) ────────────────────

    def publish_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()

        # Capture stamp once — used for both Image and CameraInfo headers
        # so that image_proc / camera_calibration see identical timestamps.
        stamp = self.get_clock().now().to_msg()

        # ── Image message ────────────────────────────────────────────────────
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.camera_frame
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = "bgr8"   # cv2.imdecode always returns BGR
        msg.is_bigendian    = False
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        self.image_pub.publish(msg)

        self._publish_camera_info(stamp)

    def _publish_camera_info(self, stamp):
        """Publish a CameraInfo message with placeholder intrinsics."""
        fx = 160.0
        fy = 160.0
        cx = self.frame_w / 2.0
        cy = self.frame_h / 2.0

        info = CameraInfo()
        info.header.stamp     = stamp
        info.header.frame_id  = self.camera_frame
        info.width            = self.frame_w
        info.height           = self.frame_h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx,  0.0, cx,
                  0.0, fy,  cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx,  0.0, cx,  0.0,
                  0.0, fy,  cy,  0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False   # signals capture_loop to exit
        # Mirrors radar node's "if HARDWARE_AVAILABLE" teardown guard
        if not self.sim_mode and self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()