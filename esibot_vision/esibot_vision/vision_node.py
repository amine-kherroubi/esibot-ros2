#!/usr/bin/env python3
"""
vision_node.py - EsiBot ROS 2 vision node
=========================================
Integrates: LaneDetector (OpenCV) + SignDetector (YOLOv8n GTSRB) + ObstacleDetector (YOLOv8n COCO)

Frames are pulled DIRECTLY from the ESP32-CAM MJPEG stream (no intermediate
camera node), which removes a full raw-image ROS hop and keeps the pipeline fluid.

Published topics:
  /camera/image_annotated/compressed  sensor_msgs/CompressedImage  (JPEG)
  /esibot/lane_error          std_msgs/Float32   [-1.0 to +1.0]
  /esibot/lane_status         std_msgs/String    [IN_LANE|LANE_LEFT|LANE_RIGHT|NO_LANE]
  /esibot/signs               std_msgs/String    JSON list
  /esibot/obstacles           std_msgs/String    JSON list
  /esibot/obstacle_in_lane    std_msgs/Bool
"""

import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

from esibot_logging import get_logger, setup_logging
from esibot_vision.lane_detector import LaneDetector
from esibot_vision.sign_detector import SignDetector
from esibot_vision.obstacle_detector import ObstacleDetector
from esibot_vision.utils import FPSCounter, draw_hud


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")
        self.log = get_logger(node=self)

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)

        # ESP32-CAM source (direct MJPEG, no camera node)
        self.declare_parameter("esp32_ip", "192.168.1.80")
        self.declare_parameter("esp32_port", 80)
        self.declare_parameter("stream_path", "/stream")
        # Optional: subscribe to a ROS image topic (CompressedImage) instead
        # of pulling the ESP32 MJPEG stream. When set, vision_node will use the
        # topic and skip the MJPEG capture thread.
        self.declare_parameter("camera_image_topic", "")
        self.declare_parameter("reconnect_delay", 3.0)
        self.declare_parameter("jpeg_quality", 80)

        # Lane
        self.declare_parameter("lane_detection", True)
        self.declare_parameter("lane_roi_ratio", 0.45)
        self.declare_parameter("lane_threshold", 60)
        self.declare_parameter("lane_min_area", 200)

        # Signs
        self.declare_parameter("sign_model_path", "")
        self.declare_parameter("sign_conf", 0.50)

        # Obstacles
        self.declare_parameter("obstacle_model_path", "")
        self.declare_parameter("obstacle_conf", 0.40)
        self.declare_parameter("obstacle_roi_ratio", 0.55)
        self.declare_parameter("lane_width_ratio", 0.50)

        # Misc
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("process_rate", 15.0)

        self._load_params()

        # ── Detectors ────────────────────────────────────────────────────
        self._lane = LaneDetector(
            roi_ratio=self.lane_roi_ratio,
            threshold=self.lane_threshold,
            min_area=self.lane_min_area,
        )
        self._signs = SignDetector(
            model_path=self.sign_model_path,
            conf_threshold=self.sign_conf,
        )
        self._obstacles = ObstacleDetector(
            model_path=self.obstacle_model_path,
            conf_threshold=self.obstacle_conf,
            roi_ratio=self.obstacle_roi_ratio,
            lane_width_ratio=self.lane_width_ratio,
        )

        # ── Thread-safe state ─────────────────────────────────────────────
        self._frame = None
        self._frame_lock = threading.Lock()
        self._last_frame_time = time.time()
        self._running = True
        self._stream = None  # persistent HTTP stream handle
        self._buf = bytes()

        # Detection results shared between the YOLO detection thread and the
        # fast publish timer — decoupled so the video stays fluid while the
        # (slow) detector updates boxes at its own rate.
        self._det_lock = threading.Lock()
        self._det_obstacles = []
        self._det_signs = []
        self._det_in_lane = False
        self._lane_cx_shared = (None, None)

        self._fps = FPSCounter()

        # ── Frame source: ESP32-CAM MJPEG (dedicated capture thread) ───────
        # If a ROS topic is provided, subscribe to it (CompressedImage); else
        # fallback to pulling the ESP32 MJPEG stream in a capture thread.
        self.stream_url = (
            f"http://{self.esp32_ip}:{self.esp32_port}{self.stream_path}"
        )
        self._using_topic = bool(self.camera_image_topic)
        if self._using_topic:
            self.log.info(f"vision_node — subscribing to {self.camera_image_topic}")
            self._image_sub = self.create_subscription(
                CompressedImage,
                self.camera_image_topic,
                self._on_compressed_image,
                10,
            )
        else:
            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True, name="vision_capture"
            )
            self._capture_thread.start()

        # ── Publishers ───────────────────────────────────────────────────
        self._annotated_pub = self.create_publisher(
            CompressedImage, "/camera/image_annotated/compressed", 10
        )
        self._lane_error_pub = self.create_publisher(
            Float32, "/esibot/lane_error", 10
        )
        self._lane_status_pub = self.create_publisher(
            String, "/esibot/lane_status", 10
        )
        self._signs_pub = self.create_publisher(String, "/esibot/signs", 10)
        self._obstacles_pub = self.create_publisher(
            String, "/esibot/obstacles", 10
        )
        self._obstacle_in_lane_pub = self.create_publisher(
            Bool, "/esibot/obstacle_in_lane", 10
        )

        # ── Publish timer (fast, fluid video) ─────────────────────────────
        self.create_timer(1.0 / self.process_rate, self._process)

        # ── Detection thread (YOLO) — runs independently of publish rate ──
        self._detect_thread = threading.Thread(
            target=self._detection_loop, daemon=True, name="vision_detect"
        )
        self._detect_thread.start()

        self.log.info(
            f"vision_node started - {self.process_rate:.0f} Hz | "
            f"lane {'ON' if self.lane_detection else 'OFF'} | "
            f"signs {'OK' if self._signs.available else 'DISABLED'} | "
            f"obstacles {'OK' if self._obstacles.available else 'DISABLED'}"
        )

    # ─────────────────────────────────────────────────────────────────────
    def _load_params(self):
        self.img_w = self.get_parameter("image_width").value
        self.img_h = self.get_parameter("image_height").value

        self.esp32_ip = self.get_parameter("esp32_ip").value
        self.esp32_port = int(self.get_parameter("esp32_port").value)
        self.stream_path = self.get_parameter("stream_path").value
        self.camera_image_topic = self.get_parameter("camera_image_topic").value
        self.reconnect_delay = float(self.get_parameter("reconnect_delay").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.lane_detection = bool(self.get_parameter("lane_detection").value)
        self.lane_roi_ratio = float(self.get_parameter("lane_roi_ratio").value)
        self.lane_threshold = int(self.get_parameter("lane_threshold").value)
        self.lane_min_area = int(self.get_parameter("lane_min_area").value)

        self.sign_model_path = self.get_parameter("sign_model_path").value
        self.sign_conf = float(self.get_parameter("sign_conf").value)

        self.obstacle_model_path = self.get_parameter("obstacle_model_path").value
        self.obstacle_conf = float(self.get_parameter("obstacle_conf").value)
        self.obstacle_roi_ratio = float(self.get_parameter("obstacle_roi_ratio").value)
        self.lane_width_ratio = float(self.get_parameter("lane_width_ratio").value)

        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.process_rate = float(self.get_parameter("process_rate").value)

    # ── Frame capture (dedicated thread) ──────────────────────────────────
    def _capture_loop(self):
        self.log.info(f"vision_node — capturing from {self.stream_url}")

        while self._running:
            try:
                frame = self._capture_from_esp32()
                if frame is None:
                    continue
                if frame.shape[:2] != (self.img_h, self.img_w):
                    frame = cv2.resize(frame, (self.img_w, self.img_h))
                with self._frame_lock:
                    self._frame = frame
                    self._last_frame_time = time.time()
            except Exception as e:
                self.log.error(f"Capture error: {e}", throttle_duration_sec=5.0)
                self._stream = None  # force reconnect
                time.sleep(self.reconnect_delay)

    def _capture_from_esp32(self):
        """Pull one decoded BGR frame from the ESP32-CAM MJPEG stream."""
        if self._stream is None:
            self._buf = bytes()
            try:
                self.log.info(f"Connecting to ESP32-CAM: {self.stream_url}")
                self._stream = urllib.request.urlopen(self.stream_url, timeout=10)
                self.log.info("ESP32-CAM connected.")
            except urllib.error.URLError as e:
                self.log.warning(
                    f"ESP32 unreachable: {e.reason} — retry in {self.reconnect_delay}s",
                    throttle_duration_sec=self.reconnect_delay,
                )
                self._stream = None
                time.sleep(self.reconnect_delay)
                return None

        chunk = self._stream.read(16384)
        if not chunk:
            self._stream = None
            return None
        self._buf += chunk

        # Parse the multipart stream by Content-Length (robust: 0xFFD9 also
        # occurs inside JPEG data, so SOI/EOI scanning mis-frames). Drain the
        # whole buffer and keep only the most recent frame to stay low-latency.
        latest = None
        while True:
            header_end = self._buf.find(b"\r\n\r\n")
            if header_end == -1:
                break
            header = self._buf[:header_end].lower()
            cl_idx = header.find(b"content-length:")
            if cl_idx == -1:
                break
            try:
                cl = int(header[cl_idx + 15 :].split(b"\r\n", 1)[0])
            except ValueError:
                self._buf = self._buf[header_end + 4 :]
                continue
            body_start = header_end + 4
            if len(self._buf) < body_start + cl:
                break  # frame incomplete — wait for more data
            latest = self._buf[body_start : body_start + cl]
            self._buf = self._buf[body_start + cl :]

        if latest is None:
            return None
        arr = np.frombuffer(latest, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _on_compressed_image(self, msg: CompressedImage):
        """Callback for CompressedImage messages when subscribing to a camera topic."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            if frame.shape[:2] != (self.img_h, self.img_w):
                frame = cv2.resize(frame, (self.img_w, self.img_h))
            with self._frame_lock:
                self._frame = frame
                self._last_frame_time = time.time()
        except Exception as e:
            self.log.error(f"Failed to decode CompressedImage: {e}", throttle_duration_sec=5.0)

    # ── Publish timer (fast) — fresh live frame + last detection overlay ──
    def _process(self):
        with self._frame_lock:
            if self._frame is None:
                return
            frame = self._frame.copy()
            last_t = self._last_frame_time

        if time.time() - last_t > 5.0:
            self.log.warning("No frame received for 5s", throttle_duration_sec=5.0)
            return

        annotated = frame.copy()

        # Lane detection (OpenCV, cheap) — optional, fresh on every frame.
        if self.lane_detection:
            lane_error, lane_status, annotated, left_cx, right_cx = self._lane.detect(
                frame, annotated
            )
        else:
            lane_error, lane_status, left_cx, right_cx = 0.0, "OFF", None, None

        with self._det_lock:
            self._lane_cx_shared = (left_cx, right_cx)
            obstacle_dets = list(self._det_obstacles)
            sign_labels = [d["label"] for d in self._det_signs]
            obstacle_in_lane = self._det_in_lane

        # Draw the latest obstacle boxes (computed by the detection thread)
        # onto the fresh live frame.
        for det in obstacle_dets:
            x1, y1, x2, y2 = det["bbox"]
            color = det.get("color", (255, 255, 255))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                annotated,
                f"{det['label']} {det['proximity']}",
                (x1, max(y1 - 5, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                color,
                1,
            )

        self._fps.tick()
        draw_hud(
            img=annotated,
            fps=self._fps.get(),
            lane_error=lane_error,
            lane_status=lane_status,
            sign_labels=sign_labels,
            obstacle_in_lane=obstacle_in_lane,
        )
        self._publish_image(annotated)

        # Lane topics (only when lane detection is enabled).
        if self.lane_detection:
            msg_err = Float32()
            msg_err.data = float(lane_error)
            self._lane_error_pub.publish(msg_err)
            msg_status = String()
            msg_status.data = lane_status
            self._lane_status_pub.publish(msg_status)
            if lane_status != "IN_LANE":
                self.log.info(
                    f"Lane: {lane_status}  err={lane_error:+.2f}",
                    throttle_duration_sec=1.0,
                )

    # ── Detection thread (slow) — YOLO obstacles + signs ──────────────────
    def _detection_loop(self):
        while self._running:
            with self._frame_lock:
                frame = None if self._frame is None else self._frame.copy()
            if frame is None:
                time.sleep(0.03)
                continue

            with self._det_lock:
                lcx, rcx = self._lane_cx_shared

            sign_dets, _ = self._signs.detect(frame, None)
            obstacle_dets, in_lane, _ = self._obstacles.detect(
                frame, None, lane_left_cx=lcx, lane_right_cx=rcx
            )

            with self._det_lock:
                self._det_signs = sign_dets
                self._det_obstacles = obstacle_dets
                self._det_in_lane = in_lane

            # Detection topics published at the detection rate.
            msg_signs = String()
            msg_signs.data = json.dumps(
                [{"label": d["label"], "conf": d["conf"]} for d in sign_dets]
            )
            self._signs_pub.publish(msg_signs)

            msg_obs = String()
            msg_obs.data = json.dumps(
                [
                    {
                        "label": o["label"],
                        "conf": o["conf"],
                        "in_lane": o["in_lane"],
                        "proximity": o["proximity"],
                    }
                    for o in obstacle_dets
                ]
            )
            self._obstacles_pub.publish(msg_obs)

            msg_in_lane = Bool()
            msg_in_lane.data = in_lane
            self._obstacle_in_lane_pub.publish(msg_in_lane)

            if in_lane:
                self.log.warning(
                    f"OBSTACLE IN LANE ({len(obstacle_dets)} detected)",
                    throttle_duration_sec=0.5,
                )

    # ─────────────────────────────────────────────────────────────────────
    def _publish_image(self, img: np.ndarray):
        ok, buf = cv2.imencode(
            ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self._annotated_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
