#!/usr/bin/env python3
"""
vision_node.py - EsiBot ROS 2 vision node
=========================================
Integrates: LaneDetector (OpenCV) + SignDetector (YOLOv8n GTSRB) + ObstacleDetector (YOLOv8n COCO)

Subscribed topics:
  /image_raw                  sensor_msgs/Image

Published topics:
  /camera/image_annotated     sensor_msgs/Image
  /esibot/lane_error          std_msgs/Float32   [-1.0 to +1.0]
  /esibot/lane_status         std_msgs/String    [IN_LANE|LANE_LEFT|LANE_RIGHT|NO_LANE]
  /esibot/signs               std_msgs/String    JSON list
  /esibot/obstacles           std_msgs/String    JSON list
  /esibot/obstacle_in_lane    std_msgs/Bool
"""

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Lane
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

        self._fps = FPSCounter()

        # ── Subscriber ───────────────────────────────────────────────────
        self.create_subscription(Image, "/image_raw", self._cb_image, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._annotated_pub = self.create_publisher(
            Image, "/camera/image_annotated", 10
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

        # ── Timer ────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.process_rate, self._process)

        self.log.info(
            f"vision_node started - {self.process_rate:.0f} Hz | "
            f"lane OK | "
            f"signs {'OK' if self._signs.available else 'DISABLED'} | "
            f"obstacles {'OK' if self._obstacles.available else 'DISABLED'}"
        )

    # ─────────────────────────────────────────────────────────────────────
    def _load_params(self):
        self.img_w = self.get_parameter("image_width").value
        self.img_h = self.get_parameter("image_height").value

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

    # ─────────────────────────────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        frame = (
            np.frombuffer(msg.data, dtype=np.uint8)
            .reshape((msg.height, msg.width, 3))
            .copy()
        )
        with self._frame_lock:
            self._frame = frame
            self._last_frame_time = time.time()

    # ─────────────────────────────────────────────────────────────────────
    def _process(self):
        with self._frame_lock:
            if self._frame is None:
                return
            frame = self._frame.copy()

        if time.time() - self._last_frame_time > 5.0:
            self.log.warning(
                "No frame received for 5s", throttle_duration_sec=5.0
            )

        self._fps.tick()
        annotated = frame.copy() if self.publish_annotated else None

        # ── A) Lane detection ─────────────────────────────────────────────
        lane_error, lane_status, annotated, left_cx, right_cx = self._lane.detect(
            frame, annotated
        )

        # ── B) Sign detection (full image) ────────────────────────────────
        sign_detections, annotated = self._signs.detect(frame, annotated)

        # ── C) Obstacle detection (only between the two tapes) ────────────
        obstacle_detections, obstacle_in_lane, annotated = self._obstacles.detect(
            frame, annotated, lane_left_cx=left_cx, lane_right_cx=right_cx
        )

        # ── Publishing ───────────────────────────────────────────────────
        msg_err = Float32()
        msg_err.data = float(lane_error)
        self._lane_error_pub.publish(msg_err)

        msg_status = String()
        msg_status.data = lane_status
        self._lane_status_pub.publish(msg_status)

        msg_signs = String()
        msg_signs.data = json.dumps(
            [{"label": d["label"], "conf": d["conf"]} for d in sign_detections]
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
                for o in obstacle_detections
            ]
        )
        self._obstacles_pub.publish(msg_obs)

        msg_in_lane = Bool()
        msg_in_lane.data = obstacle_in_lane
        self._obstacle_in_lane_pub.publish(msg_in_lane)

        # ── HUD + annotated image ─────────────────────────────────────────
        if self.publish_annotated and annotated is not None:
            draw_hud(
                img=annotated,
                fps=self._fps.get(),
                lane_error=lane_error,
                lane_status=lane_status,
                sign_labels=[d["label"] for d in sign_detections],
                obstacle_in_lane=obstacle_in_lane,
            )
            self._publish_image(annotated)

        # ── Logs ─────────────────────────────────────────────────────────
        if lane_status != "IN_LANE":
            self.log.info(
                f"Lane: {lane_status}  err={lane_error:+.2f}", throttle_duration_sec=1.0
            )

        if sign_detections:
            self.log.info(
                f"Signs: {[d['label'] for d in sign_detections]}",
                throttle_duration_sec=1.0,
            )

        if obstacle_in_lane:
            self.log.warning(
                f"OBSTACLE IN LANE ({len(obstacle_detections)} detected)",
                throttle_duration_sec=0.5,
            )

    # ─────────────────────────────────────────────────────────────────────
    def _publish_image(self, img: np.ndarray):
        h, w = img.shape[:2]
        msg = Image()
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img.tobytes()
        self._annotated_pub.publish(msg)


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
