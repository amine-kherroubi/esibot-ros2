#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import cv2
import numpy as np
import urllib.request
import threading
import time


class CameraStreamNode(Node):

    def __init__(self):
        super().__init__('esibot_camera_node')

        # Paramètres
        self.declare_parameter('esp32_ip',        '192.168.1.80')
        self.declare_parameter('esp32_port',       80)
        self.declare_parameter('stream_path',      '/stream')
        self.declare_parameter('frame_width',      320)
        self.declare_parameter('frame_height',     240)
        self.declare_parameter('publish_rate',     10.0)
        self.declare_parameter('camera_frame',     'camera_optical_frame')
        self.declare_parameter('show_fps',         True)
        self.declare_parameter('reconnect_delay',  3.0)

        self.esp32_ip        = self.get_parameter('esp32_ip').value
        self.esp32_port      = self.get_parameter('esp32_port').value
        self.stream_path     = self.get_parameter('stream_path').value
        self.frame_w         = self.get_parameter('frame_width').value
        self.frame_h         = self.get_parameter('frame_height').value
        self.publish_rate    = self.get_parameter('publish_rate').value
        self.camera_frame    = self.get_parameter('camera_frame').value
        self.show_fps        = self.get_parameter('show_fps').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value

        self.stream_url = f'http://{self.esp32_ip}:{self.esp32_port}{self.stream_path}'

        self.latest_frame  = None
        self.frame_lock    = threading.Lock()
        self.frame_count   = 0
        self.fps           = 0.0
        self.fps_timer     = time.time()
        self.running       = True

        self.image_pub  = self.create_publisher(Image,      '/camera/image_raw',   10)
        self.info_pub   = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.status_pub = self.create_publisher(String,     '/camera/status',      10)

        self.pub_timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(f'esibot_camera ESP32 — URL: {self.stream_url}')

    def capture_loop(self):
        while self.running:
            try:
                self.get_logger().info(f'Connexion ESP32-CAM: {self.stream_url}')
                stream = urllib.request.urlopen(self.stream_url, timeout=10)
                self.get_logger().info('ESP32-CAM connecte !')
                self._publish_status('CONNECTED')

                buf = bytes()
                while self.running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk

                    # Chercher frame JPEG dans flux MJPEG
                    start = buf.find(b'\xff\xd8')  # debut JPEG
                    end   = buf.find(b'\xff\xd9')  # fin JPEG

                    if start != -1 and end != -1 and end > start:
                        jpg   = buf[start:end + 2]
                        buf   = buf[end + 2:]

                        arr   = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            if self.frame_count == 0:
                                self.get_logger().info(
                                    f'Premiere frame ESP32: {frame.shape} mean={frame.mean():.1f}')

                            processed = self.process_frame(frame)
                            with self.frame_lock:
                                self.latest_frame = processed
                            self.frame_count += 1

            except urllib.error.URLError as e:
                self.get_logger().warn(f'ESP32 injoignable: {e.reason}')
                self._publish_status(f'DISCONNECTED: {e.reason}')
                self.get_logger().info(f'Reconnexion dans {self.reconnect_delay}s...')
                time.sleep(self.reconnect_delay)

            except Exception as e:
                self.get_logger().error(f'Erreur: {e}')
                self._publish_status(f'ERROR: {e}')
                time.sleep(self.reconnect_delay)

    def process_frame(self, frame):
        # Redimensionner
        frame = cv2.resize(frame, (self.frame_w, self.frame_h))

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
       #         (self.frame_w - 80, 22),
       #         cv2.FONT_HERSHEY_SIMPLEX,
       #         0.5, (200, 200, 200), 1, cv2.LINE_AA)

        return frame

    def _update_fps(self):
        now = time.time()
        elapsed = now - self.fps_timer
        if elapsed >= 1.0:
            self.fps         = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_timer   = now

    def publish_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()

        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.camera_frame
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = 'bgr8'
        msg.is_bigendian    = False
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        self.image_pub.publish(msg)

        self.publish_camera_info(stamp)

    def publish_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp    = stamp
        info.header.frame_id = self.camera_frame
        info.width           = self.frame_w
        info.height          = self.frame_h
        fx = 160.0
        fy = 160.0
        cx = self.frame_w / 2.0
        cy = self.frame_h / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False
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


if __name__ == '__main__':
    main()
