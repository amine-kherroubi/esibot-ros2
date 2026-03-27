#!/usr/bin/env python3
"""EsiBot driver node.

Bridges an ESP32 motor/encoder controller to ROS 2:
- Subscribes to cmd_vel
- Publishes odom and battery_state
- Broadcasts TF (odom -> base_frame)
"""

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

DEFAULT_WHEEL_BASE = 0.16
DEFAULT_WHEEL_RADIUS = 0.033
DEFAULT_TICKS_PER_REV = 330
DEFAULT_PUBLISH_RATE = 20.0
DEFAULT_BAUD_RATE = 115200
DEFAULT_SERIAL_TIMEOUT = 0.1
DEFAULT_BATTERY_VOLTAGE = 12.0
DEFAULT_CMD_VEL_TIMEOUT = 0.5
DEFAULT_RECONNECT_INTERVAL = 2.0


class EsibotDriver(Node):
    def __init__(self) -> None:
        super().__init__("esibot_driver")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", DEFAULT_BAUD_RATE)
        self.declare_parameter("serial_timeout", DEFAULT_SERIAL_TIMEOUT)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)
        self.declare_parameter("wheel_base", DEFAULT_WHEEL_BASE)
        self.declare_parameter("wheel_radius", DEFAULT_WHEEL_RADIUS)
        self.declare_parameter("encoder_ticks_per_rev", DEFAULT_TICKS_PER_REV)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("cmd_vel_timeout", DEFAULT_CMD_VEL_TIMEOUT)
        self.declare_parameter("reconnect_on_error", True)
        self.declare_parameter("reconnect_interval", DEFAULT_RECONNECT_INTERVAL)

        self._serial_port = self.get_parameter("serial_port").value
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._serial_timeout = float(self.get_parameter("serial_timeout").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._wheel_base = float(self.get_parameter("wheel_base").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._ticks_per_rev = int(self.get_parameter("encoder_ticks_per_rev").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._battery_topic = self.get_parameter("battery_topic").value
        self._publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self._reconnect_on_error = bool(self.get_parameter("reconnect_on_error").value)
        self._reconnect_interval = float(self.get_parameter("reconnect_interval").value)

        if self._publish_rate <= 0.0:
            self.get_logger().warning(
                f"publish_rate must be > 0.0. Falling back to {DEFAULT_PUBLISH_RATE:.1f} Hz."
            )
            self._publish_rate = DEFAULT_PUBLISH_RATE

        if self._serial_timeout <= 0.0:
            self.get_logger().warning(
                f"serial_timeout must be > 0.0. Falling back to {DEFAULT_SERIAL_TIMEOUT:.2f}s."
            )
            self._serial_timeout = DEFAULT_SERIAL_TIMEOUT

        if self._ticks_per_rev <= 0:
            self.get_logger().warning(
                f"encoder_ticks_per_rev must be > 0. Falling back to {DEFAULT_TICKS_PER_REV}."
            )
            self._ticks_per_rev = DEFAULT_TICKS_PER_REV

        if self._wheel_base <= 0.0:
            self.get_logger().warning(
                f"wheel_base must be > 0.0. Falling back to {DEFAULT_WHEEL_BASE:.3f}."
            )
            self._wheel_base = DEFAULT_WHEEL_BASE

        if self._wheel_radius <= 0.0:
            self.get_logger().warning(
                f"wheel_radius must be > 0.0. Falling back to {DEFAULT_WHEEL_RADIUS:.3f}."
            )
            self._wheel_radius = DEFAULT_WHEEL_RADIUS

        if self._cmd_vel_timeout < 0.0:
            self.get_logger().warning(
                "cmd_vel_timeout must be >= 0.0. Disabling timeout."
            )
            self._cmd_vel_timeout = 0.0

        if self._reconnect_interval <= 0.0:
            self.get_logger().warning(
                f"reconnect_interval must be > 0.0. Falling back to {DEFAULT_RECONNECT_INTERVAL:.1f}s."
            )
            self._reconnect_interval = DEFAULT_RECONNECT_INTERVAL

        publish_period = 1.0 / self._publish_rate
        if self._serial_timeout > publish_period:
            self.get_logger().warning(
                "serial_timeout (%.3fs) is longer than publish period (%.3fs); "
                "this can stall the timer callback.",
                self._serial_timeout,
                publish_period,
            )

        self._meters_per_tick = (
            2.0 * math.pi * self._wheel_radius
        ) / self._ticks_per_rev

        # State
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_left_ticks = 0
        self._prev_right_ticks = 0
        self._has_encoder_baseline = False
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._cmd_timeout_active = False
        self._last_battery_voltage = DEFAULT_BATTERY_VOLTAGE
        self._last_cmd_time = self.get_clock().now()
        self._last_reconnect_time = self.get_clock().now()

        # ROS interfaces
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._battery_pub = self.create_publisher(BatteryState, self._battery_topic, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._serial_conn = None
        self._connect_serial()

        self._last_update_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._publish_rate, self._update)

        self.get_logger().info(
            "esibot_driver started (port=%s, baud=%d, rate=%.1fHz, "
            "odom_topic=%s, cmd_vel_topic=%s, frames=%s->%s)"
            % (
                self._serial_port,
                self._baud_rate,
                self._publish_rate,
                self._odom_topic,
                self._cmd_vel_topic,
                self._odom_frame,
                self._base_frame,
            )
        )

    def _connect_serial(self) -> None:
        if self._serial_conn:
            try:
                self._serial_conn.close()
            except Exception:
                pass
            self._serial_conn = None

        try:
            import serial
        except Exception as exc:
            self.get_logger().warning(
                f"pyserial not available ({exc}). Running without hardware."
            )
            self._serial_conn = None
            return

        try:
            self._serial_conn = serial.Serial(
                self._serial_port,
                self._baud_rate,
                timeout=self._serial_timeout,
            )
            self.get_logger().info(f"Serial connected on {self._serial_port}")
        except Exception as exc:
            self.get_logger().warning(
                f"Serial not available on {self._serial_port} ({exc}). Running without hardware."
            )
            self._serial_conn = None

    def _update(self) -> None:
        now = self.get_clock().now()

        if self._serial_conn is None and self._reconnect_on_error:
            since_reconnect = (now - self._last_reconnect_time).nanoseconds * 1e-9
            if since_reconnect >= self._reconnect_interval:
                self._last_reconnect_time = now
                self._connect_serial()

        if self._cmd_vel_timeout > 0.0:
            since_cmd = (now - self._last_cmd_time).nanoseconds * 1e-9
            if since_cmd > self._cmd_vel_timeout:
                if not self._cmd_timeout_active:
                    self.get_logger().warning(
                        "cmd_vel timeout (%.2fs) exceeded; stopping the robot.",
                        self._cmd_vel_timeout,
                    )
                self._cmd_timeout_active = True
                self._last_cmd_linear = 0.0
                self._last_cmd_angular = 0.0
                if self._serial_conn is not None:
                    self._send_motor_cmd(0.0, 0.0)
            else:
                self._cmd_timeout_active = False

        dt = (now - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self._publish_rate
        self._last_update_time = now

        if self._serial_conn is None:
            dist_center, delta_theta, battery_voltage = self._simulate_motion(dt)
        else:
            left_ticks, right_ticks, battery_voltage = self._read_encoders()
            if not self._has_encoder_baseline:
                self._prev_left_ticks = left_ticks
                self._prev_right_ticks = right_ticks
                self._has_encoder_baseline = True
                dist_center = 0.0
                delta_theta = 0.0
            else:
                delta_left = left_ticks - self._prev_left_ticks
                delta_right = right_ticks - self._prev_right_ticks
                self._prev_left_ticks = left_ticks
                self._prev_right_ticks = right_ticks

                dist_left = delta_left * self._meters_per_tick
                dist_right = delta_right * self._meters_per_tick
                dist_center = (dist_right + dist_left) / 2.0
                delta_theta = (dist_right - dist_left) / self._wheel_base

        self._x += dist_center * math.cos(self._theta + delta_theta / 2.0)
        self._y += dist_center * math.sin(self._theta + delta_theta / 2.0)
        self._theta += delta_theta
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        stamp = now.to_msg()
        self._publish_odometry(stamp, dist_center, delta_theta, dt)
        if self._publish_tf_enabled:
            self._publish_tf(stamp)
        self._publish_battery(stamp, battery_voltage)

    def _simulate_motion(self, dt: float) -> Tuple[float, float, float]:
        linear = self._last_cmd_linear
        angular = self._last_cmd_angular
        dist_center = linear * dt
        delta_theta = angular * dt
        return dist_center, delta_theta, DEFAULT_BATTERY_VOLTAGE

    def _read_encoders(self) -> Tuple[int, int, float]:
        if self._serial_conn is None:
            return (
                self._prev_left_ticks,
                self._prev_right_ticks,
                self._last_battery_voltage,
            )

        try:
            line = self._serial_conn.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("ENC:"):
                return (
                    self._prev_left_ticks,
                    self._prev_right_ticks,
                    self._last_battery_voltage,
                )

            parts = line[4:].split(",")
            if len(parts) < 2:
                return (
                    self._prev_left_ticks,
                    self._prev_right_ticks,
                    self._last_battery_voltage,
                )

            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
            if len(parts) > 2:
                self._last_battery_voltage = float(parts[2])
            return left_ticks, right_ticks, self._last_battery_voltage
        except Exception as exc:
            self.get_logger().warning(f"Encoder parse error: {exc}")
            return (
                self._prev_left_ticks,
                self._prev_right_ticks,
                self._last_battery_voltage,
            )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_linear = msg.linear.x
        self._last_cmd_angular = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

        if self._serial_conn is None:
            return

        v_right = msg.linear.x + (msg.angular.z * self._wheel_base / 2.0)
        v_left = msg.linear.x - (msg.angular.z * self._wheel_base / 2.0)
        self._send_motor_cmd(v_right, v_left)

    def _send_motor_cmd(self, v_right: float, v_left: float) -> None:
        if self._serial_conn is None:
            return
        try:
            self._serial_conn.write(f"CMD:{v_right:.3f},{v_left:.3f}\n".encode())
        except Exception as exc:
            self.get_logger().warning(f"Serial write error: {exc}")
            try:
                self._serial_conn.close()
            except Exception:
                pass
            self._serial_conn = None

    def _publish_odometry(
        self, stamp, dist_center: float, delta_theta: float, dt: float
    ) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(self._theta)

        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01

        if dt > 0.0:
            linear = dist_center / dt
            angular = delta_theta / dt
        else:
            linear = 0.0
            angular = 0.0

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.01
        msg.twist.covariance[35] = 0.01

        self._odom_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = _yaw_to_quaternion(self._theta)
        self._tf_broadcaster.sendTransform(transform)

    def _publish_battery(self, stamp, voltage: float) -> None:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.voltage = voltage
        msg.present = True
        self._battery_pub.publish(msg)

    def destroy_node(self) -> None:
        if self._serial_conn:
            try:
                self._serial_conn.write(b"CMD:0.000,0.000\n")
            except Exception:
                pass
            self._serial_conn.close()
        super().destroy_node()


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EsibotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
