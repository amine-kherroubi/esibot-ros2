#!/usr/bin/env python3
"""
esibot_driver.py  —  OPEN-LOOP VERSION (no encoders)
=====================================================
Adapted for the actual EsiBot wiring:
  - No encoders → odometry estimated from commands sent (open-loop)
  - ENA/ENB hardwired to 5V → direction control via IN1-IN4 only
  - UART via GPIO pins (Pi Pin8/Pin10) → /dev/ttyAMA0

How open-loop odometry works:
  The Pi knows exactly what CMD it sent and at what time.
  linear_velocity  ≈ sign(v) × LINEAR_SPEED_MPS   (when moving)
  angular_velocity ≈ sign(ω) × ANGULAR_SPEED_RPS  (when turning)
  Calibrate these two constants once with a tape measure.

UART protocol:
  RPi → ESP32 :  "CMD:<v_right>,<v_left>\\n"   e.g. "CMD:0.300,-0.280\\n"
  ESP32 → RPi :  "BAT:<voltage>\\n"             e.g. "BAT:11.2\\n"

Topics:
  Publishes  -> /odom            (nav_msgs/Odometry)
  Publishes  -> /tf              (geometry_msgs/TransformStamped)
  Publishes  -> /battery_state   (sensor_msgs/BatteryState)
  Subscribes -> /cmd_vel         (geometry_msgs/Twist)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster


# ─────────────────────────────────────────────────────────
#  ROBOT PHYSICAL PARAMETERS  — calibrate empirically
# ─────────────────────────────────────────────────────────

WHEEL_BASE = 0.16       # TODO(calibrate): distance between wheels (meters)
                        # Measure with a ruler on the assembled robot

# Effective real-world speed when a full-magnitude command is sent.
# How to calibrate:
#   LINEAR  → send CMD forward for 2s, measure distance → value = dist / 2.0
#   ANGULAR → send CMD spin for 2s, measure angle (rad) → value = angle / 2.0
LINEAR_SPEED_MPS  = 0.20   # TODO(calibrate): m/s at full forward command
ANGULAR_SPEED_RPS = 1.0    # TODO(calibrate): rad/s at full rotation command

# Dead zone: commands smaller than this are treated as zero
DEAD_ZONE = 0.01


class EsibotDriver(Node):

    def __init__(self):
        super().__init__('esibot_driver')

        # ── ROS2 parameters ───────────────────────────────────────────────────
        # serial_port changed to /dev/ttyAMA0 (Pi GPIO UART, Pin8/Pin10)
        self.declare_parameter('serial_port',  '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('base_frame',   'base_footprint')
        self.declare_parameter('publish_rate', 20.0)

        self.serial_port  = self.get_parameter('serial_port').value
        self.baud_rate    = self.get_parameter('baud_rate').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ── Robot pose state ──────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Last received /cmd_vel (for open-loop integration) ────────────────
        self._cmd_linear  = 0.0
        self._cmd_angular = 0.0
        self._last_update_time = self.get_clock().now()

        # ── Battery voltage (updated from ESP32 BAT: messages) ────────────────
        self._battery_voltage = 11.1

        # ── ROS2 Publishers ───────────────────────────────────────────────────
        self.odom_pub       = self.create_publisher(Odometry,     '/odom',          10)
        self.battery_pub    = self.create_publisher(BatteryState, '/battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── ROS2 Subscriber ───────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # ── Serial connection ─────────────────────────────────────────────────
        self.serial_conn = None
        self._connect_serial()

        # ── Periodic update timer ─────────────────────────────────────────────
        self.create_timer(1.0 / self.publish_rate, self._update)

        self.get_logger().info(
            f'esibot_driver (open-loop) | port={self.serial_port} | '
            f'{self.publish_rate}Hz | WHEEL_BASE={WHEEL_BASE}m | '
            f'LINEAR_SPEED={LINEAR_SPEED_MPS}m/s | ANGULAR_SPEED={ANGULAR_SPEED_RPS}rad/s'
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  SERIAL CONNECTION
    # ══════════════════════════════════════════════════════════════════════════

    def _connect_serial(self):
        """
        Opens UART on /dev/ttyAMA0 (Pi GPIO14=TX Pin8, GPIO15=RX Pin10).

        IMPORTANT — before first use, disable the Pi serial console:
            sudo raspi-config
            → Interface Options → Serial Port
            → "login shell over serial?" → No
            → "serial port hardware enabled?" → Yes
            → Reboot
        """
        try:
            import serial
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.05   # short so the 20Hz loop is never blocked
            )
            self.get_logger().info(f'Serial connected on {self.serial_port}')
        except Exception as e:
            self.serial_conn = None
            self.get_logger().warn(
                f'Serial not available ({e}) — running in simulation mode.'
            )

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN UPDATE LOOP  (20 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def _update(self):
        """
        1. Read any incoming line from ESP32 (battery voltage).
        2. Integrate the last known command into the pose (open-loop odometry).
        3. Publish /odom, /tf, /battery_state.
        """
        self._read_serial()

        # ── Compute dt ────────────────────────────────────────────────────────
        now = self.get_clock().now()
        dt  = (now - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = now
        dt  = min(dt, 0.5)   # clamp: avoid huge jumps if node was paused

        # ── Map command to real-world speed using calibrated constants ─────────
        v = math.copysign(LINEAR_SPEED_MPS,  self._cmd_linear)  if abs(self._cmd_linear)  > DEAD_ZONE else 0.0
        w = math.copysign(ANGULAR_SPEED_RPS, self._cmd_angular) if abs(self._cmd_angular) > DEAD_ZONE else 0.0

        # ── Integrate pose ────────────────────────────────────────────────────
        dist_center = v * dt
        delta_theta = w * dt

        self.x     += dist_center * math.cos(self.theta + delta_theta / 2.0)
        self.y     += dist_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = now.to_msg()
        self._publish_odometry(stamp, v, w)
        self._publish_tf(stamp)
        self._publish_battery(self._battery_voltage)

    # ══════════════════════════════════════════════════════════════════════════
    #  READ INCOMING SERIAL FROM ESP32
    # ══════════════════════════════════════════════════════════════════════════

    def _read_serial(self):
        """
        Non-blocking read of one line from ESP32.
        Parses:  "BAT:11.2\\n"  → updates battery voltage
        Ignores: "ENC:..." (no encoders in this build)
        Logs:    everything else as debug (ESP32 Serial.print messages)
        """
        if self.serial_conn is None:
            return
        try:
            line = self.serial_conn.readline().decode('utf-8').strip()
            if not line:
                return
            if line.startswith('BAT:'):
                self._battery_voltage = float(line[4:])
            elif line.startswith('ENC:'):
                pass   # ignore, no encoders
            else:
                self.get_logger().debug(f'ESP32: {line}')
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    # ══════════════════════════════════════════════════════════════════════════
    #  /cmd_vel CALLBACK  →  send CMD to ESP32
    # ══════════════════════════════════════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist):
        """
        Stores the command for open-loop odometry integration.
        Sends "CMD:<v_right>,<v_left>\\n" to ESP32 immediately.

        The ESP32 interprets:
          positive value → forward direction for that wheel
          negative value → reverse direction
          magnitude      → speed level (ESP32 maps to PWM duty or on/off)

        Since ENA/ENB are hardwired to 5V, the ESP32 controls speed by
        toggling IN1/IN2 (left) and IN3/IN4 (right):
          Forward:  IN1=HIGH IN2=LOW  (or IN3=HIGH IN4=LOW)
          Backward: IN1=LOW  IN2=HIGH (or IN3=LOW  IN4=HIGH)
          Stop:     IN1=LOW  IN2=LOW
        """
        self._cmd_linear  = msg.linear.x
        self._cmd_angular = msg.angular.z

        v_right = msg.linear.x + (msg.angular.z * WHEEL_BASE / 2.0)
        v_left  = msg.linear.x - (msg.angular.z * WHEEL_BASE / 2.0)

        if self.serial_conn is not None:
            cmd = f'CMD:{v_right:.3f},{v_left:.3f}\n'
            try:
                self.serial_conn.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial write error: {e}')

    # ══════════════════════════════════════════════════════════════════════════
    #  PUBLISHERS
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_odometry(self, stamp, linear_vel, angular_vel):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id  = self.base_frame

        msg.pose.pose.position.x  = self.x
        msg.pose.pose.position.y  = self.y
        msg.pose.pose.position.z  = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(self.theta)

        msg.twist.twist.linear.x  = linear_vel
        msg.twist.twist.angular.z = angular_vel

        # Higher covariance → slam_toolbox trusts scan more than odometry
        # This is correct for open-loop (drifts more than encoder-based)
        msg.pose.covariance[0]  = 0.1    # x
        msg.pose.covariance[7]  = 0.1    # y
        msg.pose.covariance[35] = 0.05   # yaw

        self.odom_pub.publish(msg)

    def _publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = self.odom_frame
        t.child_frame_id          = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = _yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def _publish_battery(self, voltage: float):
        msg = BatteryState()
        msg.voltage = voltage
        msg.present = True
        self.battery_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════════════════════════════════════

    def destroy_node(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.write(b'CMD:0.000,0.000\n')  # safety stop
            except Exception:
                pass
            self.serial_conn.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────
#  HELPER
# ─────────────────────────────────────────────────────────

def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ─────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EsibotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()