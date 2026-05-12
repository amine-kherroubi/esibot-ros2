#!/usr/bin/env python3
"""
esibot_driver.py  —  ENCODER-BASED VERSION (closed-loop odometry)
==================================================================
Wiring (from EsiBot Wiring Guide v1):
  - UART via RPi GPIO UART (Pin 8 TXD → ESP32 GPIO3/U0RXD,
                             Pin 10 RXD ← ESP32 GPIO1/U0TXD)
    → serial_port: /dev/ttyAMA0
  - Left  wheel encoder D0 → RPi GPIO17 (Pin 11), powered from RPi 3.3V (Pin 1)
  - Right wheel encoder D0 → RPi GPIO18 (Pin 12), powered from RPi 3.3V (Pin 1)
  - Encoders are 20-hole discs → 20 ticks per revolution (rising edges only)
  - ENA/ENB hardwired to 5V → direction/stop controlled via IN1-IN4 only

Odometry:
  Encoder ticks are counted in a background thread using RPi.GPIO interrupts.
  Each rising edge on D0 = 1 tick. The driver integrates left/right tick deltas
  into a differential-drive pose estimate (Runge-Kutta 2nd order).

  Ticks per revolution = 20  (20-hole encoder disc, rising edges only)
  Wheel radius  → measure your robot, update WHEEL_RADIUS below
  Wheel base    → measure your robot, update WHEEL_BASE below

UART protocol:
  RPi → ESP32 :  "CMD:<v_right>,<v_left>\n"   e.g. "CMD:0.300,-0.280\n"
  ESP32 → RPi :  "BAT:<voltage>\n"             e.g. "BAT:11.2\n"

Topics:
  Publishes  -> /odom            (nav_msgs/Odometry)
  Publishes  -> /tf              (geometry_msgs/TransformStamped)
  Publishes  -> /battery_state   (sensor_msgs/BatteryState)
  Subscribes -> /cmd_vel         (geometry_msgs/Twist)

Before first use:
  1. Disable Pi serial console:
       sudo raspi-config → Interface Options → Serial Port
       "login shell over serial?" → No
       "serial port hardware enabled?" → Yes  → Reboot
  2. Install RPi.GPIO if not present:
       pip3 install RPi.GPIO
"""

import math
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

# ─────────────────────────────────────────────────────────
#  ROBOT PHYSICAL PARAMETERS  — calibrate empirically
# ─────────────────────────────────────────────────────────

# TODO(calibrate): Measure the distance between the two wheel contact centres
# with a ruler on the assembled robot and set this value.
WHEEL_BASE   = 0.16    # metres  — distance between wheel centres

# TODO(calibrate): Measure the actual outer diameter of the wheel (including
# tyre if present) and divide by 2.
WHEEL_RADIUS = 0.033   # metres  — wheel radius

# 20-hole encoder disc, counting rising edges only → 20 ticks per full revolution.
# If you later use both edges (BOTH in GPIO.add_event_detect) set this to 40.
TICKS_PER_REV = 20

# Derived: metres per tick
METRES_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

# Velocity limits — clamp incoming cmd_vel before forwarding to ESP32
MAX_LINEAR_VEL  = 0.3   # m/s
MAX_ANGULAR_VEL = 2.0   # rad/s

# RPi GPIO pin numbers (BCM numbering) for encoder digital outputs
GPIO_ENCODER_LEFT  = 17   # D0 of left  encoder → RPi Pin 11
GPIO_ENCODER_RIGHT = 18   # D0 of right encoder → RPi Pin 12


class EsibotDriver(Node):

    def __init__(self):
        super().__init__('esibot_driver')

        # ── ROS2 parameters ───────────────────────────────────────────────────
        self.declare_parameter('serial_port',    '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',      115200)
        self.declare_parameter('odom_frame',     'odom')
        self.declare_parameter('base_frame',     'base_footprint')
        self.declare_parameter('publish_rate',   20.0)
        self.declare_parameter('publish_tf',     True)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('sim_mode',       False)

        self.serial_port      = self.get_parameter('serial_port').value
        self.baud_rate        = self.get_parameter('baud_rate').value
        self.odom_frame       = self.get_parameter('odom_frame').value
        self.base_frame       = self.get_parameter('base_frame').value
        self.publish_rate     = self.get_parameter('publish_rate').value
        self.publish_tf       = self.get_parameter('publish_tf').value
        self.cmd_vel_timeout  = self.get_parameter('cmd_vel_timeout').value
        self.sim_mode         = self.get_parameter('sim_mode').value

        # ── Robot pose state ──────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Encoder tick counters (written by GPIO interrupt, read by timer) ──
        self._lock         = threading.Lock()
        self._ticks_left   = 0   # cumulative, incremented by interrupt
        self._ticks_right  = 0
        self._prev_left    = 0   # snapshot from previous update cycle
        self._prev_right   = 0

        # ── Velocity estimate (for /odom twist field) ─────────────────────────
        self._vel_linear  = 0.0
        self._vel_angular = 0.0

        # ── cmd_vel watchdog ──────────────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()

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
        if not self.sim_mode:
            self._connect_serial()

        # ── GPIO encoder interrupts ───────────────────────────────────────────
        self._gpio_available = False
        if not self.sim_mode:
            self._setup_gpio()

        # ── Periodic update timer ─────────────────────────────────────────────
        self._last_update_time = self.get_clock().now()
        self.create_timer(1.0 / self.publish_rate, self._update)

        self.get_logger().info(
            f'esibot_driver (encoder) | port={self.serial_port} | '
            f'{self.publish_rate} Hz | WHEEL_BASE={WHEEL_BASE} m | '
            f'WHEEL_RADIUS={WHEEL_RADIUS} m | TICKS_PER_REV={TICKS_PER_REV} | '
            f'sim_mode={self.sim_mode}'
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  GPIO SETUP
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_gpio(self):
        """
        Configure RPi GPIO interrupts for both wheel encoders.
        Encoders are powered at 3.3V so their D0 output is RPi-safe.
        Rising-edge detection counts one tick per hole in the disc.
        """
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(GPIO_ENCODER_LEFT,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(GPIO_ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            GPIO.add_event_detect(
                GPIO_ENCODER_LEFT,
                GPIO.RISING,
                callback=self._left_encoder_cb,
            )
            GPIO.add_event_detect(
                GPIO_ENCODER_RIGHT,
                GPIO.RISING,
                callback=self._right_encoder_cb,
            )

            self._gpio_available = True
            self._GPIO = GPIO
            self.get_logger().info(
                f'Encoder interrupts registered on GPIO{GPIO_ENCODER_LEFT} (left) '
                f'and GPIO{GPIO_ENCODER_RIGHT} (right)'
            )
        except Exception as e:
            self._gpio_available = False
            self.get_logger().warn(
                f'RPi.GPIO not available ({e}). '
                'Falling back to open-loop odometry from cmd_vel.'
            )

    # ── Interrupt callbacks (run in GPIO thread) ───────────────────────────

    def _left_encoder_cb(self, channel):
        with self._lock:
            self._ticks_left += 1

    def _right_encoder_cb(self, channel):
        with self._lock:
            self._ticks_right += 1

    # ══════════════════════════════════════════════════════════════════════════
    #  SERIAL CONNECTION
    # ══════════════════════════════════════════════════════════════════════════

    def _connect_serial(self):
        try:
            import serial
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.03,   # must be < 1/publish_rate = 0.05 s
            )
            self.get_logger().info(f'Serial connected on {self.serial_port}')
        except Exception as e:
            self.serial_conn = None
            self.get_logger().warn(
                f'Serial not available ({e}) — running without motor commands.'
            )

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN UPDATE LOOP  (20 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def _update(self):
        """
        1. Read any incoming line from ESP32 (battery voltage).
        2. Apply cmd_vel watchdog (stop if no command received recently).
        3. Integrate encoder ticks into pose (closed-loop) or fall back to
           open-loop cmd_vel integration if GPIO is unavailable.
        4. Publish /odom, /tf, /battery_state.
        """
        self._read_serial()

        # ── cmd_vel watchdog ──────────────────────────────────────────────────
        if self.cmd_vel_timeout > 0.0:
            elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if elapsed > self.cmd_vel_timeout:
                self._send_stop()

        # ── dt ────────────────────────────────────────────────────────────────
        now = self.get_clock().now()
        dt  = (now - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = now
        dt  = min(dt, 0.5)   # clamp to avoid huge jumps after pause

        # ── Odometry integration ──────────────────────────────────────────────
        if self._gpio_available and not self.sim_mode:
            self._integrate_encoders(dt)
        else:
            # Fallback: open-loop from last received cmd_vel
            self._integrate_open_loop(dt)

        # ── Publish ───────────────────────────────────────────────────────────
        stamp = now.to_msg()
        self._publish_odometry(stamp)
        if self.publish_tf:
            self._publish_tf(stamp)
        self._publish_battery(self._battery_voltage)

    # ── Closed-loop odometry from encoder ticks ────────────────────────────

    def _integrate_encoders(self, dt: float):
        """Differential-drive kinematics from encoder tick deltas."""
        with self._lock:
            curr_left  = self._ticks_left
            curr_right = self._ticks_right

        delta_left  = (curr_left  - self._prev_left)  * METRES_PER_TICK
        delta_right = (curr_right - self._prev_right) * METRES_PER_TICK

        self._prev_left  = curr_left
        self._prev_right = curr_right

        delta_center = (delta_right + delta_left)  / 2.0
        delta_theta  = (delta_right - delta_left)  / WHEEL_BASE

        # Runge-Kutta 2nd order integration
        self.x     += delta_center * math.cos(self.theta + delta_theta / 2.0)
        self.y     += delta_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocity estimate for /odom twist field
        if dt > 1e-6:
            self._vel_linear  = delta_center / dt
            self._vel_angular = delta_theta  / dt

    # ── Open-loop fallback (no GPIO) ───────────────────────────────────────

    def _integrate_open_loop(self, dt: float):
        """Integrate last cmd_vel directly into pose (no encoders)."""
        v = self._vel_linear
        w = self._vel_angular
        delta_center = v * dt
        delta_theta  = w * dt
        self.x     += delta_center * math.cos(self.theta + delta_theta / 2.0)
        self.y     += delta_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # ══════════════════════════════════════════════════════════════════════════
    #  READ INCOMING SERIAL FROM ESP32
    # ══════════════════════════════════════════════════════════════════════════

    def _read_serial(self):
        """Non-blocking read: parses BAT:<v> messages, ignores everything else."""
        if self.serial_conn is None:
            return
        try:
            line = self.serial_conn.readline().decode('utf-8').strip()
            if not line:
                return
            if line.startswith('BAT:'):
                self._battery_voltage = float(line[4:])
            else:
                self.get_logger().debug(f'ESP32: {line}')
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    # ══════════════════════════════════════════════════════════════════════════
    #  /cmd_vel CALLBACK  →  send CMD to ESP32
    # ══════════════════════════════════════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist):
        """
        Clamp velocity, store for watchdog/open-loop fallback, send to ESP32.

        The ESP32 interprets CMD values as signed wheel velocities:
          positive → forward, negative → reverse, magnitude drives PWM/on-off.
        ENA/ENB are hardwired to 5V so direction is set via IN1-IN4 only.
        """
        linear  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  msg.linear.x))
        angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, msg.angular.z))

        self._vel_linear  = linear
        self._vel_angular = angular
        self._last_cmd_time = self.get_clock().now()

        v_right = linear + (angular * WHEEL_BASE / 2.0)
        v_left  = linear - (angular * WHEEL_BASE / 2.0)

        if self.serial_conn is not None:
            cmd = f'CMD:{v_right:.3f},{v_left:.3f}\n'
            try:
                self.serial_conn.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Serial write error: {e}')

    def _send_stop(self):
        """Send a zero-velocity command if we haven't already."""
        if self._vel_linear == 0.0 and self._vel_angular == 0.0:
            return
        self._vel_linear  = 0.0
        self._vel_angular = 0.0
        if self.serial_conn is not None:
            try:
                self.serial_conn.write(b'CMD:0.000,0.000\n')
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════════════
    #  PUBLISHERS
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id  = self.base_frame

        msg.pose.pose.position.x  = self.x
        msg.pose.pose.position.y  = self.y
        msg.pose.pose.position.z  = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(self.theta)

        msg.twist.twist.linear.x  = self._vel_linear
        msg.twist.twist.angular.z = self._vel_angular

        # Covariance — encoder-based is more reliable than open-loop,
        # but still a basic differential drive without IMU correction.
        msg.pose.covariance[0]  = 0.05   # x
        msg.pose.covariance[7]  = 0.05   # y
        msg.pose.covariance[35] = 0.02   # yaw

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
        self._send_stop()
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        if self._gpio_available:
            try:
                self._GPIO.cleanup()
            except Exception:
                pass
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