#!/usr/bin/env python3
"""
esibot_driver.py  —  ENCODER-BASED VERSION (closed-loop odometry)
==================================================================
Wiring (EsiBot Wiring v3):
  - UART read-only (Pin 10 RXD ← ESP32 GPIO1/U0TXD) — BAT telemetry only
  - Left  wheel encoder D0 → RPi GPIO17 (Pin 11), powered from RPi 3.3V (Pin 1)
  - Right wheel encoder D0 → RPi GPIO24 (Pin 18), powered from RPi 3.3V (Pin 1)
  - L298N IN1 → GPIO5, IN2 → GPIO6, IN3 → GPIO13, IN4 → GPIO26
  - L298N ENA → GPIO18 (Pin 12) — hardware PWM, left motor speed
  - L298N ENB → GPIO19 (Pin 35) — hardware PWM, right motor speed
  - Remove 5V bridge jumpers on ENA/ENB before connecting

UART protocol:
  ESP32 → RPi :  "BAT:<voltage>\n"   (read-only, Pi sends nothing)

GPIO backend: pigpio (shares pigpiod daemon with radar_node)

Topics:
  Publishes  -> /odom            (nav_msgs/Odometry)
  Publishes  -> /tf              (geometry_msgs/TransformStamped)
  Publishes  -> /battery_state   (sensor_msgs/BatteryState)
  Subscribes -> /cmd_vel         (geometry_msgs/Twist)
"""

import math
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
import pigpio

# ─────────────────────────────────────────────────────────
#  ROBOT PHYSICAL PARAMETERS  — calibrate empirically
# ─────────────────────────────────────────────────────────

WHEEL_BASE   = 0.16    # metres  — distance between wheel centres
WHEEL_RADIUS = 0.033   # metres  — wheel radius

# 20-hole encoder disc, counting both edges (RISING + FALLING) → 40 ticks/rev.
TICKS_PER_REV = 40

# Derived: metres per tick
METRES_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

# Velocity limits — clamp incoming cmd_vel before forwarding to ESP32
MAX_LINEAR_VEL  = 0.3   # m/s
MAX_ANGULAR_VEL = 2.0   # rad/s

# RPi GPIO pin numbers (BCM numbering) for encoder digital outputs
GPIO_ENCODER_LEFT  = 24   # D0 of left  encoder → RPi Pin 18
GPIO_ENCODER_RIGHT = 17   # D0 of right encoder → RPi Pin 11

# L298N motor driver — direction control (BCM numbering)
GPIO_IN1 = 13   # Pin 33 — left  motor forward
GPIO_IN2 = 26   # Pin 37 — left  motor backward
GPIO_IN3 = 5    # Pin 29 — right motor forward
GPIO_IN4 = 6    # Pin 31 — right motor backward

# L298N speed control via PWM on ENA/ENB
GPIO_ENA = 19   # Pin 35 — left  motor speed (hardware PWM1)
GPIO_ENB = 18   # Pin 12 — right motor speed (hardware PWM0)
PWM_FREQ     = 1000  # Hz — motor PWM frequency
MAX_PWM_DUTY =   55  # % — cap duty cycle to limit top speed (tune empirically)

# Velocity threshold below which the motor is stopped (m/s)
MOTOR_DEADBAND = 0.05


class EsibotDriver(Node):

    def __init__(self):
        super().__init__('esibot_driver')

        # ── ROS2 parameters ───────────────────────────────────────────────────
        self.declare_parameter('serial_port',    '/dev/ttyS0')
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

        # ── Encoder tick counters (written by GPIO callback, read by timer) ───
        self._lock         = threading.Lock()
        self._ticks_left   = 0   # cumulative, incremented by callback
        self._ticks_right  = 0
        self._prev_left    = 0   # snapshot from previous update cycle
        self._prev_right   = 0

        # ── Velocity estimate (for /odom twist field) ─────────────────────────
        self._vel_linear  = 0.0
        self._vel_angular = 0.0

        # ── pigpio connection + encoder callback handles ─────────────────────
        self._pi = None
        self._cb_left = None
        self._cb_right = None

        # ── Motor direction (+1 forward, -1 backward, 0 stopped) ─────────────
        # Used to sign encoder tick deltas — IR encoders count both directions
        self._dir_left  = 0
        self._dir_right = 0

        # ── Encoder debug counter (log ticks every 2 s) ───────────────────────
        self._debug_tick = 0

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

        # ── GPIO encoder interrupts + motor outputs ───────────────────────────
        self._gpio_available = False
        self._motor_gpio_available = False
        if not self.sim_mode:
            self._setup_gpio()
            self._setup_motor_gpio()

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
    #  GPIO SETUP (pigpio — shares pigpiod with radar_node)
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_gpio(self):
        """Connect to pigpiod and register encoder callbacks."""
        try:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                raise RuntimeError('pigpiod not running')

            self._pi.set_mode(GPIO_ENCODER_LEFT, pigpio.INPUT)
            self._pi.set_mode(GPIO_ENCODER_RIGHT, pigpio.INPUT)
            self._pi.set_pull_up_down(GPIO_ENCODER_LEFT, pigpio.PUD_UP)
            self._pi.set_pull_up_down(GPIO_ENCODER_RIGHT, pigpio.PUD_UP)

            self._cb_left = self._pi.callback(
                GPIO_ENCODER_LEFT, pigpio.EITHER_EDGE, self._left_encoder_cb
            )
            self._cb_right = self._pi.callback(
                GPIO_ENCODER_RIGHT, pigpio.EITHER_EDGE, self._right_encoder_cb
            )

            self._gpio_available = True
            self.get_logger().info(
                f'Encoder callbacks registered (pigpio) on GPIO{GPIO_ENCODER_LEFT} (left) '
                f'and GPIO{GPIO_ENCODER_RIGHT} (right)'
            )
        except Exception as e:
            self._gpio_available = False
            self.get_logger().warn(
                f'pigpio not available ({e}). '
                'Falling back to open-loop odometry from cmd_vel.'
            )

    def _setup_motor_gpio(self):
        """Configure L298N IN1-IN4 (direction) and ENA/ENB (PWM speed) via pigpio."""
        try:
            pi = self._pi
            if pi is None or not pi.connected:
                raise RuntimeError('pigpio not connected')

            for pin in (GPIO_IN1, GPIO_IN2, GPIO_IN3, GPIO_IN4):
                pi.set_mode(pin, pigpio.OUTPUT)
                pi.write(pin, 0)

            pi.set_PWM_frequency(GPIO_ENA, PWM_FREQ)
            pi.set_PWM_frequency(GPIO_ENB, PWM_FREQ)
            pi.set_PWM_dutycycle(GPIO_ENA, 0)
            pi.set_PWM_dutycycle(GPIO_ENB, 0)

            self._motor_gpio_available = True
            self.get_logger().info(
                f'Motor GPIO configured (pigpio): IN1=GPIO{GPIO_IN1}, IN2=GPIO{GPIO_IN2}, '
                f'IN3=GPIO{GPIO_IN3}, IN4=GPIO{GPIO_IN4}, '
                f'ENA=GPIO{GPIO_ENA} (PWM {PWM_FREQ}Hz), ENB=GPIO{GPIO_ENB} (PWM {PWM_FREQ}Hz)'
            )
        except Exception as e:
            self._motor_gpio_available = False
            self.get_logger().warn(f'Motor GPIO setup failed ({e}) — motors disabled.')

    def _set_motor(self, v_left: float, v_right: float):
        """PWM motor control via pigpio: direction via IN1-IN4, speed via ENA/ENB."""
        if not self._motor_gpio_available:
            return
        pi = self._pi

        # Convert % duty (0-100) to pigpio range (0-255)
        duty_left  = int(min(abs(v_left)  / MAX_LINEAR_VEL * MAX_PWM_DUTY, MAX_PWM_DUTY) * 255 / 100)
        duty_right = int(min(abs(v_right) / MAX_LINEAR_VEL * MAX_PWM_DUTY, MAX_PWM_DUTY) * 255 / 100)

        # Left motor direction + sign tracking for encoder integration
        if v_left > MOTOR_DEADBAND:
            pi.write(GPIO_IN1, 1)
            pi.write(GPIO_IN2, 0)
            self._dir_left = +1
        elif v_left < -MOTOR_DEADBAND:
            pi.write(GPIO_IN1, 0)
            pi.write(GPIO_IN2, 1)
            self._dir_left = -1
        else:
            pi.write(GPIO_IN1, 0)
            pi.write(GPIO_IN2, 0)
            duty_left = 0
            self._dir_left = 0

        # Right motor direction + sign tracking for encoder integration
        if v_right > MOTOR_DEADBAND:
            pi.write(GPIO_IN3, 1)
            pi.write(GPIO_IN4, 0)
            self._dir_right = +1
        elif v_right < -MOTOR_DEADBAND:
            pi.write(GPIO_IN3, 0)
            pi.write(GPIO_IN4, 1)
            self._dir_right = -1
        else:
            pi.write(GPIO_IN3, 0)
            pi.write(GPIO_IN4, 0)
            duty_right = 0
            self._dir_right = 0

        # Speed via PWM duty cycle (pigpio range 0-255)
        pi.set_PWM_dutycycle(GPIO_ENA, duty_left)
        pi.set_PWM_dutycycle(GPIO_ENB, duty_right)

    # ── Encoder callbacks (run in pigpio thread) ──────────────────────────

    def _left_encoder_cb(self, gpio, level, tick):
        with self._lock:
            self._ticks_left += 1

    def _right_encoder_cb(self, gpio, level, tick):
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

        # Apply direction sign — IR encoders count pulses regardless of direction
        delta_left  = (curr_left  - self._prev_left)  * METRES_PER_TICK * self._dir_left
        delta_right = (curr_right - self._prev_right) * METRES_PER_TICK * self._dir_right

        self._prev_left  = curr_left
        self._prev_right = curr_right

        # Log encoder ticks every 2 s to confirm hardware is counting
        self._debug_tick += 1
        if self._debug_tick >= 40:
            self._debug_tick = 0
            self.get_logger().info(
                f'Encoders — L:{curr_left} ticks (dir={self._dir_left:+d})  '
                f'R:{curr_right} ticks (dir={self._dir_right:+d})  '
                f'| pose x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f}°'
            )

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
        """Clamp velocity, store for watchdog/open-loop fallback, drive L298N via GPIO."""
        linear  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  msg.linear.x))
        angular = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, msg.angular.z))

        self._vel_linear  = linear
        self._vel_angular = angular
        self._last_cmd_time = self.get_clock().now()

        if linear == 0.0 and angular != 0.0:
            v_right = max(0.0,  angular) * WHEEL_BASE
            v_left  = max(0.0, -angular) * WHEEL_BASE
        else:
            v_right = linear + (angular * WHEEL_BASE / 2.0)
            v_left  = linear - (angular * WHEEL_BASE / 2.0)

        self._set_motor(v_left, v_right)

    def _send_stop(self):
        """Stop motors if not already stopped."""
        if self._vel_linear == 0.0 and self._vel_angular == 0.0:
            return
        self._vel_linear  = 0.0
        self._vel_angular = 0.0
        self._set_motor(0.0, 0.0)

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
        if self._cb_left is not None:
            self._cb_left.cancel()
        if self._cb_right is not None:
            self._cb_right.cancel()
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        if self._pi is not None and self._pi.connected:
            try:
                self._pi.set_PWM_dutycycle(GPIO_ENA, 0)
                self._pi.set_PWM_dutycycle(GPIO_ENB, 0)
                for pin in (GPIO_IN1, GPIO_IN2, GPIO_IN3, GPIO_IN4):
                    self._pi.write(pin, 0)
            except Exception:
                pass
            self._pi.stop()
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
