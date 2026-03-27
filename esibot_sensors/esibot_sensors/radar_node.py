#!/usr/bin/env python3
"""
EsibotSensors — HC-SR04 sweep node
====================================
Publishes sensor_msgs/LaserScan on /scan by sweeping an SG90 servo
through ±π/2 (−90° → +90°, centred on forward) and recording one
HC-SR04 distance per step.

Convention (matches URDF servo_joint limits):
  angle = −π/2  → sensor faces right  (−Y)
  angle =  0    → sensor faces forward (+X)  ← home / centre
  angle = +π/2  → sensor faces left   (+Y)

"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState

# Try to import GPIO. If unavailable, fall back to simulation mode.
try:
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("RPi.GPIO not found — running in SIMULATION/MOCK mode.")


class EsibotSensors(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Scan geometry — matches URDF servo_joint limits (±π/2) ──────────
        #   angle_min = −π/2 → right
        #   angle_max = +π/2 → left
        #   centre    =  0   → forward
        self.angle_min       = -math.pi / 2   # −90°
        self.angle_max       =  math.pi / 2   # +90°
        self.angle_increment =  math.radians(10)  # 10° per step → 19 readings

        # ── HC-SR04 valid range (datasheet) ─────────────────────────────────
        self.range_min = 0.02   # 2 cm
        self.range_max = 4.00   # 400 cm

      # ── GPIO pin assignments (overridable from launch file / params) ─────
        self.declare_parameter('servo_pin',    17)
        self.declare_parameter('trig_pin',     27)
        self.declare_parameter('echo_pin',     22)
        self.declare_parameter('sweep_period', 3.0)
        self.declare_parameter('sim_mode',     False)

        self.servo_pin = self.get_parameter('servo_pin').value
        self.trig_pin  = self.get_parameter('trig_pin').value
        self.echo_pin  = self.get_parameter('echo_pin').value
        sweep_period   = self.get_parameter('sweep_period').value

        if HARDWARE_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)
            GPIO.setup(self.trig_pin,  GPIO.OUT)
            GPIO.setup(self.echo_pin,  GPIO.IN)
            # 50 Hz PWM — standard for SG90
            self.pwm_servo = GPIO.PWM(self.servo_pin, 50)
            self.pwm_servo.start(0)
        else:
            self.get_logger().info(
                "No hardware detected. Publishing simulated scan data."
            )

        # ── Re-entrancy guard ────────────────────────────────────────────────
        # fires, skip that tick rather than launching a second blocking sweep.
        self._scanning = False

        # ── Timer ────────────────────────────────────────────────────────────
        # Worst-case sweep: 19 steps × (0.10 s settle + 0.01 s measure)
        # self.timer = self.create_timer(sweep_period, self.timer_callback) for real hardware
        self.timer = self.create_timer(3.0, self.timer_callback)


    # ── Timer callback ───────────────────────────────────────────────────────

    def timer_callback(self):
        # skip this tick if the previous sweep hasn't finished.
        if self._scanning:
            self.get_logger().warn(
                "Previous sweep still running — skipping this tick. "
                "Consider increasing the timer period."
            )
            return
        self.publish_scan()

    # ── Main scan publisher ──────────────────────────────────────────────────

    def publish_scan(self):
        self._scanning = True
        sweep_start = time.time()

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'  
        msg.angle_min       = self.angle_min
        msg.angle_max       = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment  = 0.0            # unknown per-step timing
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max

        # ── Build ranges array ───────────────────────────────────────────────
        ranges = []
        angle  = self.angle_min

        while angle <= self.angle_max + 1e-9:   # +epsilon avoids float drift
                        
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name     = ['servo_joint']
            js.position = [angle]
            self.joint_state_pub.publish(js)

            raw = self.read_distance(angle)

            # clamp invalid readings to range_max + 1.0 per REP-117
            if raw < self.range_min or raw > self.range_max:
                dist = self.range_max + 1.0
            else:
                dist = raw

            ranges.append(dist)
            angle += self.angle_increment

        # compute actual sweep duration and set scan_time
        sweep_duration  = time.time() - sweep_start
        msg.scan_time   = float(sweep_duration)
        msg.ranges      = ranges

        msg.time_increment = float(sweep_duration) / (len(ranges) - 1)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published LaserScan: {len(ranges)} ranges, "
            f"sweep_time={sweep_duration:.3f}s, "
            f"ranges={[f'{r:.2f}' for r in ranges]}"
        )

        self._scanning = False

    # ── Hardware / simulation abstraction ───────────────────────────────────

    def read_distance(self, angle: float) -> float:
        """
        Return distance in metres at the given servo angle (radians).
        On real hardware: move servo then trigger HC-SR04.
        In simulation:   return a fake wall reading with noise.
        """
        if HARDWARE_AVAILABLE:
            self.set_servo_angle(angle)
            return self.hc_sr04_distance()
        else:
            # Simulate a flat wall ~1 m ahead with small Gaussian noise
            time.sleep(0.01)  # mimic measurement delay
            return 1.0 + random.gauss(0.0, 0.02)

    # ── Hardware helpers ─────────────────────────────────────────────────────

    def set_servo_angle(self, angle: float):
        """
        Move SG90 to the requested angle (radians, ±π/2 convention).
        """
        angle_deg = math.degrees(angle) + 90.0   # shift: −90..+90 → 0..180
        duty = 2.0 + (angle_deg / 18.0)
        self.pwm_servo.ChangeDutyCycle(duty)
        time.sleep(0.1)   # 100 ms settle (conservative for 10° steps)

    def hc_sr04_distance(self) -> float:
        """
        Trigger HC-SR04 and return distance in metres.
        Returns range_max + 1.0 if echo never arrives (timeout guard).
        """
        # Reset sensor
        GPIO.output(self.trig_pin, False)
        time.sleep(0.01)

        # 10 µs trigger pulse
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        # Wait for echo to go HIGH (rising edge) — with timeout.
        timeout = time.time() + 0.05   # 50 ms timeout
        while GPIO.input(self.echo_pin) == 0:
            if time.time() > timeout:
                self.get_logger().warn("HC-SR04: echo start timeout")
                return self.range_max + 1.0
        start = time.time()   # rising edge — captured once, after loop exits

        # Wait for echo to go LOW (falling edge) — with timeout.
        timeout = time.time() + 0.05
        while GPIO.input(self.echo_pin) == 1:
            if time.time() > timeout:
                self.get_logger().warn("HC-SR04: echo end timeout")
                return self.range_max + 1.0
        end = time.time()     # falling edge — captured once, after loop exits

        # Distance = (time × speed_of_sound) / 2
        # Speed of sound ≈ 343 m/s at 20°C
        distance = ((end - start) * 343.0) / 2.0
        return distance


def main(args=None):
    rclpy.init(args=args)
    node = EsibotSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if HARDWARE_AVAILABLE:
            node.pwm_servo.stop()
            GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()