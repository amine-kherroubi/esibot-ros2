#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

WHEEL_BASE            = 0.16
WHEEL_RADIUS          = 0.033
ENCODER_TICKS_PER_REV = 330
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / ENCODER_TICKS_PER_REV

class EsibotDriver(Node):
    def __init__(self):
        super().__init__('esibot_driver')
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.prev_left_ticks  = 0
        self.prev_right_ticks = 0
        self.sim_ticks = 0

        self.odom_pub    = self.create_publisher(Odometry,     '/odom',          10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.serial_conn = None
        self._connect_serial()
        self.create_timer(0.05, self._update)
        self.get_logger().info('esibot_driver started | 20.0Hz')

    def _connect_serial(self):
        try:
            import serial
            self.serial_conn = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info('Serial connected')
        except Exception as e:
            self.serial_conn = None
            self.get_logger().warn(f'Serial not available ({e}) — running in simulation mode.')

    def _update(self):
        left_ticks, right_ticks, battery_voltage = self._read_encoders()
        delta_left  = left_ticks  - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        self.prev_left_ticks  = left_ticks
        self.prev_right_ticks = right_ticks

        dist_left  = delta_left  * METERS_PER_TICK
        dist_right = delta_right * METERS_PER_TICK
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / WHEEL_BASE

        self.x     += dist_center * math.cos(self.theta + delta_theta / 2.0)
        self.y     += dist_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        now = self.get_clock().now().to_msg()
        self._publish_odometry(now, dist_center, delta_theta)
        self._publish_tf(now)
        self._publish_battery(battery_voltage)

    def _read_encoders(self):
        if self.serial_conn is None:
            # simulation: slowly increment ticks so odom moves
            self.sim_ticks += 1
            return self.sim_ticks, self.sim_ticks, 12.0
        try:
            line = self.serial_conn.readline().decode('utf-8').strip()
            if not line.startswith('ENC:'):
                return self.prev_left_ticks, self.prev_right_ticks, 12.0
            parts = line[4:].split(',')
            return int(parts[0]), int(parts[1]), float(parts[2]) if len(parts) > 2 else 12.0
        except Exception as e:
            self.get_logger().warn(f'Encoder parse error: {e}')
            return self.prev_left_ticks, self.prev_right_ticks, 12.0

    def _cmd_vel_callback(self, msg: Twist):
        if self.serial_conn is not None:
            v_right = msg.linear.x + (msg.angular.z * WHEEL_BASE / 2.0)
            v_left  = msg.linear.x - (msg.angular.z * WHEEL_BASE / 2.0)
            try:
                self.serial_conn.write(f'CMD:{v_right:.3f},{v_left:.3f}\n'.encode())
            except Exception as e:
                self.get_logger().warn(f'Serial write error: {e}')

    def _publish_odometry(self, stamp, dist_center, delta_theta):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_footprint'
        msg.pose.pose.position.x  = self.x
        msg.pose.pose.position.y  = self.y
        msg.pose.pose.position.z  = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(self.theta)
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.01
        self.odom_pub.publish(msg)

    def _publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = _yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def _publish_battery(self, voltage):
        msg = BatteryState()
        msg.voltage = voltage
        msg.present = True
        self.battery_pub.publish(msg)

    def destroy_node(self):
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()

def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

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
