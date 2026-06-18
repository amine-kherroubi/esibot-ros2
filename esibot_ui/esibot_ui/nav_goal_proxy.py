#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty
from nav2_msgs.action import NavigateToPose


class NavGoalProxy(Node):
    def __init__(self):
        super().__init__('nav_goal_proxy')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._status_pub = self.create_publisher(String, '/nav_goal_status', 10)

        # QoS BEST_EFFORT pour être compatible avec rosbridge
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, '/nav_goal', self._on_goal, qos)
        self._current_handle = None
        self._cancel_sub = self.create_subscription(
            Empty, '/cancel_nav_goal', self._on_cancel, 10)
        self.get_logger().info('NavGoalProxy ready — listening on /nav_goal, /cancel_nav_goal')

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'Nav goal status: {status}')

    def _on_cancel(self, msg):
        if self._current_handle is not None:
            self.get_logger().info('Cancelling current goal')
            self._current_handle.cancel_goal_async()
            self._current_handle = None
            self._publish_status('cancelled')
        else:
            self.get_logger().info('No active goal to cancel')

    def _on_goal(self, pose_msg: PoseStamped):
        self.get_logger().info(f'Received goal frame_id={pose_msg.header.frame_id} pos=({pose_msg.pose.position.x:.2f},{pose_msg.pose.position.y:.2f})')

        # Annuler le goal en cours avant d'en envoyer un nouveau
        if self._current_handle is not None:
            self.get_logger().info('Cancelling current goal before sending new one')
            self._current_handle.cancel_goal_async()
            self._current_handle = None

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server not available')
            self._publish_status('error')
            return

        self._publish_status('sending')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_msg.pose.position.x
        goal_pose.pose.position.y = pose_msg.pose.position.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = pose_msg.pose.orientation.z
        goal_pose.pose.orientation.w = pose_msg.pose.orientation.w

        self.get_logger().info(f'Sending goal frame_id={goal_pose.header.frame_id} to Nav2')

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = ''

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._publish_status('error')
            return
        self._current_handle = handle
        self._publish_status('navigating')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        pass

    def _result_cb(self, future):
        result = future.result()
        if result.status == 4:
            self._publish_status('reached')
        else:
            self._publish_status('error')
        self._current_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
