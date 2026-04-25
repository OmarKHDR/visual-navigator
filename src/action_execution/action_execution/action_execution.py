import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from custom_nav_interfaces.action import Navigate
import json

class ActionExecutionNode(Node):
    def __init__(self):
        super().__init__('action_execution_node')

        self.declare_parameter('action_duration', 2.0)
        self.action_duration = self.get_parameter('action_duration').value
        self._current_goal_handle = None

        self.command_sub = self.create_subscription(
            String, '/navigation_command', self.command_callback, 10
        )

        self.status_pub = self.create_publisher(String, '/action_status', 10)

        self._action_client = ActionClient(self, Navigate, '/navigate_action')

        self.get_logger().info('Action Execution Node started.')

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            command = data.get('command', 'STOP')
            reason = data.get('reason', '')
            priority = data.get('priority', 'LOW')
        except json.JSONDecodeError:
            self.get_logger().warn('Received non valid JSON on /navigation_command')
            return

        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling in-flight goal before sending new one.')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.publish_status(command, 'SERVER_UNAVAILABLE', reason)
            return

        goal_msg = Navigate.Goal()
        goal_msg.command = command
        goal_msg.reason = reason
        goal_msg.priority = priority
        goal_msg.duration = self.action_duration

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.feedback_callback(fb, command, reason)
        )
        send_future.add_done_callback(
            lambda future: self.goal_response_callback(future, command, reason)
        )

    def publish_status(self, command, status, reason, progress=0.0):
        out_msg = String()
        out_msg.data = json.dumps({
            'command': command,
            'status': status,
            'reason': reason,
            'progress': progress
        })
        self.status_pub.publish(out_msg)
        self.get_logger().info(f'[{status}] {command} ({progress:.0f}%)')

    def goal_response_callback(self, future, command, reason):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status(command, 'REJECTED', 'Goal rejected by Action Server')
            self._current_goal_handle = None
            return

        self._current_goal_handle = goal_handle
        self.publish_status(command, 'ACCEPTED', reason)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.get_result_callback(future, command)
        )

    def feedback_callback(self, feedback_msg, command, reason):
        feedback = feedback_msg.feedback
        self.publish_status(command, feedback.status, reason, feedback.progress * 100)

    def get_result_callback(self, future, command):
        self._current_goal_handle = None
        result = future.result().result
        status = 'COMPLETED' if result.success else result.final_status
        self.publish_status(command, status, 'Action finished')

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()