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

        # Subscribes to /navigation_command (per table 3.8 and 4)
        self.command_sub = self.create_subscription(
            String, '/navigation_command', self.command_callback, 10
        )

        # Publishes to /action_status (per table 3.8)
        self.status_pub = self.create_publisher(String, '/action_status', 10)

        # Uses Action (Consumer of /navigate_action) (per table 3.8 and 4)
        self._action_client = ActionClient(self, Navigate, '/navigate_action')

        self.get_logger().info('Action Execution Node started (Bridge to ActionServer).')

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            command = data.get('command', 'STOP')
            reason = data.get('reason', '')
            priority = data.get('priority', 'LOW')
        except json.JSONDecodeError:
            return

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.publish_status(command, 'SERVER_UNAVAILABLE', reason)
            return

        goal_msg = Navigate.Goal()
        goal_msg.command = command
        goal_msg.reason = reason
        goal_msg.priority = priority

        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

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

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status('UNKNOWN', 'REJECTED', 'Goal rejected by Action Server')
            return

        self.publish_status('UNKNOWN', 'ACCEPTED', 'Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.publish_status('UNKNOWN', feedback.status, 'Providing feedback', feedback.progress * 100)

    def get_result_callback(self, future):
        result = future.result().result
        status = 'COMPLETED' if result.success else result.final_status
        self.publish_status('UNKNOWN', status, 'Action finished')

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
