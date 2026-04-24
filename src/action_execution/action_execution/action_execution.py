import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_nav_interfaces.action import Navigate
import time

class ActionExecutionNode(Node):
    def __init__(self):
        super().__init__('action_execution_node')

        self.declare_parameter('action_duration', 2.0)
        self.action_duration = self.get_parameter('action_duration').value

        self._action_server = ActionServer(
            self,
            Navigate,
            '/navigate_action',
            self.execute_callback
        )

        self.get_logger().info(f'Real Action Server started for /navigate_action | Duration: {self.action_duration}s')

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Executing goal -> Command: {goal_handle.request.command} '
            f'| Reason: {goal_handle.request.reason}'
        )

        feedback_msg = Navigate.Feedback()
        steps = 10
        duration = self.action_duration

        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result = Navigate.Result()
                result.success = False
                result.final_status = 'CANCELLED'
                return result

            feedback_msg.progress = float(i) / steps
            feedback_msg.status = 'IN_PROGRESS'
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Action Progress: {feedback_msg.progress * 100:.0f}%')
            time.sleep(duration / steps)

        goal_handle.succeed()

        result = Navigate.Result()
        result.success = True
        result.final_status = 'COMPLETED'
        self.get_logger().info('Goal succeeded.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
