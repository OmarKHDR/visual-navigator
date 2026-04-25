import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_nav_interfaces.action import Navigate
import time

class MockActionServer(Node):
    def __init__(self):
        super().__init__('mock_action_server')
        self._action_server = ActionServer(
            self,
            Navigate,
            '/navigate_action',
            self.execute_callback
        )
        self.get_logger().info('Mock Action Server started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request: {goal_handle.request.command}')
        
        feedback_msg = Navigate.Feedback()
        feedback_msg.status = "Executing"
        
        # Simulate execution over 5 seconds
        for i in range(1, 6):
            feedback_msg.progress = i * 0.2
            self.get_logger().info(f'Publishing Feedback: {feedback_msg.status} ({feedback_msg.progress * 100:.0f}%)')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)  # Sleep 1s per feedback step

        goal_handle.succeed()
        
        result = Navigate.Result()
        result.success = True
        result.final_status = "COMPLETED_SUCCESSFULLY"
        self.get_logger().info('Goal execution completed successfully.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
