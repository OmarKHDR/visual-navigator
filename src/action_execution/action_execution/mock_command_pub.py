import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MockCommandPublisher(Node):
    def __init__(self):
        super().__init__('mock_command_publisher')
        self.publisher_ = self.create_publisher(String, '/navigation_command', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.command_index = 0
        self.commands = [
            {'command': 'MOVE_FORWARD', 'reason': 'Path clear', 'priority': 'NORMAL'},
            {'command': 'TURN_LEFT', 'reason': 'Obstacle ahead', 'priority': 'HIGH'},
            {'command': 'STOP', 'reason': 'Destination reached', 'priority': 'CRITICAL'}
        ]
        self.get_logger().info('Mock Command Publisher started.')

    def timer_callback(self):
        cmd = self.commands[self.command_index % len(self.commands)]
        self.command_index += 1
        msg = String()
        msg.data = json.dumps(cmd)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MockCommandPublisher()
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
