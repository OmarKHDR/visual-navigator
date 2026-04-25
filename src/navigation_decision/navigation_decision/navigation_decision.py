import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_nav_interfaces.srv import EstimateMotion
import json

class NavigationDecisionNode(Node):
    def __init__(self):
        super().__init__('navigation_decision_node')

        self.declare_parameter('safety_distance', 0.4)
        self.safety_distance = self.get_parameter('safety_distance').value

        self.latest_camera_motion = None
        self.latest_object_data = None
        self.latest_depth_data = None

        self.motion_sub = self.create_subscription(String, '/camera_motion', self.motion_callback, 10)
        self.object_sub = self.create_subscription(String, '/object_data', self.object_callback, 10)
        self.depth_sub = self.create_subscription(String, '/depth_data', self.depth_callback, 10)

        # Per table: Publishes to /navigation_command
        self.publisher_ = self.create_publisher(String, '/navigation_command', 10)
        self._motion_client = self.create_client(EstimateMotion, '/estimate_motion')
        self.timer = self.create_timer(0.2, self.make_decision)

    def motion_callback(self, msg):
        try:
            self.latest_camera_motion = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_camera_motion = None

    def object_callback(self, msg):
        try:
            self.latest_object_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_object_data = None

    def depth_callback(self, msg):
        try:
            self.latest_depth_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_depth_data = None

    def publish_command(self, command, reason, priority):
        msg = String()
        msg.data = json.dumps({'command': command, 'reason': reason, 'priority': priority})
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Command: {command} | {reason}')

    def make_decision(self):
        # Perception Rule: Every frame must have at least one detected object
        if self.latest_object_data is None or self.latest_object_data.get('count', 0) == 0:
            self.get_logger().warn('Blocking decision: No objects detected yet.')
            return

        command = 'STOP'
        reason = 'Waiting for data'
        priority = 'LOW'

        # Unreliable motion MUST STOP
        if self.latest_camera_motion is not None and not self.latest_camera_motion.get('reliable', True):
            self.publish_command('STOP', 'Motion estimation unreliable', 'HIGH')
            return

        # Check safety distance
        if self.latest_depth_data is not None and self.latest_depth_data.get('too_close', False):
            self.publish_command('STOP', 'Obstacle too close', 'HIGH')
            return

        direction = self.latest_camera_motion.get('direction', 'UNKNOWN') if self.latest_camera_motion else 'UNKNOWN'
        
        if direction == 'LEFT':
            command = 'MOVE_RIGHT'
            reason = 'Camera drifting left'
            priority = 'MEDIUM'
        elif direction == 'RIGHT':
            command = 'MOVE_LEFT'
            reason = 'Camera drifting right'
            priority = 'MEDIUM'
        elif direction == 'FORWARD':
            command = 'MOVE_FORWARD'
            reason = 'Path clear'
            priority = 'LOW'
        elif direction == 'BACKWARD':
            command = 'STOP'
            reason = 'Backward drift detected'
            priority = 'MEDIUM'
        else:
            command = 'MOVE_FORWARD'
            reason = 'Defaulting forward'
            priority = 'LOW'

        self.publish_command(command, reason, priority)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
