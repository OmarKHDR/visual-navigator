import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from custom_nav_interfaces.action import Navigate
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

        self.motion_sub = self.create_subscription(
            String, '/camera_motion', self.motion_callback, 10)

        self.object_sub = self.create_subscription(
            String, '/object_data', self.object_callback, 10)

        self.depth_sub = self.create_subscription(
            String, '/depth_data', self.depth_callback, 10)

        # True Action Client
        self._action_client = ActionClient(self, Navigate, '/navigate_action')
        
        # Service Client for EstimateMotion
        self._motion_client = self.create_client(EstimateMotion, '/estimate_motion')

        self.timer = self.create_timer(0.2, self.make_decision)

        self.get_logger().info(f'Navigation Decision Node started | Safety distance: {self.safety_distance}')

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

    def send_goal(self, command, reason, priority):
        self.get_logger().info(f'Sending action goal: {command} | {reason}')
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available!')
            return

        goal_msg = Navigate.Goal()
        goal_msg.command = command
        goal_msg.reason = reason
        goal_msg.priority = priority

        self._action_client.send_goal_async(goal_msg)

    def request_motion_estimate(self):
        if self._motion_client.wait_for_service(timeout_sec=0.1):
            req = EstimateMotion.Request()
            req.empty = True
            self._motion_client.call_async(req)
        
    def make_decision(self):
        # Rule: Call the service periodically to keep motion estimation updated
        self.request_motion_estimate()

        command = 'STOP'
        reason = 'Waiting for data'
        priority = 'LOW'

        if self.latest_camera_motion is None and self.latest_depth_data is None and self.latest_object_data is None:
            self.send_goal(command, reason, priority)
            return

        if self.latest_camera_motion is not None and not self.latest_camera_motion.get('reliable', True):
            self.send_goal('STOP', 'Motion estimation unreliable', 'HIGH')
            return

        if self.latest_depth_data is not None and self.latest_depth_data.get('too_close', False):
            self.send_goal('STOP', 'Obstacles too close based on geometry depth', 'HIGH')
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
            reason = 'Backward motion detected'
            priority = 'MEDIUM'
        else:
            command = 'MOVE_FORWARD'
            reason = 'Defaulting forward'
            priority = 'LOW'

        self.send_goal(command, reason, priority)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
