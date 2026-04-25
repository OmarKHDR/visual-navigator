import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from custom_nav_interfaces.srv import EstimateMotion
import numpy as np
import json

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')

        self.declare_parameter('focal_length', 800.0)
        self.focal_length = self.get_parameter('focal_length').value

        self.direction = 'UNKNOWN'
        self.reliable = False

        self.subscription = self.create_subscription(
            Float32MultiArray, '/motion_data', self.motion_data_callback, 10
        )
        self.publisher = self.create_publisher(String, '/camera_motion', 10)
        self.srv = self.create_service(
            EstimateMotion, '/estimate_motion', self.estimate_motion_callback
        )

        self.get_logger().info('Visual Odometry Node started.')

    def motion_data_callback(self, msg):
        vectors = msg.data
        if not vectors or len(vectors) == 0:
            self.reliable = False
            self.direction = 'UNKNOWN'
        else:
            # Vectors are pairs of [dx, dy]
            vec_array = np.array(vectors).reshape(-1, 2)
            valid_vectors = vec_array[(vec_array[:, 0] != 0) | (vec_array[:, 1] != 0)]

            # Check for low feature count or dynamic/noisy scenes
            if len(valid_vectors) < 5:
                self.reliable = False
                self.direction = 'UNKNOWN'
            else:
                self.reliable = True
                mean_dx = np.mean(valid_vectors[:, 0])
                mean_dy = np.mean(valid_vectors[:, 1])

                if mean_dx > 2.0:
                    self.direction = 'LEFT'
                elif mean_dx < -2.0:
                    self.direction = 'RIGHT'
                elif abs(mean_dx) < 1.0 and abs(mean_dy) < 1.0:
                    self.direction = 'FORWARD'
                elif mean_dy > 2.0:
                    self.direction = 'BACKWARD'
                else:
                    self.direction = 'FORWARD'

        out_msg = String()
        out_msg.data = json.dumps({
            'direction': self.direction,
            'reliable': self.reliable
        })
        self.publisher.publish(out_msg)

    def estimate_motion_callback(self, request, response):
        response.direction = self.direction
        response.reliable = self.reliable
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()