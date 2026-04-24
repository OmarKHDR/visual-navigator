import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

# Estimate of real-world height of objects in meters
REAL_HEIGHTS_M = {
    'person': 1.7,
    'car': 1.5,
    'chair': 1.0,
    'door': 2.1,
    'bottle': 0.25,
    'cell phone': 0.15,
    'laptop': 0.25,
    'dining table': 0.8,
    'tv': 0.6,
    'book': 0.2
}

class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation_node')

        # Focal length in pixels (approximate for typical webcams)
        self.declare_parameter('focal_length_px', 800.0)
        self.declare_parameter('depth_threshold', 1.0) # spec parameter
        
        self.focal_length_px = self.get_parameter('focal_length_px').value
        self.depth_threshold = self.get_parameter('depth_threshold').value

        # Subscribe to objects instead of computing deep depth model
        self.subscription = self.create_subscription(
            String, '/object_data', self.object_data_callback, 10
        )
        self.publisher_ = self.create_publisher(String, '/depth_data', 10)

        self.get_logger().info('Geometry-based Depth Estimation Node ready.')

    def object_data_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        objects = data.get('objects', [])
        
        depths = []
        objects_with_depth = []

        for obj in objects:
            cls = obj.get('class', 'unknown').lower()
            x1, y1, x2, y2 = obj.get('bbox', [0, 0, 0, 0])
            pixel_height = max(1, y2 - y1)  # avoid div by zero
            
            # Default to 0.5m if object height is unknown
            real_h = REAL_HEIGHTS_M.get(cls, 0.5) 

            # Geometry Distance Formula: Z = (f * H) / h
            estimated_distance = (self.focal_length_px * real_h) / pixel_height
            
            obj_depth_info = {
                'class': cls,
                'distance': estimated_distance
            }
            objects_with_depth.append(obj_depth_info)
            depths.append(estimated_distance)

        out_data = {
            'mean_depth': round(float(np.mean(depths)) if depths else 999.0, 3),
            'min_depth': round(float(np.min(depths)) if depths else 999.0, 3),
            'objects_depth': objects_with_depth,
            'too_close': any(d < self.depth_threshold for d in depths)
        }

        out_msg = String()
        out_msg.data = json.dumps(out_data)
        self.publisher_.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
