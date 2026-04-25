import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

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

        self.declare_parameter('focal_length_px', 800.0)
        self.declare_parameter('depth_threshold', 1.0)
        
        self.focal_length_px = self.get_parameter('focal_length_px').value
        self.depth_threshold = self.get_parameter('depth_threshold').value

        # Enforced by table: Subscribes STRICTLY to /camera_frames
        self.subscription = self.create_subscription(
            Image, '/camera_frames', self.frame_callback, 10
        )
        self.publisher_ = self.create_publisher(String, '/depth_data', 10)

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.get_logger().info('Geometry-based Depth Estimation Node ready.')

    def frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)

        depths = []
        objects_with_depth = []

        for result in results:
            for box in result.boxes:
                cls = result.names[int(box.cls)].lower()
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                pixel_height = max(1, y2 - y1)
                
                real_h = REAL_HEIGHTS_M.get(cls, 0.5)
                estimated_distance = (self.focal_length_px * real_h) / pixel_height
                
                objects_with_depth.append({
                    'class': cls,
                    'distance': estimated_distance
                })
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

if __name__ == '__main__':
    main()