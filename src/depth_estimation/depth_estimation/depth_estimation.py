import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import json
from transformers import pipeline
from PIL import Image as PILImage  # ADD THIS

class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_estimation_node')

        self.declare_parameter('depth_model_path', 'depth-anything/Depth-Anything-V2-Small-hf')

        model_path  = self.get_parameter('depth_model_path').value
        self.bridge = CvBridge()

        self.get_logger().info('Loading DepthAnythingV2 model (first run may take a moment)...')
        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.depth_pipe = pipeline(
            task='depth-estimation',
            model=model_path,
            device=device
        )
        self.get_logger().info(f'Depth model loaded on {device}.')

        self.subscription = self.create_subscription(
            Image, '/camera_frames', self.depth_callback, 10)
        self.publisher_   = self.create_publisher(String, '/object_depth', 10)

        self.get_logger().info('Depth Estimation Node ready.')

    def depth_callback(self, msg):
        # Convert ROS image to OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR numpy array → RGB → PIL Image (what the pipeline expects)
        frame_rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image  = PILImage.fromarray(frame_rgb)

        # Run depth estimation
        result    = self.depth_pipe(pil_image)
        depth_map = np.array(result['depth'])

        # Normalize to 0-255 for visualization
        depth_min        = depth_map.min()
        depth_max        = depth_map.max()
        depth_normalized = (depth_map - depth_min) / (depth_max - depth_min + 1e-8)
        depth_uint8      = (depth_normalized * 255).astype(np.uint8)

        # Colorize and show depth window
        depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_MAGMA)
        cv2.imshow('Depth Estimation', depth_colored)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        # Divide into 3x3 grid and compute mean depth per region
        h, w   = depth_normalized.shape
        cell_h = h // 3
        cell_w = w // 3

        region_labels = [
            'top-left',  'top-center',  'top-right',
            'mid-left',  'center',      'mid-right',
            'bot-left',  'bot-center',  'bot-right'
        ]

        regions = {}
        for i, label in enumerate(region_labels):
            r, c = divmod(i, 3)
            region         = depth_normalized[r*cell_h:(r+1)*cell_h, c*cell_w:(c+1)*cell_w]
            regions[label] = round(float(np.mean(region)), 4)

        closest_region = max(regions, key=regions.get)
        closest_value  = regions[closest_region]

        depth_data = {
            'mean_depth':     round(float(np.mean(depth_normalized)), 4),
            'min_depth':      round(float(depth_normalized.min()), 4),
            'max_depth':      round(float(depth_normalized.max()), 4),
            'closest_region': closest_region,
            'closest_value':  closest_value,
            'regions':        regions
        }

        out_msg      = String()
        out_msg.data = json.dumps(depth_data)
        self.publisher_.publish(out_msg)

        self.get_logger().info(
            f'Depth published | Mean: {depth_data["mean_depth"]:.3f} '
            f'| Closest: {closest_region} ({closest_value:.3f})'
        )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()