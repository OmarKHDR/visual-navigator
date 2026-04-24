import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from custom_nav_interfaces.srv import EstimateMotion
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')

        self.bridge = CvBridge()
        self.prev_gray = None
        self.p0 = None
        
        self.direction = 'UNKNOWN'
        self.reliable = False

        self.subscription = self.create_subscription(
            Image, '/camera_frames', self.frame_callback, 10
        )
        self.publisher = self.create_publisher(String, '/camera_motion', 10)
        self.srv = self.create_service(
            EstimateMotion, '/estimate_motion', self.estimate_motion_callback
        )

        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict(maxCorners=100,
                                   qualityLevel=0.3,
                                   minDistance=7,
                                   blockSize=7)

        self.get_logger().info('Visual Odometry Node started.')

    def frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.p0 = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            return

        if self.p0 is None or len(self.p0) < 10:
            self.p0 = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.reliable = False
            self.prev_gray = gray
            return

        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.p0, None, **self.lk_params)

        if p1 is not None and st is not None:
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]

            if len(good_new) > 10:
                self.reliable = True
                dx = good_new[:, 0] - good_old[:, 0]
                dy = good_new[:, 1] - good_old[:, 1]
                mean_dx = np.mean(dx)
                mean_dy = np.mean(dy)

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

                self.p0 = good_new.reshape(-1, 1, 2)
            else:
                self.reliable = False
                self.direction = 'UNKNOWN'
        
        self.prev_gray = gray

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
