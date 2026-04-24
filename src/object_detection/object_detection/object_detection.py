import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')

        self.conf = self.get_parameter('confidence_threshold').value
        model_path = self.get_parameter('model_path').value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_detections = []

        self.subscription = self.create_subscription(
            Image, '/camera_frames', self.detect_callback, 10)

        self.publisher_ = self.create_publisher(String, '/object_data', 10)

        self.display_timer = self.create_timer(0.03, self.update_display)

        self.get_logger().info(
            f'Object Detection Node started | '
            f'Confidence threshold: {self.conf}'
        )

    def detect_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=self.conf, verbose=False)

        all_detections = []

        for result in results:
            for box in result.boxes:
                class_name = result.names[int(box.cls)]
                detection = {
                    'class': class_name,
                    'confidence': round(float(box.conf), 3),
                    # [x1, y1, x2, y2]
                    'bbox': [int(v) for v in box.xyxy[0].tolist()]
                }
                all_detections.append(detection)

        self.latest_frame = frame.copy()
        self.latest_detections = all_detections

        out_msg = String()
        out_msg.data = json.dumps({
            'count': len(all_detections),
            'objects': all_detections
        })
        self.publisher_.publish(out_msg)

    def update_display(self):
        if self.latest_frame is None:
            return

        display = self.latest_frame.copy()

        for det in self.latest_detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class']} {det['confidence']:.2f}"
            
            color = (0, 255, 0)
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            cv2.putText(display, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow('Object Detection Tracker', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()