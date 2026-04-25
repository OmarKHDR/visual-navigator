import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # Parameters
        self.declare_parameter('camera_source', '0')
        self.declare_parameter('frame_rate', 10.0)

        camera_source = self.get_parameter('camera_source').value
        frame_rate    = self.get_parameter('frame_rate').value

        try:
            source = int(camera_source)
        except (ValueError, TypeError):
            source = camera_source

        self.cap    = cv2.VideoCapture(source)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera source: {source}')
            return

        self.publisher_ = self.create_publisher(Image, '/camera_frames', 10)
        self.timer      = self.create_timer(1.0 / frame_rate, self.publish_frame)

        self.get_logger().info(f'Camera Stream Node started. Source: {source}')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame.')
            return

        # Publish to ROS topic
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Published frame to /camera_frames')

        # Show the frame in a window
        cv2.imshow('Camera Stream', frame)

        # Press 'q' to quit the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit key pressed. Shutting down.')
            cv2.destroyAllWindows()
            self.cap.release()
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()