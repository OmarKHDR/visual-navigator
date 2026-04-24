import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.parameter import parameter_value_to_python
import numpy as np

class RoiFeatureNode(Node):
    def __init__(self):
        super().__init__('roi_feature_node')
        self.bridge=CvBridge()
        self.sub = self.create_subscription(
            Image,
            'camera_frames',
            self.sub_callback,
            10
            )
        self.pub = self.create_publisher(
            Float32MultiArray,
            'roi_features',
            10
            )
        #######
        self.declare_parameter('roi_size', 5)
        self.roi_size = self.get_parameter('roi_size').value
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="roi_size",
            node_name="roi_feature_node",
            callback=self.param_callback,
        )

    def sub_callback(self, msg):
        frame= self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,w=gray.shape
        dy, dx = h //self.roi_size ,w//self.roi_size
        features=[]
        for r in range (self.roi_size):
            for c in range (self.roi_size):
                y_s, x_s = r*dy, c*dx
                cell= gray[y_s:y_s+dy, x_s:x_s+dx]
                avg=float(np.mean(cell))
                M = cv2.moments(cell)
                if M["m00"] == 0:
                    cx,cy= dx/2,dy/2
                else:
                    cx=(M["m10"]/M["m00"])
                    cy=(M["m01"]/M["m00"])
                features.extend([x_s+cx,y_s+cy,avg])
        #print(len(features))
        output= Float32MultiArray()
        output.data=features
        self.pub.publish(output)
        #print(output.data)        
                
    def param_callback(self, p: rclpy.parameter.Parameter) -> None:
        self.roi_size =int(parameter_value_to_python(p.value))
        self.get_logger().info(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")

def main(args=None):
    rclpy.init(args=args)
    node = RoiFeatureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()