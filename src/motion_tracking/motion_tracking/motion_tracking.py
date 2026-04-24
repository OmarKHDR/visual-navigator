import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.parameter import parameter_value_to_python
import numpy as np

class MotionTrackingNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'roi_features',
            self.sub_callback,
            10
            )
        self.pub = self.create_publisher(
            Float32MultiArray,
            'motion_data',
            10
            )
        self.declare_parameter('motion_threshold', 0.05)
        self.motion_threshold = self.get_parameter('motion_threshold').value
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name="motion_threshold",
            node_name="motion_node",
            callback=self.param_callback,
        )
        self.prev_data = None
    def sub_callback(self, msg):
        num_cells=len(msg.data)//3
        feature_matrix=np.array(msg.data).reshape(num_cells,3)
        #print(feature_matrix)
        motion_vectors=[]
        if self.prev_data is not None:
            for i in range (num_cells):
                curr_x,curr_y,intensity= feature_matrix[i]
                prev_x,prev_y,_=self.prev_data[i]
                dx = curr_x-prev_x
                dy = curr_y-prev_y
                magnitude= np.sqrt(dx**2+dy**2)

                if (intensity>240 or intensity<20) or (magnitude<self.motion_threshold):
                    motion_vectors.extend([0.0,0.0])
                else:
                     motion_vectors.extend([float(dx),float(dy)])
        else:
            self.prev_data=feature_matrix

        output= Float32MultiArray()
        output.data=motion_vectors
        self.pub.publish(output)
        #print(output)
        self.prev_data=feature_matrix


    def param_callback(self, p: rclpy.parameter.Parameter) -> None:
        self.motion_threshold =int(parameter_value_to_python(p.value))
        self.get_logger().info(f"Received an update to parameter: {p.name}: {rclpy.parameter.parameter_value_to_python(p.value)}")    

def main(args=None):
    rclpy.init(args=args)
    node = MotionTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()
