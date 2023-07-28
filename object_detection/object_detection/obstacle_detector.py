import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Obj_Det(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.laser_sub = self.create_subscription(LaserScan,'input_scan',self.scan_callback,qos_profile_sensor_data)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def scan_callback(self, msg):
        #distancia al frente
        dist = msg.ranges[round(len(msg.ranges)/2)]

        if(not math.isinf(dist)):
            detection_tf = TransformStamped()
            detection_tf.header = msg.header
            detection_tf.child_frame_id = "detected_obstacle"
            detection_tf.transform.translation.x = msg.ranges[round(len(msg.ranges)/2)]
            self.tf_static_broadcaster.sendTransform(detection_tf)

def main(args=None):
    rclpy.init(args=args)
    Detect_Object = Obj_Det()
    try: 
        rclpy.spin(Detect_Object)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

