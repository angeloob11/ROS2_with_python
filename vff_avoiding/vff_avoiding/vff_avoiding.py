import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import numpy as np

class VFF(Node):
    def __init__(self):
        super().__init__('vff_node')

        self.vel_pub = self.create_publisher(Twist,'output_vel', 100)
        self.vff_debug_pub = self.create_publisher(MarkerArray, 'vff_debug', 100)
        self.scan_sub = self.create_subscription(LaserScan, 'input_scan', self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.control_cycle)
        self.last_scan = None

    def scan_callback(self, msg):
        self.last_scan = msg
    
    def control_cycle(self):

        if(self.last_scan == None):
            return

        vector, atractive, repulsive = self.get_vff(self.last_scan)
        angle = math.atan2(vector[1], vector[0])
        module = math.sqrt(vector[0]*vector[0] + vector[1]*vector[1])
        vel = Twist()
        vel.linear.x = np.clip(module, 0.0, 0.3)
        vel.angular.z = np.clip(angle, -0.5, 0.5)
        

        self.vel_pub.publish(vel)

        if(self.vff_debug_pub.get_subscription_count()>0):
            self.vff_debug_pub.publish(self.get_debug_vff(atractive, repulsive, vector))
            self.get_logger().info(f'vff:{repulsive}')
        
    
    def get_vff(self, msg):

        OBSTACLE_DISTANCE = 1.0
        vff_atractive = np.array([OBSTACLE_DISTANCE,0.0])
        vff_repulsive = np.array([0.0,0.0])
        vff_result = np.array([0.0,0.0])

        min_dist = min(msg.ranges)
        for i, obj in enumerate(msg.ranges):
            if obj == min_dist:
                min_indx = i
                

        if(min_dist < OBSTACLE_DISTANCE):
            
            angle = msg.angle_min + msg.angle_increment * min_indx
            oposite_angle = angle + math.pi
            complemetary_dist = OBSTACLE_DISTANCE - min_dist

            vff_repulsive[0] = np.cos(oposite_angle) * complemetary_dist
            vff_repulsive[1] = np.sin(oposite_angle) * complemetary_dist

        vff_result[0] = vff_atractive[0] + vff_repulsive[0]
        vff_result[1] = vff_atractive[1] + vff_repulsive[1]
        return vff_result, vff_atractive, vff_repulsive
    
    def get_debug_vff(self, v, w, u):

        marker_array = MarkerArray()
        marker_array.markers.append(self.make_marker(v, 'BLUE'))
        marker_array.markers.append(self.make_marker(w, 'RED'))
        marker_array.markers.append(self.make_marker(u, 'GREEN'))

        return marker_array
    
    def make_marker(self, v, color):
        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = 0.0
        start.y = 0.0
        end = Point()
        end.x = v[0]
        end.y = v[1]
        marker.points = [start, end]

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.color.a = 1.0

        if(color == 'RED'):
            marker.id = 0
            marker.color.r = 1.0
        elif(color == 'GREEN'):
            marker.id = 1
            marker.color.g = 1.0
        elif(color == 'BLUE'):
            marker.id = 2
            marker.color.b = 1.0
        
        return marker

def main(args=None):
    rclpy.init(args=args)

    VFF_NODE = VFF()

    rclpy.spin(VFF_NODE)
    VFF_NODE.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    


        




    
