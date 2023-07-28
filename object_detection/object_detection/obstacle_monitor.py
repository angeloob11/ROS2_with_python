import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Obj_Monitor(Node):
    def __init__(self):
        super().__init__('obstalce_monitor')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.maker_pub = self.create_publisher(Marker,'obstacle_marker',1)
        self.timer = self.create_timer(0.5,self.control_cycle)

    def control_cycle(self):
        robot2obtacle = TransformStamped()

        try: 
            robot2obtacle = self.tf_buffer.lookup_transform('base_link', 'detected_obstacle', rclpy.time.Time())
        except TransformException:
            self.get_logger().info(f'Could not transform: {TransformException}')
        
        x = robot2obtacle.transform.translation.x
        y = robot2obtacle.transform.translation.y
        z = robot2obtacle.transform.translation.z
        theta = math.atan2(y, x)

        self.get_logger().info(f'Obstacle detected at {x},{y},{z}, theta={theta}')

        obstacle_arrow = Marker()
        obstacle_arrow.header.frame_id = 'base_link'
        obstacle_arrow.header.stamp = self.get_clock().now().to_msg()
        obstacle_arrow.type = Marker.ARROW
        obstacle_arrow.action = Marker.ADD
        obstacle_arrow.lifetime = Duration(seconds=1.0).to_msg()

        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0
        end = Point()
        end.x = x
        end.y = y
        end.z = z
        obstacle_arrow.points = [start, end]

        obstacle_arrow.color.r = 1.0
        obstacle_arrow.color.g = 0.0
        obstacle_arrow.color.b = 0.0
        obstacle_arrow.color.a = 1.0

        obstacle_arrow.scale.x = 0.02
        obstacle_arrow.scale.y = 0.1
        obstacle_arrow.scale.z = 0.1
        self.maker_pub.publish(obstacle_arrow)
        
def main(args=None):
    rclpy.init(args=args)
    Monitor_Object = Obj_Monitor()
    try: 
        rclpy.spin(Monitor_Object)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
