import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BumpGo(Node):
    def __init__(self):
        super().__init__('bump_go')

        #Definición de los estados

        self.FORWARD = 0
        self.BACK = 1
        self.TURN = 2
        self.STOP = 3
        self.state = self.FORWARD
        self.state_ts = self.get_clock().now()

        self.TURNING_TIME = 2.0
        self.BACKING_TIME = 2.0
        self.SCAN_TIMEOUT = 1.0

        #Valores para las velocidades en m/s, rad/s y distancia al obtaculo en m

        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3
        self.OBSTACLE_DISTANCE = 1.0

        self.last_scan = None

        #creo el subsriptor al tópico del laser

        self.scan_sub = self.create_subscription(
            LaserScan,
            'input_scan',
            self.scan_callback,
            qos_profile_sensor_data)
        
        #creo el publisher de la velocidad y su timer

        self.vel_pub = self.create_publisher(
            Twist,
            'output_vel',
            10
        )
        self.timer = self.create_timer(
            0.05,
            self.control_callback
        )
    
    #creo el laser callback, que recibe los datos del laser y los pone en la variable last_scan
    def scan_callback(self, msg):
        self.last_scan = msg

    def control_callback(self):

        #Primero miro que tenga listo el laser

        if self.last_scan == None:
            return
        
        #creo el msg donde va la velocidad
        out_vel = Twist()

        #Si el estado es ir adelante, mando la  velocidad adelante xD
        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR

            if self.check_forward_2_stop():
                self.go_state(self.STOP)
            if self.check_forward_2_back():
                self.go_state(self.BACK)

        
        #Resvio que pasa si el estado es Back, mando velocidad negativa por 2 s
        elif self.state == self.BACK:
            out_vel.linear.x = -self.SPEED_LINEAR

            if self.check_back_2_turn():
                self.go_state(self.TURN)
        
        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR

            if self.check_turn_2_forward():
                self.go_state(self.FORWARD)
        
        elif self.state == self.STOP:
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)
        
        #publico la  out_vel en el topic output_vel
        self.vel_pub.publish(out_vel)
        

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_forward_2_back(self):
        pos = round(len(self.last_scan.ranges) / 2)
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed > Duration(seconds=self.SCAN_TIMEOUT)

    def check_stop_2_forward(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)

    def check_back_2_turn(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.BACKING_TIME)

    def check_turn_2_forward(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)
    
    
def main(args=None):
    rclpy.init(args=args)

    bump_go_node = BumpGo()

    rclpy.spin(bump_go_node)
    bump_go_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        

    
    








