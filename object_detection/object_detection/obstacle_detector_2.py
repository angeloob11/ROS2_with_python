import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from rclpy.duration import Duration
import numpy as np

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q


def Create_Transform(Q, T):
  R = quaternion_rotation_matrix(Q)
  T = np.array([[T[0]],[T[1]],[T[2]]])
  Rt = np.concatenate((R,T), axis=1)
  Transform = np.concatenate((Rt, [[0,0,0,1]]), axis=0)
  return Transform


class Obj_Det(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_sub = self.create_subscription(LaserScan,'input_scan',self.scan_callback,qos_profile_sensor_data)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def scan_callback(self, msg):
        #distancia al frente
        dist = msg.ranges[round(len(msg.ranges)/2)]

        if(not math.isinf(dist)):

            laser2object = Create_Transform([0,0,0,1], [dist, 0, 0])

            odom2laser_msg= TransformStamped()

            try: 
                odom2laser_msg = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                R = [odom2laser_msg.transform.rotation.x,
                     odom2laser_msg.transform.rotation.y,
                     odom2laser_msg.transform.rotation.z,
                     odom2laser_msg.transform.rotation.w]
                T = [odom2laser_msg.transform.translation.x,
                     odom2laser_msg.transform.translation.y,
                     odom2laser_msg.transform.translation.z]
                odom2laser = Create_Transform(R, T)
            except TransformException:
                self.get_logger().info(f'Could not find the Transform: {TransformException}')
            
            odom2object = odom2laser.dot(laser2object)

            odom2object_msg = TransformStamped()
            odom2object_msg.header.stamp = msg.header.stamp
            odom2object_msg.header.frame_id = "odom"
            odom2object_msg.child_frame_id = "detected_obstacle"
            
            Q = quaternion_from_matrix(odom2object)

            odom2object_msg.transform.rotation.x = Q[0] 
            odom2object_msg.transform.rotation.y = Q[1]
            odom2object_msg.transform.rotation.z = Q[2]
            odom2object_msg.transform.rotation.w = Q[3]
            odom2object_msg.transform.translation.x = odom2object[0,3]
            odom2object_msg.transform.translation.y = odom2object[1,3]
            odom2object_msg.transform.translation.z = odom2object[2,3]
            self.tf_static_broadcaster.sendTransform(odom2object_msg)

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