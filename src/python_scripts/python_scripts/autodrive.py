# from python_scripts.pubsub import MinimalSubscriber
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

import time


class L_Subscriber(Node):
    def __init__(self):
        super().__init__('L_Subscriber')
        self.L_subscriber = self.create_subscription(LaserScan,'/scan',self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.L_subscriber
        self.drive = self.create_publisher(Float64MultiArray,'/joint_velocity_controller/commands',10)
        self.driver = self.create_publisher(Float64MultiArray,'/joint_position_controller/commands',10)

    def scan_callback(self, msg):
        # Process the scan data
        self.get_logger().info("Minimum range: {}".format((msg.ranges)))
        self.rangelist(msg)

    def rangelist(self, msg):
        vel = Float64MultiArray()
        pos = Float64MultiArray()
        vel.data=[-40.0,40.0]
        self.drive.publish(vel)
        pos.data=[0.0,0.0]
        self.driver.publish(pos)    
#iii
        if msg.ranges[-1] < msg.ranges[0]:
            k = msg.ranges[-1]-msg.ranges[-2]
        else:
            k = msg.ranges[1]-msg.ranges[0]

        if abs(k) < 0.5 :
            kp=2.5*k
            pos.data=[kp,kp]
            self.driver.publish(pos)

        if abs(k) > 0.6:
            # kp=3.0*k
            pos.data=[0.0,0.0]
            self.driver.publish(pos)

        print(vel.data)
        print(pos.data)
        print("kvalue ",k)

        return



def main():
    rclpy.init()

    #LIDAR
    LSubscriber = L_Subscriber()
    rclpy.spin(LSubscriber)

    LSubscriber.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()