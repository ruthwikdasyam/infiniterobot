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
        self.L_subscriber = self.create_subscription(LaserScan,'/scan',self.scan_callback, QoSProfile(depth=15, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.L_subscriber
        self.drive = self.create_publisher(Float64MultiArray,'/joint_velocity_controller/commands',15) # velocity controller node
        self.driver = self.create_publisher(Float64MultiArray,'/joint_position_controller/commands',15) # position controller node

    def scan_callback(self, msg):
        # Process the scan data
        # self.get_logger().info("Sensor Output {}".format((msg.ranges)))
        print(" Sensor Output ", msg.ranges)
        self.rangelist(msg)

    def rangelist(self, msg):
        vel = Float64MultiArray()
        pos = Float64MultiArray()
        vel.data=[-20.0,20.0]
        self.drive.publish(vel)
        pos.data=[0.0,0.0]
        self.driver.publish(pos)    
#iii
        if msg.ranges[-1] < msg.ranges[0]: #checking which way to turn
            # print(" Turning Left ")
            k = msg.ranges[-1]-msg.ranges[-2] # difference of left two lidar outputs
        else:
            # print(" Turning Right ")
            k = msg.ranges[1]-msg.ranges[0] # difference of right 2 lidar outputs

        if abs(k) < 0.4: # difference measure
            text = " TURNING "
            print(text)
            kp=0.05*(1/k) # proportional control
            pos.data=[kp,kp] 
            self.driver.publish(pos) # publshing
        
        if abs(k) > 0.5:
            text = " GOING STRAIGHT "
            print(text)
            # kp=3.0*k
            pos.data=[0.0,0.0] # going straight
            self.driver.publish(pos)

        print("velocity ->  ", vel.data[1])
        print("Turning  ->  ",round(pos.data[1],3))
        print(" ")
        # print("kvalue ",k)
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