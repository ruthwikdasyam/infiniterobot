#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time



def main():
    rclpy.init()
    print('Creating \'Subscriber\' node')
    node = rclpy.create_node('Subscriber')
    node.create_subscription(Float64MultiArray, '/joint_velocity_controller/commands', print_vel, 10)
    node.create_subscription(Float64MultiArray, '/joint_position_controller/commands', print_pos,10)
    rclpy.spin(node)


def print_vel(msg):
    print("Reading Velocity: ",msg.data)
    
def print_pos(msg):
    print("Reading Poistion: ",msg.data)
    print('\n')




if __name__=='__main__':
    main()