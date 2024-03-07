#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from tf_transformations import euler_from_quaternion
import numpy as np

class NewCarProportionalController(Node):
    def __init__(self):
        super().__init__('car_proportional_controller')

        self.joint_positions = Float64MultiArray()
        self.wheel_velocities = Float64MultiArray()

        self.pos_publisher = self.create_publisher(Float64MultiArray, '/joint_position_controller/commands', 10)
        self.vel_publisher = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        # joint_state_pub = node.create_publisher(JointState, '/joint_states', 10)
            
        self.sub_node = rclpy.create_node('imu_data_subscriber_node')
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=5)
        self.sub_node.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, qos_profile)

        self.xpos = 0.000
        self.ypos = 0.000
        self.yaw_ang = 0.0
        self.ang_velocity = 0.0
        self.K_linear = 0.5
        self.K_angle_correction = 0.075
        self.last_timestamp = None
        self.iterations = 100
        self.steer_angle=0

        self.x_target = 10.0
        self.y_target = 10.0
        print("started")
        rclpy.spin_once(self.sub_node)

    def imu_callback(self, msg):
        self.wheel_radius = 4.0
        if self.last_timestamp is not None:
            current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            dt = current_timestamp - self.last_timestamp

            orientation = msg.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]

            roll, pitch, yaw = euler_from_quaternion(q)

            # Extract linear acceleration and angular velocity from the IMU message
            # linear_acceleration = msg.linear_acceleration
            self.angular_velocity_ = msg.angular_velocity.z

            # Estimate position in x and y based on linear acceleration
            dx = self.wheel_radius * self.angular_velocity_ * np.cos(self.steer_angle) * dt
            dy = self.wheel_radius * self.angular_velocity_ * np.sin(self.steer_angle) * dt

            self.xpos+=dx
            self.ypos+=dy
            self.yaw_ang=yaw
            # print("xpos: ", self.xpos)
            # # print("xposd: ", dx)
            # print("ypos: ", self.xpos)
            # # print("yposd: ", dy)

            # print("callback")
            # print(f"dx = { dx}, radius = {self.wheel_radius}, omega={self.angular_velocity_}, yaw={yaw}, dt={dt}" )

        self.last_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9



    def control_loop(self):
        rate=self.create_rate(10)
        initial_distance = abs(math.sqrt((( self.x_target)**2) + (( self.y_target)**2)))
        # initial_velocity = 1.0
        # steer_angle = 1.0
        # pos_publisher.publish(joint_positions)
        # vel_publisher.publish(wheel_velocities)
        distance = initial_distance
        rclpy.spin_once(self.sub_node)
        t = 0
        while distance >2 and rclpy.ok(): 
            rclpy.spin_once(self.sub_node)
            distance = abs(math.sqrt((( self.x_target - self.xpos)**2) + (( self.y_target - self.ypos)**2)))
            # print("Distance",distance)
            linear_velocity = distance * self.K_linear+3.0

            desired_angle = math.atan2((self.y_target),(self.x_target))
            if self.yaw_ang < desired_angle:
                self.steer_angle = 0.5
            else:
                self.steer_angle = 0.0
                if t>self.iterations:break
                else: t+=self.K_angle_correction
            self.wheel_velocities.data = [-linear_velocity, linear_velocity]
            self.joint_positions.data = [self.steer_angle, self.steer_angle]   
            self.pos_publisher.publish(self.joint_positions)
            self.vel_publisher.publish(self.wheel_velocities) 
            print("Publishing Velocity: ", self.wheel_velocities.data)
            print("Publishing Position: ", self.joint_positions.data)


        print('Goal Position Reached (10,10)')
        print('Stopping Car')
        self.steer_angle = 0.0
        linear_velocity = 0.0
        self.wheel_velocities.data = [-linear_velocity, linear_velocity]
        self.joint_positions.data = [self.steer_angle, self.steer_angle]
        print("Publishing Velocity: ", self.wheel_velocities.data)
        print("Publishing Position: ", self.joint_positions.data)        
        self.pos_publisher.publish(self.joint_positions)
        self.vel_publisher.publish(self.wheel_velocities)
        self.sub_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    car_proportional_controller = NewCarProportionalController()
    car_proportional_controller.control_loop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()