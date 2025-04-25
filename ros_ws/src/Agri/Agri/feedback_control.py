#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import serial
import time
from math import sin, cos


WHEEL_RADIUS = 5  # assumption, verify with mechanical
TICKS_PER_REVOLUTION = 1000  # assumption, check manually through encoder with electrical
CIRCUMFERENCE = 2 * 3.14 * WHEEL_RADIUS
DISTANCE_PER_TICK = CIRCUMFERENCE / TICKS_PER_REVOLUTION

class FeedbackControl(Node):

    def __init__(self):
        super().__init__('Feedback')

        # Encoder values from previous second
        self.ENCODER1 = 0
        self.ENCODER2 = 0
        self.ENCODER3 = 0
        self.ENCODER4 = 0

        # (x,y) of the robot initially origin
        self.x = 0
        self.y = 0

        self.DL = None  # Distance travelled by Left wheel
        self.DR = None  # Distance travelled by Right wheel
        self.DW = 10  # Assumption, check with mechanical -> Center between L and R wheel
        self.D = None  # Distance travelled by DW -> (DL + DR) / 2

        # Angles -> θt = θt-1 + Δθ
        self.deltaTheta = None
        self.currentTheta = 0
        self.nextTheta = 0

        self.serial_port = '/dev/ttyUSB0'  
        self.baud_rate = 115200

        # Initialize the serial connection
        self.SERIAL = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Define the publisher for IMU and Encoder data
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.pose_publisher = self.create_publisher(Pose2D, "encoder_ticks", 10)

        # Define the publisher for Odometry data
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Allow some time for the serial connection to initialize
        time.sleep(2)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz loop rate

    def timer_callback(self):
        try:
            # Read line from serial
            line = self.SERIAL.readline().decode('utf-8').strip()

            self.get_logger().info(f'{line}')
            data = line.split(',')

            # Parse accelerometer and gyroscope data
            ax, ay, az = float(data[0]), float(data[1]), float(data[2])
            gx, gy, gz = float(data[3]), float(data[4]), float(data[5])

            # Parse encoder data
            EncoderValue_1 = int(data[6])
            EncoderValue_2 = int(data[7])
            EncoderValue_3 = int(data[8])
            EncoderValue_4 = int(data[9])

            # Distance traveled by the wheels in 0.1 seconds
            DISTANCE_ENCODER_1 = (EncoderValue_1 - self.ENCODER1) * DISTANCE_PER_TICK
            DISTANCE_ENCODER_2 = (EncoderValue_2 - self.ENCODER2) * DISTANCE_PER_TICK
            DISTANCE_ENCODER_3 = (EncoderValue_3 - self.ENCODER3) * DISTANCE_PER_TICK
            DISTANCE_ENCODER_4 = (EncoderValue_4 - self.ENCODER4) * DISTANCE_PER_TICK

            # Update the encoder values for the next iteration
            self.ENCODER1 = EncoderValue_1
            self.ENCODER2 = EncoderValue_2
            self.ENCODER3 = EncoderValue_3
            self.ENCODER4 = EncoderValue_4

            # Calculate distances
            self.DL = min(DISTANCE_ENCODER_1, DISTANCE_ENCODER_2)
            self.DR = min(DISTANCE_ENCODER_3, DISTANCE_ENCODER_4)
            self.D = (self.DL + self.DR) / 2

            # Update theta
            self.deltaTheta = (self.DR - self.DL) / (2 * self.DW)
            self.nextTheta = self.currentTheta + self.deltaTheta

            # Update x, y coordinates
            self.x = self.x + (self.D * cos(self.currentTheta + (self.deltaTheta / 2)))
            self.y = self.y + (self.D * sin(self.currentTheta + (self.deltaTheta / 2)))

            # --- Publish IMU Data ---
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            self.imu_publisher.publish(imu_msg)
            self.get_logger().info(f'Published IMU data: Accel({ax}, {ay}, {az}), Gyro({gx}, {gy}, {gz})')

            # --- Publish Pose Data ---
            pose_msg = Pose2D()
            pose_msg.x = float(self.x)
            pose_msg.y = float(self.y)
            pose_msg.theta = float(self.nextTheta)
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Published Pose: x: {pose_msg.x}, y: {pose_msg.y}, theta: {pose_msg.theta}")

            # Set currentTheta as nextTheta for the next iteration
            self.currentTheta = self.nextTheta

            # --- Publish Odometry Data ---
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            # Set position in odometry message
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            # Convert theta to quaternion
            quaternion = quaternion_from_euler(0, 0, self.nextTheta)
            odom_msg.pose.pose.orientation = Quaternion(*quaternion)

            # Set velocity in odometry message
            odom_msg.twist.twist.linear.x = self.D / 0.1  # m/s
            odom_msg.twist.twist.angular.z = self.deltaTheta / 0.1  # rad/s

            # Publish the odometry message
            self.odom_publisher.publish(odom_msg)
            self.get_logger().info(f'Published Odometry: x: {self.x}, y: {self.y}, theta: {self.nextTheta}')

        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')


def main(args=None):
    rclpy.init(args=args)
    feedback = FeedbackControl()

    try:
        rclpy.spin(feedback)

    except KeyboardInterrupt:
        feedback.get_logger().info('Shutting down serial reader node.')

    feedback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
