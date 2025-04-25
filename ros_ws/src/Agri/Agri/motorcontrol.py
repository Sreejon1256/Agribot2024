#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from custom_inferfaces.srv import ButtonService
import serial

# Initialize the serial communication
SERIAL = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

class MotorSpeed(Node):

    def __init__(self):
        super().__init__('motor_control')

        # Subscribe to 'cmd_vel' for movement commands
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.publisher_motor_left = self.create_publisher(Float32, 'motor_left_speed', 10)
        self.publisher_motor_right = self.create_publisher(Float32, 'motor_right_speed', 10)

        # Service to receive button states
        self.srv = self.create_service(ButtonService, 'button_state', self.handle_button_state)

        # Maximum and minimum PWM values
        self.max_pwm = float(200)
        self.min_pwm = float(-200)

        # Track previous button states to detect changes
        self.prev_button_states = [False, False]  # Assuming two buttons, initialize as False

    def listener_callback(self, msg):
        """
        Callback for cmd_vel messages. This converts velocity commands to motor speeds.
        """

        # Calculate motor speeds based on linear and angular velocity
        motor_left_speed = msg.linear.x - msg.angular.z
        motor_right_speed = msg.linear.x + msg.angular.z

        # Scale the values to the range [-200, 200]
        motor_left_speed *= self.max_pwm
        motor_right_speed *= self.max_pwm

        # Ensure the PWM values are clamped between -200 and 200
        motor_left_speed = max(min(motor_left_speed, self.max_pwm), self.min_pwm)
        motor_right_speed = max(min(motor_right_speed, self.max_pwm), self.min_pwm)

        # Send the motor speeds over serial
        SERIAL.write(f'{motor_left_speed},{motor_right_speed},{motor_left_speed},{motor_right_speed}\n'.encode())

        # Publish the scaled PWM values for the motors
        self.publisher_motor_left.publish(Float32(data=motor_left_speed))
        self.publisher_motor_right.publish(Float32(data=motor_right_speed))

        self.get_logger().info(f"LEFT MOTOR: {motor_left_speed}, RIGHT MOTOR: {motor_right_speed}")

    def handle_button_state(self, request, response):
        """
        Service callback that handles button state changes.
        """
        button_states = request.button_states  # Assuming this is a list of booleans
        self.get_logger().info(f'Received button states: {button_states}')

        # Check if the button states have changed compared to the previous state
        if button_states != self.prev_button_states:
            # Send the new button states over serial as a list of strings
            SERIAL.write(f'{int(button_states[0])},{int(button_states[1])}\n'.encode())
            self.get_logger().info(f'Serially written button states: {button_states}')

            # Update the previous button states
            self.prev_button_states = button_states

        # Respond with success
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorSpeed()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()