#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from custom_inferfaces.srv import ButtonService  # Import your custom service

class TeleopJoyNode(Node):
    def __init__(self):
        super().__init__('teleop_joy_node')

        # Define scaling factors for the joystick input
        self.linear_scale = 1.0  # Scale for linear movement
        self.angular_scale = 1.0  # Scale for angular movement
        
        # To keep track of the previous button states to detect changes
        self.prev_buttons = [0, 0]  # Initialize for the first 3 buttons (can be adjusted)

        # Subscriber to 'joy' topic (joystick input)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Publisher for 'cmd_vel' topic (movement commands)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Service client to call 'button_state' service
        self.client = self.create_client(ButtonService, 'button_state')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for button state service...')

    def joy_callback(self, joy_msg):
        """
        Callback function for joystick input. Sends velocity commands to the robot
        and calls the service when button 0 or button 2 is pressed or released.
        """
        # Create a Twist message based on joystick input
        twist = Twist()
        x = joy_msg.axes[1] * self.linear_scale  # Joystick forward/backward axis
        z = joy_msg.axes[2] * self.angular_scale  # Joystick left/right axis
        twist.linear.x = x
        twist.angular.z = z

        # Publish the Twist message to control robot movement
        self.cmd_pub.publish(twist)

        # Log the current velocity
        self.get_logger().info(f"Linear x={float(x)}, Angular z={float(z)}")

        # Check the state of the first three buttons
        current_buttons = [joy_msg.buttons[0], joy_msg.buttons[1]]
        self.get_logger().info(f"Button states: {current_buttons}")

        # Check for changes in button 0 or button 2 state (press/release)
        button_states=[False, False]
        if current_buttons[0] != self.prev_buttons[0] or current_buttons[1] != self.prev_buttons[1]:
            # Convert the button states (True/False) and send the service request
            if (current_buttons[0] > self.prev_buttons[0]):
                button_states[0]=bool(1)
            else:
                button_states[0]=bool(0)

            if (current_buttons[1] > self.prev_buttons[1]):
                button_states[1]=bool(1)
            else:
                button_states[1]=bool(0)
            self.send_button_service_request(button_states)

            # Update previous button states
            self.prev_buttons[0] = current_buttons[0]
            self.prev_buttons[1] = current_buttons[1]

    def send_button_service_request(self, button_states):
        """
        Sends a request to the ButtonService when a button state changes.
        """
        # Create the service request
        request = ButtonService.Request()
        request.button_states = button_states  # Send the list of button states (True/False)

        # Log the button states being sent
        self.get_logger().info(f"Sending button states: {button_states}")

        # Call the service asynchronously
        future = self.client.call_async(request)
        
        # Add callback for when the service call completes
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """
        Handles the response from the ButtonService.
        """
        try:
            # Retrieve the result and log the success status
            response = future.result()
            self.get_logger().info(f'Service call success: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

