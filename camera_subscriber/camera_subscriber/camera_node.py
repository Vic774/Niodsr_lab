#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class NewPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.window_name = "Velocity Publisher"
        self.window_size = (512, 250)
        self.cv_image = np.zeros((self.window_size[0], self.window_size[1], 3), np.uint8)
        cv2.imshow(self.window_name, self.cv_image)
        cv2.waitKey(25)
        self.x_y = None

	timer_period = 0.25  # seconds
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        cv2.setMouseCallback(self.window_name, self.get_mouse_click)
        print("Publisher ready")

    def timer_callback(self):
        if self.x_y is not None:
            cmd_velocity = Twist()
            cmd_velocity.linear.y = 0.0
            velocity = 0.75

            if self.x_y[1] < self.window_size[0] / 2:
                cmd_velocity.linear.x = velocity  # Move up
            else:
                cmd_velocity.linear.x = -velocity  # Move down

            self.publisher_.publish(cmd_velocity)
            self.get_logger().info(f"Publishing velocity: {cmd_velocity.linear.x}")
            
            # Clear the previous square
            self.cv_image = np.zeros((self.window_size[0], self.window_size[1], 3), np.uint8)

            # Draw a purple square at the clicked position
            square_size = 40
            color = (180, 0, 180)  # Purple in BGR format
            cv2.rectangle(self.cv_image, (self.x_y[0] - square_size // 2, self.x_y[1] - square_size // 2),
                          (self.x_y[0] + square_size // 2, self.x_y[1] + square_size // 2), color, -1)
            
            cv2.imshow(self.window_name, self.cv_image)
            
        cv2.waitKey(25)

    def get_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x_y = (x, y)

def main(args=None):
    rclpy.init(args=args)
    new_publisher = NewPublisher()
    rclpy.spin(new_publisher)
    new_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
