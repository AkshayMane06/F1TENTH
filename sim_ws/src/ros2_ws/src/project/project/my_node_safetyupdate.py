#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.speed = 0.0
        # Create ROS subscribers and publishers.
        self.scan_subscription = self.create_subscription( # Subscribe to the scan topic
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom', # this is more reliable than /pf/pose/odom
            self.odom_callback,
            10
        )
        
        # Update the speed of the car
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 1000)
        self.teleop_publisher_ = self.create_publisher(AckermannDriveStamped, 'teleop', 1000)
        self.bool_publisher_ = self.create_publisher(Bool, 'emergency_breaking', 1000)

        # Define thresholds for slowing down and emergency braking
        self.slow_threshold = 2  # Distance to start slowing down (in meters)
        self.emergency_threshold = 1  # Distance to apply emergency braking (in meters)

    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate distance to obstacle
        min_distance = min(scan_msg.ranges)
        
        # Check if the obstacle is within the emergency braking threshold
        if min_distance < self.emergency_threshold:
            self.apply_emergency_brake()
        # Check if the obstacle is within the slowing down threshold
        elif min_distance < self.slow_threshold:
            self.slow_down()
        else:
            self.continue_driving()

    def apply_emergency_brake(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        self.get_logger().info("Emergency brake engaged at speed {}".format(self.speed)) # Output to Log
        self.publisher_.publish(drive_msg) # for autonomous control
        self.teleop_publisher_.publish(drive_msg) # for manual control

    def slow_down(self):
        # Reduce speed gradually
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.5  # Adjust the desired reduced speed as needed
        self.get_logger().info("Slowing down to {} m/s".format(drive_msg.drive.speed)) # Output to Log
        self.publisher_.publish(drive_msg) # for autonomous control
        self.teleop_publisher_.publish(drive_msg) # for manual control

    def continue_driving(self):
        # Continue driving at current speed
        pass  # No need to adjust speed

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

