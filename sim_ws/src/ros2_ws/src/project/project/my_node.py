#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        # Set PID gains               #5 0.09 0.01  worked 
        self.kp = 5     #0.5
        self.kd = 0.09     #0.1 
        self.ki = 0.01       #0.01

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # Store necessary values
        self.desired_distance = 1.8  # Adjust as needed og 1.0  1.8

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
        """
        index = int((angle - range_data.angle_min) / range_data.angle_increment)
        if np.isinf(range_data.ranges[index]) or np.isnan(range_data.ranges[index]):
            return 0.0
        else:
            return range_data.ranges[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()
        """
        left_angle = -np.pi / 4  # Adjust as needed  AKSHAY here it was 4 
        right_angle = np.pi / 4  # Adjust as needed

        left_dist = self.get_range(range_data, left_angle)
        right_dist = self.get_range(range_data, right_angle)
        #self.get_logger().info("range {}".format(range_data.ranges)) # Output to Log
        error =(- (left_dist + (dist * (np.sin(left_angle))))) #AKSHAY SHOULD write LSin theta here  and minus sign
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control
        """
        angle = (self.kp * error + self.kd * (error - self.prev_error) + self.ki * self.integral) 
        
        # Clamp the steering angle within permissible range
        max_steering_angle = np.pi / 4
        angle = max(-max_steering_angle, min(max_steering_angle, angle)) 
        self.get_logger().info("angle {}".format(angle)) # Output to Log
        self.get_logger().info("error {}".format(error)) # Output to Log
        #self.get_logger().info("range {}".format(range_data.ranges[0])) # Output to Log
        #self.get_logger().info("integral {}".format(self.integral)) # Output to Log
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        """
        self.error = self.get_error(msg, self.desired_distance)

        # Integrate the error for Ki term
        self.integral += self.error

        # Perform PID control
        #--velocity = 1.0  # Adjust as needed
        velocity = 1.0
        self.pid_control(self.error, velocity)

        # Update prev_error for next iteration
        self.prev_error = self.error

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
