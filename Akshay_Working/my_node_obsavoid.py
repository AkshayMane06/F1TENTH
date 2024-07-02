#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    AKSHAY MANE : This will follow the gap
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish to drive
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Initialize steering angle
        self.steering_angle = 0.0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """
        # Example: Rejecting high values (> 3m)
        proc_ranges = [r if r < 3.0 else 0.0 for r in ranges]
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # Find the max gap in the ranges
        max_length = 0
        max_start = 0
        max_end = 0
        start_idx = 0
        for i, r in enumerate(free_space_ranges):
            if r == 0:
                if start_idx != 0:
                    length = i - start_idx
                    if length > max_length:
                        max_length = length
                        max_start = start_idx
                        max_end = i
                    start_idx = 0
            else:
                if start_idx == 0:
                    start_idx = i

        return max_start, max_end

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # Find the furthest point within the gap
        max_range = 0
        best_point_idx = ranges.index(min(ranges))

        # for i in range(start_i, end_i):
        #     if ranges[i] > max_range:
        #         max_range = ranges[i]
        #         best_point_idx = i

        return best_point_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        self.steering_angle = 0.0
        # Find closest point to LiDAR
        closest_point_idx = np.argmin(proc_ranges)
        #self.get_logger().info("ranges {}".format(len(ranges))) # Output to Log
        
        
        # Eliminate all points inside 'bubble' (set them to zero)
        # bubble_radius = 0.2
        # bubble_indices = range(closest_point_idx - int(bubble_radius / data.angle_increment),
        #                        closest_point_idx + int(bubble_radius / data.angle_increment))
        # for i in bubble_indices:
        #     if 0 <= i < len(proc_ranges):
        #         proc_ranges[i] = 0.0

        # Find max length gap
        start_idx, end_idx = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_point_idx = self.find_best_point(start_idx, end_idx, ranges)
        #self.get_logger().info(" {}".format()) # Output to Log
        # Update steering angle based on gap position
        #self.get_logger().info("proc ranges {}".format(max(proc_ranges))) # Output to Log
       # if (max(proc_ranges) <  2.9) :
            #self.get_logger().info("looping")
        if best_point_idx < 360 :
            self.steering_angle = 0.2  # Turn right if gap is on the left
        elif best_point_idx > 720:
            self.steering_angle = - 0.2  # Turn left if gap is on the right
        else:
	        self.steering_angle = 0.0
        #else:
         #   self.steering_angle = 0.0    

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = 1.5  # Example speed
        #self.get_logger().info("angle {}".format(self.steering_angle)) # Output to Log
        self.publisher_.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

