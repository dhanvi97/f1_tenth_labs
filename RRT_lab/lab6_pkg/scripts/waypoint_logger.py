#!/usr/bin/env python
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
import atexit
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from os.path import expanduser

home = expanduser("~")
file = open(home + "/f1tenth_ws/src/lab6_pkg/data/waypoints.txt", "w")

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')
        self.waypoints = None
        self.odom_sub = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def odom_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.waypoints.append([x, y, yaw])
    
    def timer_callback(self):
        global file
        if self.waypoints is not None:
            file.write(str(self.waypoints[-1][0]) + "," + str(self.waypoints[-1][1]) + "," + str(self.waypoints[-1][2]) + "\n")
        self.get_logger().info("Logging waypoints...")
    
    def exit_handler(self):
        global file
        file.close()
        self.get_logger().info("Exiting...")

if __name__ == '__main__':
    rclpy.init()
    node = WaypointLogger()
    node.waypoints = []
    atexit.register(node.exit_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass