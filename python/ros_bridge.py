#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import json
from snn_model import SNNModel


class ROSBridge:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('snn_ros_bridge')
        
        # Load SNN model with configuration file
        self.snn = SNNModel('config/snn_config.json')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to LiDAR scan topic
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        # Process LiDAR data with SNN
        ranges = msg.ranges
        
        # Preprocess: replace inf with max range
        max_range = msg.range_max
        processed_ranges = [r if r < max_range else max_range for r in ranges]
        
        # Pass processed ranges through the SNN model
        output = self.snn.process(processed_ranges)
        
        # Convert output to robot commands (example: first two outputs are linear and angular)
        cmd = Twist()
        cmd.linear.x = output[0] * 0.5  # Scale as needed
        cmd.angular.z = output[1] * 0.5
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def run(self):
        # Keep node running
        rospy.spin()


if __name__ == "__main__":
    bridge = ROSBridge()
    bridge.run()
