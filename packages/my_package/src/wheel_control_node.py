#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64MultiArray

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        pixel_counts_topic = f"/{vehicle_name}/camera_node/pixel_counts"
        # initial wheel velocities
        self._vel_left = 0.0
        self._vel_right = 0.0
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        # construct subscriber to pixel counts
        self._subscriber = rospy.Subscriber(pixel_counts_topic, Float64MultiArray, self.pixel_counts_callback)
        self.yellow_pixel_count = 0.0
        self.white_pixel_count = 0.0

    def pixel_counts_callback(self, data):
        # read the pixel counts from the message
        self.yellow_pixel_count = data.data[0]
        self.white_pixel_count = data.data[1]
        rospy.loginfo(f"Received yellow pixel count: {self.yellow_pixel_count}")
        rospy.loginfo(f"Received white pixel count: {self.white_pixel_count}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Adjust wheel velocities based on the pixel counts
            # Example logic: if more yellow pixels, turn right; if more white pixels, turn left
            self._vel_left = self.yellow_pixel_count
            self._vel_right = self.white_pixel_count

            # Create the message
            message = WheelsCmdStamped()
            message.vel_left = self._vel_left
            message.vel_right = self._vel_right

            # Publish the wheel velocities
            self._publisher.publish(message)

            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped()
        stop.vel_left = 0
        stop.vel_right = 0
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # keep the process from terminating
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()