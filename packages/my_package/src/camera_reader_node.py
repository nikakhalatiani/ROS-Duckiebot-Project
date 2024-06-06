#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import time

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # construct publisher
        self._publisher = rospy.Publisher(f"/{self._vehicle_name}/camera_node/pixel_counts", Float64MultiArray, queue_size=1)


        # self.left_motor  = rospy.Publisher(f"/{self._vehicle_name}/left_wheel_encoder_node/tick", Float64, queue_size=1)
        # self.right_motor = rospy.Publisher(f"/{self._vehicle_name}/right_wheel_encoder_node/tick", Float64, queue_size=1)

        self.left_tick = 0
        self.right_tick = 0
        self.blue_threshold = 0.04  # Define a threshold for the blue count
        self.state_change_time = None  # Time when the state changed
        self.specific_command_duration = 4.1 # Duration for executing specific commands
        self.state = "normal"  # Current state of the robot
        self.state_start_time = None  # Time when the current state started
        self.state_index = 0  # Index to track the current step in the predefined sequence
        self.predefined_steps = [
            ("turn_left", 0.25),  # (action, duration in seconds)
            ("go_straight", 1),
            ("turn_right", 0.3),
            ("go_straight", 1),
            ("turn_right", 0.3),
            ("go_straight", 1),
            ("turn_left", 0.25)
        ]


    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        image = cv2.bilateralFilter(image, 12, 125, 155)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_hsl = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

        # Define color range for yellow
        yellow_lower_color = np.array([9, 129, 155])
        yellow_upper_color = np.array([83, 255, 255])
        # Define color range for white
        white_lower_color = np.array([0, 173, 0])
        white_upper_color = np.array([179, 255, 255])

        # Defind color range for blue
        blue_lower_color = np.array([113, 133, 0])
        blue_upper_color = np.array([179, 255, 255])

        # Create masks for yellow and white colors
        yellow_mask = cv2.inRange(img_hsv, yellow_lower_color, yellow_upper_color)
        white_mask = cv2.inRange(img_hsl, white_lower_color, white_upper_color)
        blue_mask = cv2.inRange(img_hsv, blue_lower_color, blue_upper_color)

        # Apply the masks to the original image to keep only yellow and white regions
        yellow_filtered = cv2.bitwise_and(image, image, mask=yellow_mask)
        white_filtered = cv2.bitwise_and(image, image, mask=white_mask)
        blue_filtered = cv2.bitwise_and(image, image, mask=blue_mask)

        # Dilate Yellow
        kernel = np.ones((7, 7), np.uint8)
        yellow_filtered = cv2.dilate(yellow_filtered, kernel, iterations=2)
        
        # Crop the image to the lower half
        yellow_filtered[:240, :] = 0
        white_filtered[:240, :] = 0
        white_filtered[:, :200] = 0
        blue_filtered[:140, :] = 0
        blue_filtered[180:, ] = 0
        
        # Combine the yellow and white images
        white_and_yellow = cv2.bitwise_or(yellow_filtered, white_filtered)
        combined = cv2.bitwise_or(white_and_yellow, blue_filtered)
     

        white_color_count = np.count_nonzero(white_filtered)
        yellow_color_count = np.count_nonzero(yellow_filtered)
        blue_color_count = np.count_nonzero(blue_filtered)

        # Normilize the pixel count
        white_color_count = white_color_count / (640 * 480)
        yellow_color_count = yellow_color_count / (640 * 480)
        blue_color_count = blue_color_count / (640 * 480)

        if blue_color_count > self.blue_threshold:
            print("Blue detected in the center region, triggering state change")
            self.state_change_time = time.time()
            self.state = "predefined_steps"
            self.state_index = 0
            self.state_start_time = time.time()

        # Check if in specific command execution state
        if self.state == "predefined_steps":
            current_time = time.time()
            if current_time - self.state_start_time > self.predefined_steps[self.state_index][1]:
                self.state_index += 1
                self.state_start_time = current_time
                if self.state_index >= len(self.predefined_steps):
                    self.state = "normal"

            if self.state == "predefined_steps":
                action = self.predefined_steps[self.state_index][0]
                if action == "turn_left":
                    self.left_tick = -0.2
                    self.right_tick = 0.2
                elif action == "go_straight":
                    self.left_tick = 0.5
                    self.right_tick = 0.5
                elif action == "turn_right":
                    self.left_tick = 0.2
                    self.right_tick = -0.2
        else:
            # Convert to signal
            left_motor = 0.3+ 0.2 * (0.2 * white_color_count)
            right_motor = 0.3 + 0.2* (0.2 * yellow_color_count)

            if white_color_count > yellow_color_count:
                print("White is more")
                if left_motor > 0.2:
                    left_motor = left_motor - 0.2
                else:
                    left_motor = left_motor
                right_motor += 0.25
                
            if yellow_color_count > white_color_count:
                print("Yellow is more")
                if right_motor > 0.2:
                    right_motor = right_motor - 0.2
                else:
                    right_motor = right_motor
                left_motor += 0.25

            self.left_tick = left_motor
            self.right_tick = right_motor


        # bart's code
        # if white_color_count > yellow_color_count:
        #     print("White is more")
        #     if left_motor > 0.25:
        #         left_motor = left_motor - 0.2
        #     else:
        #         left_motor = left_motor
        #     right_motor += 0.05
            
        # if yellow_color_count > white_color_count:
        #     print("Yellow is more")
        #     if right_motor > 0.2:
        #         right_motor = right_motor - 0.25
        #     else:
        #         right_motor = right_motor
        #     left_motor += 0.05


        # self.left_tick = 0
        # self.right_tick = 0

        cv2.imshow("Combined", combined)
        cv2.waitKey(1)

        

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Create a Float64MultiArray message to publish the values
            pixel_counts = Float64MultiArray()
            pixel_counts.data = [self.left_tick, self.right_tick]

            # Publish the pixel counts
            self._publisher.publish(pixel_counts)
            
            rate.sleep()

    def nothing(self):
        pass

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # start the run loop
    node.run()
    # keep spinning
    rospy.spin()