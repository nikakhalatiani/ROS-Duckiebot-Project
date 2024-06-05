#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray

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
        self._publisher = rospy.Publisher(f"/{self._vehicle_name}/camera_node/pixel_counts", Float64MultiArray, queue_size=10)

        # Initialize attributes to store pixel counts
        self.yellow_pixel_count = 0
        self.white_pixel_count = 0

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color range for yellow
        yellow_lower_color = np.array([9, 129, 155])
        yellow_upper_color = np.array([83, 255, 255])
        # Define color range for white
        white_lower_color = np.array([0, 0, 185])
        white_upper_color = np.array([179, 70, 255])

        # Create masks for yellow and white colors
        yellow_mask = cv2.inRange(img_hsv, yellow_lower_color, yellow_upper_color)
        white_mask = cv2.inRange(img_hsv, white_lower_color, white_upper_color)

        # Apply the masks to the original image to keep only yellow and white regions
        yellow_filtered = cv2.bitwise_and(image, image, mask=yellow_mask)
        white_filtered = cv2.bitwise_and(image, image, mask=white_mask)

        # Convert the filtered images to grayscale
        yellow_gray = cv2.cvtColor(yellow_filtered, cv2.COLOR_BGR2GRAY)
        white_gray = cv2.cvtColor(white_filtered, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale images
        yellow_blur = cv2.GaussianBlur(yellow_gray, (5, 5), 0)
        white_blur = cv2.GaussianBlur(white_gray, (5, 5), 0)

        # Apply the Sobel operator to the blurred images
        sobelx_yellow = cv2.Sobel(yellow_blur, cv2.CV_64F, 1, 0)
        sobely_yellow = cv2.Sobel(yellow_blur, cv2.CV_64F, 0, 1)
        sobelx_white = cv2.Sobel(white_blur, cv2.CV_64F, 1, 0)
        sobely_white = cv2.Sobel(white_blur, cv2.CV_64F, 0, 1)

        # Compute the magnitude of the gradients
        Gmag_yellow = np.sqrt(sobelx_yellow**2 + sobely_yellow**2)
        Gmag_white = np.sqrt(sobelx_white**2 + sobely_white**2)

        # Threshold the gradient magnitude
        threshold = 95
        mask_mag_yellow = (Gmag_yellow > threshold)
        mask_mag_white = (Gmag_white > threshold)

        # Combine masks with the original image to keep only yellow on the left and white on the right
        height, width = image.shape[:2]
        mask_left = np.zeros((height, width), dtype=bool)
        mask_left[:, :width//2] = True
        mask_right = np.zeros((height, width), dtype=bool)
        mask_right[:, width//2:] = True

        yellow_edge_mask = mask_mag_yellow & mask_left
        white_edge_mask = mask_mag_white & mask_right

        # Apply the edge masks to the original image
        yellow_edges = np.uint8(yellow_edge_mask * 255)
        white_edges = np.uint8(white_edge_mask * 255)

        combined_edges = cv2.bitwise_or(yellow_edges, white_edges)
        final_image = cv2.bitwise_and(image, image, mask=combined_edges)

        # Make the upper half of the final image black
        final_image[:height//2, :] = 0

        # Count and print the amount of yellow and white pixels
        self.yellow_pixel_count = np.count_nonzero(yellow_edges)
        self.white_pixel_count = np.count_nonzero(white_edges)

        # Yellow color scale to white
        mul_yellow = 2
        self.yellow_pixel_count = self.yellow_pixel_count * mul_yellow

        # Scale the pixel count to float values between 0 and 1
        self.yellow_pixel_count = self.yellow_pixel_count / (height * width)
        self.white_pixel_count = self.white_pixel_count / (height * width)

        # Display the results
        cv2.imshow(self._window, final_image)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Create a Float64MultiArray message to publish the values
            pixel_counts = Float64MultiArray()
            pixel_counts.data = [self.yellow_pixel_count, self.white_pixel_count]

            # Publish the pixel counts
            self._publisher.publish(pixel_counts)
            # sabaaa
            # saba2
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # start the run loop
    node.run()
    # keep spinning
    rospy.spin()