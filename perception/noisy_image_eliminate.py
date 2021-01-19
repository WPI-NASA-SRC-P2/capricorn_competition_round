#!/usr/bin/env python

'''
WPI Team Capricorn
NASA Space Robotics Challenge Phase 2

Author: Mahimana Bhatt
Team Endurance

This code reads stereo data from the topic, removes noisy images for the video stream and published it on new topics:
/capricorn/left
/capricorn/right
 
Algorithm: This algorithm first extracts edges from the image with high threshold, this operation leaves only noise and 
some terrain edges in a binary form, we calculate the sum of the whole image, in a noisy image the white pixels will be 
way greater than a non-noisy image, hence, we calculate the difference between adjacent edge image sums, and if the 
difference is greater than a threshold, the image is considered as noisy and we don't update the last sum for the noisy 
image to avoid two adjacent noisy images to pass through the filter.

Improvements:
1. When the base station enters into the image, the number of white pixels in edge image increases which causes
a delay to consider as a normal image, this needs improvement.
2. The threshold of the image can be made adaptive to increase the elimination of noisy images.
'''

#TODO: Convert code in CPP for optimization
#TODO: do both of the left and right check in two separate functions

# Computer Vision Libraries
from cv2 import cv2
import numpy as np

import sys
import logging
# ROS Libraries
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

class noisy_image_eliminate:
    bridge = CvBridge()
    right_last_sum = 0
    left_last_sum = 0
    right_actual_sum = 0
    left_actual_sum = 0
    threshold = 4000
    upper_threshold = 1.3
    lower_threshold = 0.7

    def check_left(self, left):
        left_image = self.bridge.imgmsg_to_cv2(left,desired_encoding='passthrough')
        left_edge = cv2.Canny(left_image, 250, 255)

        left_sum = int(np.sum(left_edge))

        left_diff = left_sum-self.left_last_sum

        if(left_sum >= self.left_actual_sum * self.lower_threshold and left_sum <= self.left_actual_sum * self.upper_threshold):
            # print("Corrected")
            self.left_last_sum = left_sum

        self.left_actual_sum = left_sum

        # For debugging
        # print("Left sum: ", left_sum,"diff: ", left_diff)

        if(left_diff < self.threshold):
            self.left_last_sum = left_sum
            return True
        else:
            return False

    def check_right(self, right):
        right_image = self.bridge.imgmsg_to_cv2(right,desired_encoding='passthrough')
        right_edge = cv2.Canny(right_image, 250, 255)

        right_sum = int(np.sum(right_edge))

        right_diff = right_sum-self.right_last_sum

        if(right_sum >= self.right_actual_sum * self.lower_threshold and right_sum <= self.right_actual_sum * self.upper_threshold):
            # print("Corrected")
            self.right_last_sum = right_sum

        self.right_actual_sum = right_sum

        # For debugging
        # print("Right sum: ", right_sum,"diff: ", right_diff)

        if(right_diff < self.threshold):
            self.right_last_sum = right_sum
            return True
        else:
            return False


    def loop(self,right, right_info, left, left_info):
        if(self.check_left(left)):
            self.left_image_pub.publish(left)  
            self.left_info_pub.publish(left_info)    

        if(self.check_right(right)):
            self.right_image_pub.publish(right)  
            self.right_info_pub.publish(right_info)

    def start(self):
        robot = sys.argv[1] + "_" + sys.argv[2]
        logging.debug("Starting Noisy Image Elimination: ")
        rospy.init_node('noisy_image_leliminate', anonymous = False)

        image_sub_right = message_filters.Subscriber('/'+robot+'/camera/right/image_raw', Image)
        image_sub_left = message_filters.Subscriber('/'+robot+'/camera/left/image_raw', Image)
        info_sub_right = message_filters.Subscriber('/'+robot+'/camera/right/camera_info', CameraInfo)
        info_sub_left = message_filters.Subscriber('/'+robot+'/camera/left/camera_info', CameraInfo)

        self.left_image_pub = rospy.Publisher('/capricorn/'+robot+'/camera/left/image_raw', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/capricorn/'+robot+'/camera/right/image_raw', Image, queue_size=10)
        self.left_info_pub = rospy.Publisher('/capricorn/'+robot+'/camera/left/camera_info', CameraInfo, queue_size=10)
        self.right_info_pub = rospy.Publisher('/capricorn/'+robot+'/camera/right/camera_info', CameraInfo, queue_size=10)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub_right, info_sub_right, image_sub_left, info_sub_left], 10, 0.5, allow_headerless=False)
        ts.registerCallback(self.loop)
        rospy.spin()

if __name__=='__main__':
    if len(rospy.myargv(argv=sys.argv)) < 3:
        print("Usage: rosrun noisy_image_eliminate.py <robot name> <robot number>")
        sys.exit()

    n = noisy_image_eliminate()
    n.start()
    