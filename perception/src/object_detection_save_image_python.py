#! /usr/bin/env python3
import rospy
import numpy as np
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

prefix = "mb_"
num_img = 000

cv_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
dataset_path = "./"
bridge = CvBridge()

def img_callback(image):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(image,  "bgr8")
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

def save_image():
    global robot_name
    global num_img
    
    rospy.Subscriber('/'+ robot_name + '/camera/left/image_raw', Image, img_callback)

    while not rospy.is_shutdown():
        input("Press 'Enter' to save")
        number_str = str(num_img)
        num_img += 1
        zero_filled_number = number_str.zfill(5)
        image_name = prefix + zero_filled_number
        cv2.imwrite(dataset_path + image_name +'.jpg', cv_image)
        print("Imade Saved " + image_name)
    
    rospy.spin()

if __name__=='__main__':
    global robot_name
    robot_name = sys.argv[1]
    rospy.init_node(robot_name + 'object_detection_save_image', anonymous = True)
    save_image()
    