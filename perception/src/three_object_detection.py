#!/usr/bin/env python3
"""
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This node launches three object detection for small_scout_1, small_excavator_1, small_hauler_1

It subscribes to two topics for every robot:
1. Left camera's image_raw
2. Disparity image (published by stereo_image_proc)

Using the image, the objects are detected, preprocessed to compute a list of objects indices, which are of interest
and then published in perception/Objects message
"""

from object_detection_cap_lib import *

g_scout_image = None
g_scout_disparity = None
g_excavator_image = None
g_excavator_disparity = None
g_hauler_image = None
g_hauler_disparity = None
g_team_number = 0

def getString(message):
    """
    Returns the string to be logged or printed
    """
    global g_team_number
    return "[PERCEPTION | THREE OBJECT DETECTION | TEAM NUMER: " + g_team_number+ "]: " + message

def scoutImageCallback(scout_image):
    rospy.loginfo_once(getString("Scout Image Callback Working"))
    global g_scout_image
    g_scout_image = g_bridge.imgmsg_to_cv2(scout_image, "bgr8")


def scoutDisparityCallback(scout_disparity):
    rospy.loginfo_once(getString("Scout Disparity Callback Working"))
    global g_scout_disparity
    g_scout_disparity = g_bridge.imgmsg_to_cv2(scout_disparity.image, "32FC1")


def excavatorImageCallback(excavator_image):
    rospy.loginfo_once(getString("Excavator Image Callback Working"))
    global g_excavator_image
    g_excavator_image = g_bridge.imgmsg_to_cv2(excavator_image, "bgr8")


def excavatorDisparityCallback(excavator_disparity):
    rospy.loginfo_once(getString("Excavator Disparity Callback Working"))
    global g_excavator_disparity
    g_excavator_disparity = g_bridge.imgmsg_to_cv2(excavator_disparity.image, "32FC1")


def haulerImageCallback(hauler_image):
    rospy.loginfo_once(getString("Hauler Image Callback Working"))
    global g_hauler_image
    g_hauler_image = g_bridge.imgmsg_to_cv2(hauler_image, "bgr8")


def haulerDisparityCallback(hauler_disparity):
    rospy.loginfo_once(getString("Hauler Disparity Callback Working"))
    global g_hauler_disparity
    g_hauler_disparity = g_bridge.imgmsg_to_cv2(hauler_disparity.image, "32FC1")


def initObjectDetection(path_to_model, path_to_label_map):
    """
    Initialize the publishers, subscribers, and reads the tensorflow model
    """
    update_rate = rospy.Rate(UPDATE_HZ)

    # Initialize subscriber for scout, excavator and hauler (image and disparity)
    scout_image_sub_left = rospy.Subscriber(
        "/" + scout_name + "/camera/left/image_raw", Image, scoutImageCallback
    )
    scout_disp_sub = rospy.Subscriber(
        "/" + scout_name + "/camera/disparity", DisparityImage, scoutDisparityCallback
    )
    excavator_image_sub_left = rospy.Subscriber(
        "/" + excavator_name + "/camera/left/image_raw", Image, excavatorImageCallback
    )
    excavator_disp_sub = rospy.Subscriber(
        "/" + excavator_name + "/camera/disparity",
        DisparityImage,
        excavatorDisparityCallback,
    )
    hauler_image_sub_left = rospy.Subscriber(
        "/" + hauler_name + "/camera/left/image_raw", Image, haulerImageCallback
    )
    hauler_disp_sub = rospy.Subscriber(
        "/" + hauler_name + "/camera/disparity", DisparityImage, haulerDisparityCallback
    )

    # initialize publisher for scout, excavator and hauler (image and disparity)
    scout_img_pub = rospy.Publisher(
        "/capricorn/" + scout_name + "/object_detection/image", Image, queue_size=10
    )
    scout_objects_pub = rospy.Publisher(
        "/capricorn/" + scout_name + "/object_detection/objects",
        ObjectArray,
        queue_size=10,
    )
    excavator_img_pub = rospy.Publisher(
        "/capricorn/" + excavator_name + "/object_detection/image", Image, queue_size=10
    )
    excavator_objects_pub = rospy.Publisher(
        "/capricorn/" + excavator_name + "/object_detection/objects",
        ObjectArray,
        queue_size=10,
    )
    hauler_img_pub = rospy.Publisher(
        "/capricorn/" + hauler_name + "/object_detection/image", Image, queue_size=10
    )
    hauler_objects_pub = rospy.Publisher(
        "/capricorn/" + hauler_name + "/object_detection/objects",
        ObjectArray,
        queue_size=10,
    )

    category_index = label_map_util.create_category_index_from_labelmap(
        str.format(path_to_label_map), use_display_name=True
    )

    tf.keras.backend.clear_session()
    model = tf.saved_model.load(str.format(path_to_model))
    model_fn = model.signatures["serving_default"]

    rospy.loginfo(getString("Registering loop callback for three object detection"))

    global g_scout_image
    global g_scout_disparity
    global g_excavator_image
    global g_excavator_disparity
    global g_hauler_image
    global g_hauler_disparity

    while not rospy.is_shutdown():
        if g_scout_disparity is not None and g_scout_image is not None:
            detectionAlgorithm(
                category_index,
                model_fn,
                g_scout_image,
                g_scout_disparity,
                scout_img_pub,
                scout_objects_pub,
                scout_name,
            )
        if g_excavator_disparity is not None and g_excavator_image is not None:
            detectionAlgorithm(
                category_index,
                model_fn,
                g_excavator_image,
                g_excavator_disparity,
                excavator_img_pub,
                excavator_objects_pub,
                excavator_name,
            )
        if g_hauler_disparity is not None and g_hauler_image is not None:
            detectionAlgorithm(
                category_index,
                model_fn,
                g_hauler_image,
                g_hauler_disparity,
                hauler_img_pub,
                hauler_objects_pub,
                hauler_name,
            )
        update_rate.sleep()


if __name__ == "__main__":
    path_to_model = sys.argv[1]
    path_to_label_map = sys.argv[2]
    g_team_number = sys.argv[3]
    scout_name = "small_scout_" + g_team_number
    excavator_name = "small_excavator_" + g_team_number
    hauler_name = "small_hauler_" + g_team_number
    rospy.init_node("first_three_object_detection", anonymous=True)
    initObjectDetection(path_to_model, path_to_label_map)
