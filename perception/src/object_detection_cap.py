#!/usr/bin/env python3
"""
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This ros node takes three command line arguments:
robot_name = small_excavator_2, etc.
absolute_path_to_model
absolute_path_to_labelmaptxt

It subscribes to two topics:
1. Left camera's image_raw
2. Disparity image (published by stereo_image_proc)

Using the image, the objects are detected, preprocessed to compute a list of objects indices, which are of interest
and then published in perception/Objects message
"""

from object_detection_cap_lib import *

g_disparity = None
g_image = None
g_robot_name = "small_scout_1"

def getString(message):
    """
    Returns the string to be logged or printed
    """
    global g_robot_name
    return "[PERCEPTION | OBJECT DETECTION CAP | " + g_robot_name +  "]: " + message

def detectionCallback(image, disparity):  
    """
    Camera image and disparity combined callback
    """  
    rospy.logwarn_once(getString("Callback Working"))

    global g_image
    global g_disparity

    g_lock.acquire()
    # set global image and disparity
    g_image = g_bridge.imgmsg_to_cv2(image, "bgr8")
    g_disparity = g_bridge.imgmsg_to_cv2(disparity.image, "32FC1")
    g_lock.release()

def initObjectDetection(path_to_model, path_to_label_map):
    """
    Initialize the publishers, subscribers, and reads the tensorflow model
    """
    update_rate = rospy.Rate(UPDATE_HZ)

    # Initialize subscriber
    image_sub_left = message_filters.Subscriber('/' + g_robot_name + '/camera/left/image_raw', Image)
    disp_sub = message_filters.Subscriber('/' + g_robot_name + '/camera/disparity', DisparityImage)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub_left, disp_sub], 1, 1, allow_headerless=True) 

    ts.registerCallback(detectionCallback)

    # global br
    # br = tf2.TransformBroadcaster()

    # initialize publisher
    img_pub = rospy.Publisher('/capricorn/'+g_robot_name+'/object_detection/image', Image, queue_size=10)
    objects_pub = rospy.Publisher('/capricorn/'+g_robot_name+'/object_detection/objects', ObjectArray, queue_size=10)

    category_index = label_map_util.create_category_index_from_labelmap(str.format(path_to_label_map), use_display_name=True)
    tf.keras.backend.clear_session()
    model = tf.saved_model.load(str.format(path_to_model))
    model_fn = model.signatures['serving_default']

    rospy.logwarn(getString("Registering loop callback for {}".format(g_robot_name)))

    while not rospy.is_shutdown():
        if(g_disparity is not None and g_image is not None):
            detectionAlgorithm(category_index, model_fn, g_image, g_disparity, img_pub, objects_pub, g_robot_name)
        update_rate.sleep()

if __name__ == '__main__':   
    g_robot_name = sys.argv[1]
    path_to_model = sys.argv[2]
    path_to_label_map = sys.argv[3]
    rospy.init_node(g_robot_name + '_object_detection', anonymous = True)    
    initObjectDetection(path_to_model, path_to_label_map)
