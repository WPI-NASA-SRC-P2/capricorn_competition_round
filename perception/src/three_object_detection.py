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
import rospy
import sys
import message_filters
import threading
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

from perception.msg import Object
from perception.msg import ObjectArray

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' 

import tensorflow as tf
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

import cv2
import numpy as np

physical_devices = tf.config.list_physical_devices('GPU') 
tf.config.experimental.set_memory_growth(physical_devices[0], True)
tf.config.experimental.set_virtual_device_configuration(physical_devices[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=800)])

K = [381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0]
Fx = K[0]
Fy = K[4]
Cx = K[2]
Cy = K[5]

STEREO_BASELINE = Cx / (2 * Fx)
HEIGHT = 480
WIDTH = 640
UPDATE_HZ = 10.0
CAMERA_FRAME_LINK = "_left_camera_optical"
CLASS_SCORE_THRESHOLD = 0.6

g_bridge = CvBridge()
g_seq = 0
g_lock = threading.Lock()
g_scout_image = None
g_excavator_image = None
g_hauler_image = None
g_scout_disparity = None

g_class_individual_thresh = {"processingPlant" : 0.8, "repairStation" : 0.8, "hopper" : 0.6, "excavator" : 0.7, "scout" : 0.7}
g_class_not_to_be_duplicated = {"processingPlant", "repairStation", "hopper", "furnace", "excavatorArm"}

def preProcessObjectDetection(scores, classes, num_detections, final_list) -> None:
    """
    Preprocesses the detected object i.e. it outputs the list of indices of only those objects which have a confidence of higher than a default threshold
    Also, if there is a custom threshold defined for the class in g_class_individual_thresh, that threshold is also check, if the confidence is higher than
    both the threshold, then we check if there is already an object of same label detected (done to remove certain duplicate object because some objects will occur only 
    once in an image as there is no copy such as processingPlant, so if there are two copies of processingPlant, then the copy with higher confidence will be published), 
    if the class does not belong to g_class_not_to_be_duplicated, it is simply added to the output list 
    """

    # maintains class labels which are added to the result
    current_class = set()
    for i in range (num_detections):
        if(scores[i] > CLASS_SCORE_THRESHOLD):
            # score greater than a default threshold
            class_label = (g_category_index.get(classes[i])).get('name')

            if(class_label in g_class_individual_thresh and scores[i] < g_class_individual_thresh[class_label]):
                # if individual threshold is defined and the current confidence is not greater than individual threshold
                continue            

            if(class_label in g_class_not_to_be_duplicated and class_label in current_class):
                # if there is already an element added to the output of a specific class which should not be duplicated
                # input scores are sorted high to low, so if there is already a label added to output, there should not another label added
                continue
            else:
                # add index to the output and buffer list maintained to keep track of added labels to output
                current_class.add(class_label)
                final_list.append(i)
        else:
            break

def estimate3dLocation(i, box, cl, score, disp_img):
    """
    Estimates the 3D attributes of a detected object using the disparity image and bounding box
    """
    # calculating higher and lower limits of bounding box in pixels
    l_y = box[0] * HEIGHT
    l_x = box[1] * WIDTH
    h_y = box[2] * HEIGHT
    h_x = box[3] * WIDTH

    # center of bounding box in pixels
    center_x = int(l_x + (h_x - l_x)/2)
    center_y = int(l_y + (h_y - l_y)/2)

    low_x = int(l_x)
    high_x = int(h_x)

    # disparity of center pixel
    min_disp = disp_img[center_y, center_x]

    # calculating the disparity of the pixel which is farthest to the robot in a window of 10 pixels in height and width of full bounding box
    if(center_y - 10 > 10):
        low_y =  int(center_y - 10)
    else:
        low_y = int(center_y)
    
    if(center_y + 10 < (HEIGHT - 1)):
        high_y = int(center_y + 10)
    else:
        high_y = int(center_y)
        
    for w in range (low_x, high_x):
        for v in range (low_y, high_y):
            if(disp_img[v, w] > min_disp):
                min_disp = disp_img[v, w]

    # calculating 3d location from the estimated disparity and camera's intrinsic parameters
    center_disparity = min_disp
    depth = STEREO_BASELINE * Fx / center_disparity
    x = (center_x - Cx) * depth / Fx
    y = (center_y - Cy) * depth / Fy
    z = depth

    # calculating the width of the output object
    w_x_1 = (l_x - Cx) * depth / Fx
    w_y_1 = (center_y - Cy) * depth / Fy
    w_z_1 = depth

    w_x_2 = (h_x - Cx) * depth / Fx
    w_y_2 = (center_y - Cy) * depth / Fy
    w_z_2 = depth

    # width of the output object
    w = math.sqrt((w_x_1 - w_x_2)**2+(w_y_1 - w_y_2)**2+(w_z_1 - w_z_2)**2)

    result = {
        "score" : score, 
        "class" : cl,
        "x" : x, 
        "y" : y,
        "z" : z,
        "w" : w,
        "s_x" : (h_x - l_x),
        "s_y" : (h_y - l_y),
        "c_x" : center_x,
        "c_y" : center_y,
    }
    return result

def constructObjectMsg(result, robot_name):
    """
    Construct the geometry_msgs/PoseStamped message for single object
    """
    object_msg = Object()
    object_msg.score = result["score"]
    object_msg.label = (g_category_index.get(result["class"])).get('name')
    object_msg.point.header.seq = g_seq   
    object_msg.point.header.frame_id = robot_name + CAMERA_FRAME_LINK
    object_msg.point.pose.position.x = result["x"]
    object_msg.point.pose.position.y = result["y"]
    object_msg.point.pose.position.z = result["z"]
    object_msg.width = result["w"]
    object_msg.center.x = result["c_x"]
    object_msg.center.y = result["c_y"]
    object_msg.size_x = result["s_x"]
    object_msg.size_y = result["s_y"]
    return object_msg

def overlayInfoImage(result, image_np):
    """
    Overlay the 3D location of the object on the image, the white colored text having x, y, z values
    """
    WHITE = (255,255,255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.28
    font_color = WHITE
    font_thickness = 0
    text = "x: {:.3f}, y: {:.3f}, z: {:.3f}" .format(result["x"], result["y"], result["z"])
    cv2.putText(image_np, text, (result["c_x"], result["c_y"]), font, font_size, font_color, font_thickness, cv2.LINE_AA)

def detectionAlgorithm(g_image, g_disparity, img_pub, disp_pub, robot_name):
    rospy.loginfo_once("Object Detection Algorithm Working")

    # getting buffered global image and disparity from callback
    image_np = g_image.copy()
    disp_img = g_disparity.copy()

    # convert images to tensor
    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis,...]

    # run object detection on input image
    output_dict = g_model_fn(input_tensor)
    rospy.logwarn_once("Started Object Detection Finally")

    # processing object detected data
    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key:value[0, :num_detections].numpy() 
                    for key,value in output_dict.items()}
    output_dict['num_detections'] = num_detections

    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

    # adding visualization of boxes and score on the existing image, published to view on rviz
    vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      output_dict['detection_boxes'],
      output_dict['detection_classes'],
      output_dict['detection_scores'],
      g_category_index,
      use_normalized_coordinates=True,
      line_thickness=2,
      min_score_thresh = CLASS_SCORE_THRESHOLD)      

    boxes = output_dict['detection_boxes']
    scores = output_dict['detection_scores']
    classes = output_dict['detection_classes']
    num_detections = output_dict['num_detections']

    # preprocess the objects
    final_list = []
    preProcessObjectDetection(scores, classes, num_detections, final_list)

    # create output perception/Objects message
    objects_msg = ObjectArray()

    global g_seq
    objects_msg.header.seq = g_seq   
    g_seq += 1
    objects_msg.header.frame_id = robot_name + CAMERA_FRAME_LINK

    for i in final_list:
        # estimate 3D location of object
        result = estimate3dLocation(i, boxes[i], classes[i], scores[i], disp_img)
        overlayInfoImage(result, image_np)
        # add to output message
        objects_msg.obj.append(constructObjectMsg(result, robot_name))

    objects_msg.number_of_objects = len(objects_msg.obj)   
    msg = g_bridge.cv2_to_imgmsg(image_np)
    img_pub.publish(msg)
    disp_pub.publish(objects_msg)

def detectionCallback(scout_image, scout_disparity, excavator_image, excavator_disparity, hauler_image, hauler_disparity):  
    """
    Camera image and disparity combined callback
    """  
    rospy.logwarn_once("Callback Working")

    global g_scout_image
    global g_scout_disparity
    global g_excavator_image
    global g_excavator_disparity
    global g_hauler_image
    global g_hauler_disparity

    # set global image and disparity
    g_scout_image = g_bridge.imgmsg_to_cv2(scout_image, "bgr8")
    g_excavator_image = g_bridge.imgmsg_to_cv2(excavator_image, "bgr8")
    g_hauler_image = g_bridge.imgmsg_to_cv2(hauler_image, "bgr8")
    g_scout_disparity = g_bridge.imgmsg_to_cv2(scout_disparity.image, "32FC1")
    g_excavator_disparity = g_bridge.imgmsg_to_cv2(excavator_disparity.image, "32FC1")
    g_hauler_disparity = g_bridge.imgmsg_to_cv2(hauler_disparity.image, "32FC1")

def initObjectDetection(path_to_model, path_to_label_map):
    """
    Initialize the publishers, subscribers, and reads the tensorflow model
    """
    update_rate = rospy.Rate(UPDATE_HZ)

    # Initialize subscriber for scout, excavator and hauler (image and disparity)
    scout_image_sub_left = message_filters.Subscriber('/'+ scout_name + '/camera/left/image_raw', Image)
    scout_disp_sub = message_filters.Subscriber('/' + scout_name + '/camera/disparity', DisparityImage)
    excavator_image_sub_left = message_filters.Subscriber('/'+ excavator_name + '/camera/left/image_raw', Image)
    excavator_disp_sub = message_filters.Subscriber('/' + excavator_name + '/camera/disparity', DisparityImage)
    hauler_image_sub_left = message_filters.Subscriber('/'+ hauler_name + '/camera/left/image_raw', Image)
    hauler_disp_sub = message_filters.Subscriber('/' + hauler_name + '/camera/disparity', DisparityImage)
    ts = message_filters.ApproximateTimeSynchronizer([scout_image_sub_left, scout_disp_sub, excavator_image_sub_left, excavator_disp_sub, hauler_image_sub_left, hauler_disp_sub], 1, 1, allow_headerless=True) 

    ts.registerCallback(detectionCallback)

    # initialize publisher for scout, excavator and hauler (image and disparity)
    scout_img_pub = rospy.Publisher('/capricorn/' + scout_name + '/object_detection/image', Image, queue_size=10)
    scout_objects_pub = rospy.Publisher('/capricorn/' + scout_name + '/object_detection/objects', ObjectArray, queue_size=10)
    excavator_img_pub = rospy.Publisher('/capricorn/' + excavator_name + '/object_detection/image', Image, queue_size=10)
    excavator_objects_pub = rospy.Publisher('/capricorn/' + excavator_name + '/object_detection/objects', ObjectArray, queue_size=10)
    hauler_img_pub = rospy.Publisher('/capricorn/' + hauler_name + '/object_detection/image', Image, queue_size=10)
    hauler_objects_pub = rospy.Publisher('/capricorn/' + hauler_name + '/object_detection/objects', ObjectArray, queue_size=10)

    # read object detection model
    global g_model_fn
    tf.keras.backend.clear_session()
    model = tf.saved_model.load(str.format(path_to_model))
    g_model_fn = model.signatures['serving_default']

    # initialize category index to convert integer indices to class labels
    global g_category_index
    g_category_index = label_map_util.create_category_index_from_labelmap(str.format(path_to_label_map), use_display_name=True)
    rospy.logwarn("Registering loop callback for three object detection")     

    while not rospy.is_shutdown():
        if(g_scout_disparity is not None and g_scout_image is not None and g_excavator_disparity is not None and g_excavator_image is not None and g_hauler_disparity is not None and g_hauler_image is not None):
            detectionAlgorithm(g_scout_image, g_scout_disparity, scout_img_pub, scout_objects_pub, scout_name)
            detectionAlgorithm(g_excavator_image, g_excavator_disparity, excavator_img_pub, excavator_objects_pub, excavator_name)
            detectionAlgorithm(g_hauler_image, g_hauler_disparity, hauler_img_pub, hauler_objects_pub, hauler_name)
        update_rate.sleep()

if __name__ == '__main__':   
    path_to_model = sys.argv[1]
    path_to_label_map = sys.argv[2]
    team_number = sys.argv[3]
    scout_name = "small_scout_" + team_number
    excavator_name = "small_excavator_" + team_number
    hauler_name = "small_hauler_" + team_number
    rospy.init_node('first_three_object_detection', anonymous = True)    
    initObjectDetection(path_to_model, path_to_label_map)
