#!/usr/bin/env python3
import rospy
import sys
import message_filters
import math
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from perception.msg import Object
from perception.msg import Objects

import tensorflow as tf
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

import cv2
import numpy as np

bridge = CvBridge()
K = [381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0]
fx = K[0]
fy = K[4]
cx = K[2]
cy = K[5]

height = 480
width = 640

def detection_callback(image, disparity):
    image_np = bridge.imgmsg_to_cv2(image,  "bgr8")
    disp_img = bridge.imgmsg_to_cv2(disparity.image,  "32FC1")    

    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis,...]
    output_dict = model_fn(input_tensor)

    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key:value[0, :num_detections].numpy() 
                    for key,value in output_dict.items()}
    output_dict['num_detections'] = num_detections

    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
    
    if 'detection_masks' in output_dict:
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                output_dict['detection_masks'], output_dict['detection_boxes'],
                image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                        tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()

    vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      output_dict['detection_boxes'],
      output_dict['detection_classes'],
      output_dict['detection_scores'],
      category_index,
      instance_masks=output_dict.get('detection_masks_reframed', None),
      use_normalized_coordinates=True,
      line_thickness=8)      

    boxes = output_dict['detection_boxes']
    scores = output_dict['detection_scores']
    classes = output_dict['detection_classes']
    num_detections = output_dict['num_detections']
    box = boxes.copy()

    objects_msg = Objects()

    objects_msg.header.seq = image.header.seq
    objects_msg.header.stamp = image.header.stamp
    objects_msg.header.frame_id = robot_name + "_left_camera_optical"

    for i in range (0, num_detections):
        if(scores[i] > 0.5): 
            box[i,0] = boxes[i,0] * height
            box[i,1] = boxes[i,1] * width
            box[i,2] = boxes[i,2] * height
            box[i,3] = boxes[i,3] * width
            centerX = int(box[i,1] + (box[i,3] - box[i,1])/2)
            centerY = int(box[i,0] + (box[i,2] - box[i,0])/2)

            # print("Center Coordinates: ({}, {})".format(centerX, centerY))
            
            low_x = int(box[i,1])
            high_x = int(box[i,3])
            min_disp = disp_img[centerY, centerX]

            if(centerY - 10 > 10):
                low_y =  int(centerY - 10)
            else:
                low_y = int(centerY)
            
            if(centerY + 10 < 479):
                high_y = int(centerY + 10)
            else:
                high_y = int(centerY)
                
            for w in range (low_x, high_x):
                for v in range (low_y, high_y):
                    if(disp_img[v, w] > min_disp):
                        min_disp = disp_img[v, w]

            center_disparity = min_disp
            depth = 0.41245282 * fx / center_disparity
            x = (centerX - cx) * depth / fx
            y = (centerY - cy) * depth / fy
            z = depth

            w_x_1 = (box[i,1] - cx) * depth / fx
            w_y_1 = (centerY - cy) * depth / fy
            w_z_1 = depth

            w_x_2 = (box[i,3] - cx) * depth / fx
            w_y_2 = (centerY - cy) * depth / fy
            w_z_2 = depth

            w = math.sqrt((w_x_1 - w_x_2)**2+(w_y_1 - w_y_2)**2+(w_z_1 - w_z_2)**2)

            WHITE = (255,255,255)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_size = 0.28
            font_color = WHITE
            font_thickness = 0
            text = "x: {:.3f}, y: {:.3f}, z: {:.3f}" .format(x, y, z)
            img_text = cv2.putText(image_np, text, (centerX, centerY), font, font_size, font_color, font_thickness, cv2.LINE_AA)

            object_msg = Object()

            object_msg.point.x = x
            object_msg.point.y = y
            object_msg.point.z = z

            object_msg.score = scores[i]
            object_msg.width = w
            
            if (classes[i] == 1):
                object_msg.label = "processingPlant"
            elif (classes[i] == 2):
                object_msg.label = "repairStation"
            elif (classes[i] == 3):
                object_msg.label = "hopper"
            elif (classes[i] == 4):
                object_msg.label = "rock"
            elif (classes[i] == 5):
                object_msg.label = "scout"
            elif (classes[i] == 6):
                object_msg.label = "excavator"
            elif (classes[i] == 7):
                object_msg.label = "hauler"
            objects_msg.obj.append(object_msg)                
        
    msg = bridge.cv2_to_imgmsg(image_np)
    img_pub.publish(msg)
    objects_pub.publish(objects_msg)

def init_object_detection(path_to_model, path_to_label_map):
    global model_fn
    tf.keras.backend.clear_session()
    model = tf.saved_model.load(str.format(path_to_model))
    model_fn = model.signatures['serving_default']

    image_sub_left = message_filters.Subscriber('/' + robot_name + '/camera/left/image_raw', Image)
    disp_sub = message_filters.Subscriber('/' + robot_name + '/camera/disparity', DisparityImage)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub_left, disp_sub], 1, 0.1, allow_headerless=True)

    global img_pub
    global objects_pub
    img_pub = rospy.Publisher('/capricorn/'+robot_name+'/object_detection/image', Image, queue_size=10)
    objects_pub = rospy.Publisher('/capricorn/'+robot_name+'/object_detection/objects', Objects, queue_size=10)

    global category_index
    category_index = label_map_util.create_category_index_from_labelmap(str.format(path_to_label_map), use_display_name=True)
    print("Registering loop callback for {}".format(robot_name))        
    ts.registerCallback(detection_callback)
    rospy.spin()

if __name__ == "__main__":    
    time.sleep(2)
    global robot_name
    robot_name = sys.argv[1]
    path_to_model = sys.argv[2]
    path_to_label_map = sys.argv[3]

    rospy.init_node(robot_name + '_object_detection')
    init_object_detection(path_to_model, path_to_label_map)