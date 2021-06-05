#!/usr/bin/env python3
"""
Author: Madhan Suresh Babu
Email: msureshbabu@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This ros node takes one command line argument:
robot_name = small_scout_1

It subscribes to :
1. Output of the Object detection module

It adds an offset to the estimated 3d location of the object calculated by the inverse camera model so that the final estimate
is the center of the object instead of its periphery. A non zero offset value shifts the object location estimate along the line
joining the object and robot_name. 

It publishes one topic : 
1. ObjectArray() containing detected objects with the location of their centers.
"""
import rospy
import tf as tf2
import sys
import message_filters
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

from perception.msg import Object
from perception.msg import ObjectArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' 

import cv2
import numpy as np

bridge = CvBridge()
processingLock = False
objects = None
UPDATE_HZ = 20

# calibrated offset values
XZ = {"processingPlant" : 1.5, "repairStation" : 2.0, "excavator" : 0.5, "hauler" : 0.0, "scout" : 0.0}

def add_offset() :
	processingLock = True

	global objects
	global objectLoc_pub
	global br

	objects_msg = ObjectArray()
	objects_msg.header.seq = objects.header.seq
	objects_msg.header.stamp = objects.header.stamp
	objects_msg.header.frame_id = robot_name + "_left_camera_optical"

	for obj in objects.obj : 
		if (obj.label == "processingPlant" or obj.label == "repairStation" or obj.label == "excavator" or obj.label == "hauler" or obj.label == "scout") and obj.label != robot_name :
			obj_offset = Object()
			obj_offset.score = obj.score
			obj_offset.label = obj.label
			obj_offset.center = obj.center
			obj_offset.size_x = obj.size_x
			obj_offset.size_y = obj.size_y

			point = Point()
			yaw = math.atan(obj.point.pose.position.x/obj.point.pose.position.z)
			point.x = obj.point.pose.position.x - XZ[obj.label] * math.sin(yaw)
			point.y = obj.point.pose.position.y
			point.z = obj.point.pose.position.z + XZ[obj.label] * math.cos(yaw)
			obj_offset.point.pose.position = point
			obj_offset.point.pose.orientation = Quaternion(0,0,0,0)
			obj_offset.width = obj.width

			# # for visualization
			# x = point.x
			# y = point.y
			# z = point.z

			# if (obj_offset.label == "processingPlant") : 
			# 	br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "processing_plant_position_estimate", robot_name + "_left_camera_optical") 

			# if (obj_offset.label == "repairStation") : 
			# 	br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "repair_station_position_estimate", robot_name + "_left_camera_optical")

			# if (obj_offset.label == "excavator") : 
			# 	br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "small_excavator_1_position_estimate", robot_name + "_left_camera_optical")

			# if (obj_offset.label == "hauler") : 
			# 	br.sendTransform((x,y,z), tf2.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "small_hauler_1_position_estimate", robot_name + "_left_camera_optical")            

		objects_msg.obj.append(obj_offset)

	objectLoc_pub.publish(objects_msg)

	processingLock = False



def object_callback(data) :
	global objects
	objects = data


def init_3d_localization():
	update_rate = rospy.Rate(UPDATE_HZ)

	rospy.Subscriber('/capricorn/' + robot_name + '/object_detection/objects', ObjectArray, object_callback)

	global objectLoc_pub
	objectLoc_pub = rospy.Publisher('/capricorn/'+robot_name+'/object_detection_offset/objects', ObjectArray, queue_size=10)

	global br
	br = tf2.TransformBroadcaster()

	while not rospy.is_shutdown() :
		if (not processingLock and objects is not None): 
			add_offset()
		update_rate.sleep()


if __name__ == '__main__' : 
	global robot_name
	robot_name = sys.argv[1]
	
	rospy.init_node(robot_name + '_3d_object_localization',anonymous=True)

	init_3d_localization() 