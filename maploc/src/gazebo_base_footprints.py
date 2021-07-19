#!/usr/bin/env python3
import math
import rospy
import actionlib

import numpy as np

"""
DISCLAIMER: PLEASE DELETE THIS CODE BEFORE FINAL SUBMISSION, AS IT DRAWS FROM GAZEBO DIRECTLY
THIS CODE IS ONLY MEANT FOR DEBUGGING THE ACCURACY OF ODOMETRY
"""

# from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from math import pi


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name


class Models_state:

    _blockListDict = {
        "small_scout_1": Block("small_scout_1", "map"),
        "small_excavator_1": Block("small_excavator_1", "map"),
        "small_hauler_1": Block("small_hauler_1", "map"),
        "small_scout_2": Block("small_scout_2", "map"),
        "small_excavator_2": Block("small_excavator_2", "map"),
        "small_hauler_2": Block("small_hauler_2", "map"),
    }

    def __init__(self):
        self.scout_pub = rospy.Publisher(
            "/capricorn/small_scout_1/gazebo_pose", PoseStamped, queue_size=1
        )
        self.excavator_pub = rospy.Publisher(
            "/capricorn/small_excavator_1/gazebo_pose", PoseStamped, queue_size=1
        )
        self.hauler_pub = rospy.Publisher(
            "/capricorn/small_hauler_1/gazebo_pose", PoseStamped, queue_size=1
        )
        self.scout2_pub = rospy.Publisher(
            "/capricorn/small_scout_2/gazebo_pose", PoseStamped, queue_size=1
        )
        self.excavator2_pub = rospy.Publisher(
            "/capricorn/small_excavator_2/gazebo_pose", PoseStamped, queue_size=1
        )
        self.hauler2_pub = rospy.Publisher(
            "/capricorn/small_hauler_2/gazebo_pose", PoseStamped, queue_size=1
        )

    def publish_all(self):
        All_coordinate = []
        try:
            model_coordinates = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            for block in self._blockListDict.values():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(
                    blockName, block._relative_entity_name
                )
                gazebo_stamped = PoseStamped()
                gazebo_stamped.header = resp_coordinates.header
                gazebo_stamped.pose = resp_coordinates.pose
                if blockName == "small_scout_1":
                    self.scout_pub.publish(gazebo_stamped)
                elif blockName == "small_excavator_1":
                    self.excavator_pub.publish(gazebo_stamped)
                elif blockName == "small_hauler_1":
                    self.hauler_pub.publish(gazebo_stamped)
                elif blockName == "small_scout_2":
                    self.scout2_pub.publish(gazebo_stamped)
                elif blockName == "small_excavator_2":
                    self.excavator2_pub.publish(gazebo_stamped)
                elif blockName == "small_hauler_2":
                    self.hauler2_pub.publish(gazebo_stamped)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    # def show_gazebo_models(self):
    #     All_coordinate = []
    #     try:
    #         model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         for block in self._blockListDict.values():
    #             blockName = str(block._name)
    #             resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
    #             x = resp_coordinates.pose.position.x
    #             y = resp_coordinates.pose.position.y
    #             z = resp_coordinates.pose.position.z
    #             All_coordinate.append((x, y, z))

    #     except rospy.ServiceException as e:
    #         rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    #     return All_coordinate


# set up the node and then spin to operate on callbacks
if __name__ == "__main__":
    # initialize node
    rospy.init_node("Gazebo_rover_position_publisher")
    rospy.loginfo("GAZEBO PUBLISHER, ONLINE")
    # acquire model coordinates before running the class for mapgen
    states = Models_state()

    # while not rospy.is_shutdown():
    #     states.publish_all()
    #     rospy.
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        states.publish_all()
        r.sleep()
