import math
import rospy
import actionlib

import numpy as np
#from gazebo_msgs.msg import ModelState   
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Models_state:

    _blockListDict = {
        'small_scout_1': Block('small_scout_1', 'link'),
        'small_excavator_1': Block('small_excavator_1', 'link'),
        'small_hauler_1':Block('small_hauler_1', 'link'),
        'repair_station': Block('repair_station', 'link'),
        'processing_plant': Block('processing_plant', 'link')

    }
    
    def show_gazebo_models(self):
        All_coordinate = []
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.values():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)     
                if (block._name == 'small_scout_1'):
                    small_scout_1_pose_x = resp_coordinates.pose.position.x
                    small_scout_1_pose_y = resp_coordinates.pose.position.y
                    small_scout_1_pose_z = resp_coordinates.pose.position.z
                    small_scout_1_coordinate = (small_scout_1_pose_x,small_scout_1_pose_y,small_scout_1_pose_z)
                    #print("small_scout_1_coordinate :")
                    #print(small_scout_1_coordinate)
                    All_coordinate.append(small_scout_1_coordinate)
                if (block._name == 'small_excavator_1'):
                    small_excavator_1_pose_x = resp_coordinates.pose.position.x
                    small_excavator_1_pose_y = resp_coordinates.pose.position.y
                    small_excavator_1_pose_z = resp_coordinates.pose.position.z
                    small_excavator_1_coordinate = (small_excavator_1_pose_x,small_excavator_1_pose_y,small_excavator_1_pose_z)
                    #print("small_excavator_coordinate :")
                    #print(small_excavator_coordinate)
                    All_coordinate.append(small_excavator_1_coordinate)
                if (block._name == 'small_hauler_1'):
                    small_hauler_1_pose_x = resp_coordinates.pose.position.x
                    small_hauler_1_pose_y = resp_coordinates.pose.position.y
                    small_hauler_1_pose_z = resp_coordinates.pose.position.z
                    small_hauler_1_coordinate = (small_hauler_1_pose_x,small_hauler_1_pose_y,small_hauler_1_pose_z)
                    #print("small_hauler_1_coordinate :")
                    #print(small_hauler_1_coordinate)
                    All_coordinate.append(small_hauler_1_coordinate)
                if (block._name == 'repair_station'):
                    repair_station_pose_x = resp_coordinates.pose.position.x
                    repair_station_pose_y = resp_coordinates.pose.position.y
                    repair_station_pose_z = resp_coordinates.pose.position.z
                    repair_station_coordinate = (repair_station_pose_x,repair_station_pose_y,repair_station_pose_z)
                    #print("repair_station_coordinate :")
                    #print(repair_station_coordinate)
                    All_coordinate.append(repair_station_coordinate)
                if (block._name == 'processing_plant'):
                    processing_plant_pose_x = resp_coordinates.pose.position.x
                    processing_plant_pose_y = resp_coordinates.pose.position.y
                    processing_plant_pose_z = resp_coordinates.pose.position.z
                    processing_plant_coordinate = (processing_plant_pose_x,processing_plant_pose_y,processing_plant_pose_z)
                    #print("processing_plant_coordinate :")
                    #print(processing_plant_coordinate)
                    All_coordinate.append(processing_plant_coordinate)
        
        

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        return All_coordinate

states = Models_state()
coordinates = states.show_gazebo_models()
print(coordinates)

rospy.init_node('coordinates_publisher', anonymous=True)

while not rospy.is_shutdown():

    scout_pub = rospy.Publisher('/scout_coordinates', Pose, queue_size = 1)
    excavator_pub = rospy.Publisher('/excavator_coordinates', Pose, queue_size = 1)
    hauler_pub = rospy.Publisher('/hauler_coordinates', Pose, queue_size = 1)
    repair_station_pub = rospy.Publisher('/repair_station_coordinates', Pose, queue_size = 1)
    processing_plant_pub = rospy.Publisher('/processing_plant_coordinates', Pose, queue_size = 1)

    msg_scout = Pose()
    msg_scout.position.x = coordinates[0][0]
    msg_scout.position.y = coordinates[0][1]
    msg_scout.position.z = coordinates[0][2]

    msg_excavator = Pose()
    msg_excavator.position.x = coordinates[1][0]
    msg_excavator.position.y = coordinates[1][1]
    msg_excavator.position.z = coordinates[1][2]

    msg_hauler = Pose()
    msg_hauler.position.x = coordinates[2][0]
    msg_hauler.position.y = coordinates[2][1]
    msg_hauler.position.z = coordinates[2][2]

    msg_repair_station = Pose()
    msg_repair_station.position.x = coordinates[3][0]
    msg_repair_station.position.y = coordinates[3][1]
    msg_repair_station.position.z = coordinates[3][2]

    msg_processing_plant = Pose()
    msg_processing_plant.position.x = coordinates[4][0]
    msg_processing_plant.position.y = coordinates[4][1]
    msg_processing_plant.position.z = coordinates[4][2]

    scout_pub.publish(msg_scout)
    excavator_pub.publish(msg_excavator)
    hauler_pub.publish(msg_hauler)
    repair_station_pub.publish(msg_repair_station)
    processing_plant_pub.publish(msg_processing_plant)




