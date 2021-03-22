import math
import rospy
import actionlib

import numpy as np
#from gazebo_msgs.msg import ModelState   
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from math import pi

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

# acquire model coordinates before running the class for mapgen
states = Models_state()
coordinates = states.show_gazebo_models()
print(coordinates)

# set the metadata variables defining map size/specifics
# source: https://www.programcreek.com/python/example/95997/nav_msgs.msg.OccupancyGrid
height = 200
width = 200
resolution = 0.25

metadata = MapMetaData()
metadata.resolution = resolution
metadata.width = width
metadata.height = height

# CHANGE TO SET ORIGIN TO THE DESIRED BASE FRAME (i.e., small_scout_1 pose or repair_station pose)
pos = np.array([-width * resolution / 2, -height * resolution / 2, 0])
quat = np.array([0, 0, 0, 1])
metadata.origin = Pose()
metadata.origin.position.x, metadata.origin.position.y = pos[:2]

occGrid = OccupancyGrid()
occGrid.info = metadata

#coordinates[0] = (x,y,z) of small_scout_1
#coordinates[0][0] = x of small_scout_1
#acquire coordinates of all of the POI
coordinates = states.show_gazebo_models()
#print(coordinates[0][1])
occGrid.data = [0]*(width*height)

# 100 = obstacle, -1 = unexplored, 0 = free
def add_obstacle(obx, oby, radius):
    # print(obx)
    # print(oby)
    # print(occGrid.info.width)
    # print(len(occGrid.data))
    obx = obx + 99
    oby = oby + 99
    #occGrid.data[int(oby*occGrid.info.width + obx)] = 100

    for theta in range(0,360,5):
        theta = theta*pi/180
        circX = int(obx + radius*np.cos(theta)) 
        circY = int(oby + radius*np.sin(theta)) 
        if(theta == 90*pi/180):
            print(obx)
            print(oby)
            print(theta)
            print(circX)
            print(circY)
        occGrid.data[int(circY*occGrid.info.width + circX)] = 100

    # for theta in range(0,90,5):
    #     theta = theta*pi/180
    #     circX = int(obx + radius*np.cos(theta))
    #     circY = int(oby + radius*np.sin(theta))
    #     occGrid.data[int(circY*occGrid.info.width + circX)] = 100

    # for theta in range(90, 180,5):
    #     theta = theta*pi/180
    #     circX = int(obx - radius*np.cos(theta))
    #     circY = int(oby + radius*np.sin(theta))
    #     occGrid.data[int(circY*occGrid.info.width + circX)] = 100

    # for theta in range(180, 270,45):
    #     theta = theta*pi/180
    #     circX = int(obx - radius*np.cos(theta))
    #     circY = int(oby - radius*np.sin(theta))
    #     occGrid.data[int(circY*occGrid.info.width + circX)] = 100

    # for theta in range(270, 360,45):
    #     theta = theta*pi/180
    #     circX = int(obx + radius*np.cos(theta))
    #     circY = int(oby - radius*np.sin(theta))
    #     occGrid.data[int(circY*occGrid.info.width + circX)] = 100

def add_initial_obstacles():
    rover_radius = 1.5/2 #m

    #add small_scout_1
    add_obstacle(coordinates[0][0], coordinates[0][1], rover_radius)
    
    #add small_excavator_1
    add_obstacle(coordinates[1][0], coordinates[1][1], rover_radius)

    #add small_hauler_1
    add_obstacle(coordinates[2][0], coordinates[2][1], rover_radius)

    #add repair_station
    add_obstacle(coordinates[3][0], coordinates[3][1], 3.19)

    #add processing_plant (UPDATE BASED ON HOPPER STUFF)
    add_obstacle(coordinates[4][0], coordinates[4][1], 1.8)

rospy.init_node("Ground_Truth")

while not rospy.is_shutdown():
    occGridPub = rospy.Publisher("Ground_Truth_Map", OccupancyGrid, queue_size=1)
        
    add_initial_obstacles()
    #add_obstacle(100,100,0)
    #add_obstacle(10, 4, 4)
    #add_obstacle(12, 6, 0)
    #add_obstacle(-5, -5, 0)
    #add_obstacle(195, 195, 0)
    occGridPub.publish(occGrid)





