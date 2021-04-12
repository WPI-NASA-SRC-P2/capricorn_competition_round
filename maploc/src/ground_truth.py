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

# class ModelsState keeps track of the ground truths of each object 
# of interest in the Gazebo file, with respect to the heightmap frame
class ModelsState:
    # dictionary for obtaining location of the objects of interest wrt heightmap
    # ('link' is the same as heightmap for all intents and purposes)
    _blockListDict = {
        'small_scout_1': Block('small_scout_1', 'link'),
        'small_excavator_1': Block('small_excavator_1', 'link'),
        'small_hauler_1':Block('small_hauler_1', 'link'),
        'repair_station': Block('repair_station', 'link'),
        'processing_plant': Block('processing_plant', 'link')
    }
    
    # function returns the xyz coordinates of each object of interest to be plotted by addObstacle function
    def show_gazebo_models(self):
        all_coordinate = []
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.values():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                x = resp_coordinates.pose.position.x
                y = resp_coordinates.pose.position.y
                z = resp_coordinates.pose.position.z
                all_coordinate.append((x, y, z))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        return all_coordinate

# acquire model coordinates before running the class for mapgen
states = ModelsState()
coordinates = states.show_gazebo_models()
print(coordinates)

# set the metadata variables defining map size/specifics
# source: https://www.programcreek.com/python/example/95997/nav_msgs.msg.OccupancyGrid
height = 200
width = 200 #simulation is 200m by 200m 
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

#acquire coordinates of all of the POI
coordinates = states.show_gazebo_models()
#print(coordinates[0][1])
occGrid.data = [0]*(width*height)

# 100 = obstacle, -1 = unexplored, 0 = free
def add_obstacle(initial_origin_x, initial_origin_y, obx, oby, radius):
    obx = obx + 99 - initial_origin_x
    oby = oby + 99 - initial_origin_y

    for theta in range(0,360,5):
        theta = theta*pi/180
        circX = int(obx + radius*np.cos(theta))   #int
        circY = int(oby + radius*np.sin(theta))   #int
        occGrid.data[int(circY*occGrid.info.width + circX)] = 100

def add_initial_obstacles(initial_origin_x, initial_origin_y):
    rover_radius = 1.5/2 #m

    #add small_scout_1
    add_obstacle(initial_origin_x, initial_origin_y, coordinates[0][0], coordinates[0][1], rover_radius)
    
    #add small_excavator_1
    add_obstacle(initial_origin_x, initial_origin_y, coordinates[1][0], coordinates[1][1], rover_radius)

    #add small_hauler_1
    add_obstacle(initial_origin_x, initial_origin_y, coordinates[2][0], coordinates[2][1], rover_radius)

    #add repair_station
    add_obstacle(initial_origin_x, initial_origin_y, coordinates[3][0], coordinates[3][1], 3.19)

    #add processing_plant (UPDATE BASED ON HOPPER STUFF)
    add_obstacle(initial_origin_x, initial_origin_y, coordinates[4][0], coordinates[4][1], 1.8)

rospy.init_node("Ground_Truth")

while not rospy.is_shutdown():
    states = ModelsState()
    coordinates = states.show_gazebo_models()
    print(coordinates)
    
    occGridPub = rospy.Publisher("/capricorn/Ground_Truth_Map", OccupancyGrid, queue_size=1)
    
    initial_origin_x = coordinates[0][0]
    initial_origin_y = coordinates[0][1]
    add_initial_obstacles(initial_origin_x, initial_origin_y)
    occGridPub.publish(occGrid)





