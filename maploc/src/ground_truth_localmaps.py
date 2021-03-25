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
import sys

# tf2 reqs
#import tf_conversions
#import tf2_ros

# Acquiring object Poses
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
                x = resp_coordinates.pose.position.x
                y = resp_coordinates.pose.position.y
                z = resp_coordinates.pose.position.z
                All_coordinate.append((x, y, z))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        return All_coordinate

    # set which of the five entities is the origin entity for the local map
    # i.e., input arg 'small_scout_1' would result in obtaining poses w.r.t. small_scout_1
    def setRelativeEntity(self, entityname):
        self._blockListDict = {
            'small_scout_1': Block('small_scout_1', entityname),
            'small_excavator_1': Block('small_excavator_1', entityname),
            'small_hauler_1':Block('small_hauler_1', entityname),
            'repair_station': Block('repair_station', entityname),
            'processing_plant': Block('processing_plant', entityname)
        }
    
    # obtain the x,y,z position w.r.t. base frame of desired entity
    def getXYCoordinates(self, entityname):
        xyBlock = self._blockListDict[entityname]
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(entityname, xyBlock._relative_entity_name)
        x = resp_coordinates.pose.position.x
        y = resp_coordinates.pose.position.y
        z = resp_coordinates.pose.position.z
        return [x,y,z]


# 100 = obstacle, -1 = unexplored, 0 = free
def add_obstacle(occGrid, obx, oby, radius):
    # - if the occupancy grid maporigin is set to 'center' then the input obx and oby will reflect the necessary offset
    # - similarly, obx and oby will be input w.r.t. the proper frame reference already
    print("radius = ", radius)

    for theta in range(0,360,2):
        theta = theta*pi/180

        # convert meters to pixels
        obx_p = obx/occGrid.info.resolution
        oby_p = oby/occGrid.info.resolution
        radius_p = radius/occGrid.info.resolution
        print('radius_p = ', radius_p)
        circX = int((obx_p + radius_p*np.cos(theta)))  
        circY = int((oby_p + radius_p*np.sin(theta))) 
        
        # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
        if int(circY*occGrid.info.width + circX) >= 0 and int(circY*occGrid.info.width + circX) < occGrid.info.width*occGrid.info.height: 
            occGrid.data[int(circY*occGrid.info.width + circX)] = 100


def add_initial_obstacles(occGrid, baseFrameEntityName, coordinates):
    # - input argument coordinates is the array of coordinates produced by Models_state.show_gazebo_models()
    #   thus, the locations of each of these obstacles is already with respect to the correct base frame
    # - baseFrameEntityName is to ensure that the base frame object is not published as an obstacle if it is a rover, as that would likely disrupt navigation

    rover_radius = 0.75 #m

    if baseFrameEntityName == 'small_scout_1':
        #add small_excavator_1
        add_obstacle(occGrid, coordinates[1][0], coordinates[1][1], rover_radius)

        #add small_hauler_1
        add_obstacle(occGrid, coordinates[2][0], coordinates[2][1], rover_radius)
    elif baseFrameEntityName == 'small_excavator_1':
        #add small_scout_1
        add_obstacle(occGrid, coordinates[0][0], coordinates[0][1], rover_radius)
        
        #add small_hauler_1
        add_obstacle(occGrid, coordinates[2][0], coordinates[2][1], rover_radius)
    elif baseFrameEntityName == 'small_hauler_1':
        #add small_scout_1
        add_obstacle(occGrid, coordinates[0][0], coordinates[0][1], rover_radius)
        
        #add small_excavator_1
        add_obstacle(occGrid, coordinates[1][0], coordinates[1][1], rover_radius)
    else:
        #add small_scout_1
        add_obstacle(occGrid, coordinates[0][0], coordinates[0][1], rover_radius)
        
        #add small_excavator_1
        add_obstacle(occGrid, coordinates[1][0], coordinates[1][1], rover_radius)

        #add small_hauler_1
        add_obstacle(occGrid, coordinates[2][0], coordinates[2][1], rover_radius) 
    
    #add repair_station
    add_obstacle(occGrid, coordinates[3][0], coordinates[3][1], 3.19)

    #add processing_plant (Hopper is not included, just the circular processing plant. Can compensate for hopper by using increased radius)
    add_obstacle(occGrid, coordinates[4][0], coordinates[4][1], 1.8)

# Set up the Local Map around the selected entity
# defaults set to small_scout_1 local map with a map resolution matching rtabmap default and a map size of 20m x 20m with origin at bottom left
# units: mapresolution = meters/pixel, mapheight = pixels, mapwidth = pixels
# RETURNS: empty 2D OccupancyGrid according to the specifications
def setupLocalMap(baseFrameEntityName='small_scout_1', mapresolution=0.05, mapheight=400, mapwidth=400, maporigin='bottom_left'):
    # acquire model coordinates 
    baseFrame = baseFrameEntityName
    states = Models_state()
    states.setRelativeEntity(baseFrame) # transform rover, repair station, and processing plant coordinates to w.r.t. base frame
    coordinates = states.show_gazebo_models() # acquire poses of rover etc.

    # setup map metadata
    metadata = MapMetaData()
    metadata.resolution = mapresolution
    metadata.width = mapwidth
    metadata.height = mapheight

    # obtain x,y,z values of the base frame
    basexyz = states.getXYCoordinates(baseFrame)

    if maporigin == 'center':
        # will be implemented later to allow for 360 view
        # pos = np.array([-width * resolution / 2, -height * resolution / 2, 0])
        # quat = np.array([0, 0, 0, 1])
        # metadata.origin = Pose()
        # metadata.origin.position.x, metadata.origin.position.y = pos[:2]        
        print('please use default maporigin instead at the moment')
    else: # origin is the bottom left of the map
        metadata.origin = Pose()
        metadata.origin.position.x = float(basexyz[0]) # using float to ensure rounds properly (no exponents close to 0)
        metadata.origin.position.y = float(basexyz[1])

    # set up 2D OccupancyGrid
    occGrid = OccupancyGrid()
    occGrid.info = metadata
    occGrid.data = [0] * (mapwidth*mapheight) # initialized filled with free/unoccupied space
    
    # add the repair station, processing plant, and the non-baseframe rovers to the occupancy grid
    add_initial_obstacles(occGrid, baseFrameEntityName, coordinates)

    return occGrid

# obstacleList is an array of triples [[x1,y1,radius1], [x2,y2,radius2], ...]
def updateMap(occGrid, obstacleList):
    occGrid.data = [0] * (occGrid.info.width*occGrid.info.height) # wipe the map clean and replace all objects
    for obstacle in obstacleList: 
        add_obstacle(occGrid, obstacle[0], obstacle[1], obstacle[2]) # add obstacles using their x, y, radius

# add the non-baseframe rovers and the processing plant and repair station to the obstacle list
def initializeObstacleList(baseFrameEntityName):
    baseFrame = baseFrameEntityName
    states = Models_state()
    states.setRelativeEntity(baseFrame) # transform rover, repair station, and processing plant coordinates to w.r.t. base frame
    coordinates = states.show_gazebo_models() # acquire poses of rover etc.
    
    if baseFrameEntityName == 'small_scout_1':
        np.delete(coordinates, 0, 0) # remove small_scout_1 from obstacle list
    elif baseFrameEntityName == 'small_excavator_1':
        np.delete(coordinates, 1, 0) # remove small_excavator_1 from obstacle list
    elif baseFrameEntityName == 'small_hauler_1':
        np.delete(coordinates, 2, 0) # remove small_hauler_1 from obstacle list

    return coordinates

if __name__=="__main__":
    # can run the ground_truth_localmaps.py with 2 input args
    # in format ground_truth_localmaps.py baseFrameEntityName maporigin
    if len(sys.argv) < 3:
        occGrid = setupLocalMap()  # DEFAULT CONDITION OF small_scout_1 AS BASE FRAME with ORIGIN AT BOTTOM LEFT
        baseFrameEntityName = 'small_scout_1'
    elif len(sys.argv) == 2:
        occGrid = setupLocalMap(baseFrameEntityName=sys.argv[1], maporigin=sys.argv[2])
        baseFrameEntityName = sys.argv[1]
        maporigin = sys.argv[2]
    else: 
        print("please enter the function with the args <baseFrameEntityName> and 'bottom_left'")
        exit

    # initialize the node 
    rospy.init_node("Ground_Truth_Localmaps")
    print("ground_truth_localmaps node, online")

    # add the original five items to the obstacle list, except for the base frame if it is a rover
    obstacleList = initializeObstacleList(baseFrameEntityName)
    #print(obstacleList)
    # set up occupancy grid publisher
    occGridPub = rospy.Publisher("/capricorn/Ground_Truth_Map", OccupancyGrid, queue_size=1)
    
    # loop to keep updating the map
    while not rospy.is_shutdown():
        occGridPub.publish(occGrid)
        
        # refresh map
        occGrid.data = [0] * (occGrid.info.width*occGrid.info.height)
        states = Models_state()
        states.setRelativeEntity(baseFrameEntityName) # transform rover, repair station, and processing plant coordinates to w.r.t. base frame
        coordinates = states.show_gazebo_models() # acquire poses of rover etc.
        add_initial_obstacles(occGrid, baseFrameEntityName, coordinates)

        # obstacleList = initializeObstacleList(baseFrameEntityName) # update the obstacleList by polling the updated gazebo coordinates
        # obstacleList.append((10, 10, 6)) # just an example of appending obstacles to obstaclelist
        # updateMap(occGrid, obstacleList)
