import math
import rospy
import actionlib

import numpy as np
#from gazebo_msgs.msg import ModelState   
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from math import pi
import sys

from perception.msg import Object
from perception.msg import ObjectArray

from tf import TransformListener

# tf2 reqs
#import tf_conversions
#import tf2_ros

# TODO: 
# - take object detection instead of gazebo
# - no rtabmap, just make local map
# - make sure that the file works for not just small scout 1 (change namespace)

# Object Plotter Class will store the object list received from subscriber
#  and will update it within itself 
class Object_Plotter:

    def __init__(self):
        # blank 20x20 occupancy grid fields
        # pose subscriber updates robot pose as it moves
        self.robot_pos_sub = self.object_sub = rospy.Subscriber("/capricorn/small_scout_1/camera/odom", Odometry, self.robot_cb)
        self.robot_pose = PoseWithCovariance()# value indicates the robot base frame pose
        self.occ_grid = OccupancyGrid()

        # object detection data and subscriber
        self.obj_list = ObjectArray()
        self.object_sub = rospy.Subscriber("/capricorn/small_scout_1/object_detection/objects", ObjectArray, self.object_cb)
        
        # occupancy grid publisher
        self.occGridPub = rospy.Publisher("/capricorn/object_detection_map", OccupancyGrid, queue_size=1)
    
    # NECESSARY FUNCTIONS

    # subscriber callback to robot pose, updates robot pose as it moves
    def robot_cb(self, odom):
        self.robot_pose = odom.pose

    # subscriber callback to object detection, updates detected obstacle list
    def object_cb(self, objlist):
        self.obj_list = objlist

    # initialize/refresh the blank 20x20 map centered on the robot 
    def init_occ_grid(self):
        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        metadata.resolution = 0.05 # (m/pixel) 0.05 matches rtabmap resolution
        metadata.width = 20/metadata.resolution # sets the map to be 20m x 20m regardless of resolution
        metadata.height = 20/metadata.resolution 

        # define origin of blank occupancy grid (should correspond to robot pose)
        base_frame = self.robot_pose.pose
        metadata.origin = base_frame

        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.info = metadata

        # initialize map data as all zeros
        self.occ_grid.data = [0] * (self.occ_grid.metadata.width*self.occ_grid.metadata.height)

    # transform the object list from camera frame to base frame
    def transform(self, old_x, old_y, robotname):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)


        try:
            trans = tfBuffer.lookup_transform(robotname+"_left_camera_optical", robotname+'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
  

        new_x = trans.transform.translation.x + old_x 
        new_y = trans.transform.translation.y + old_y
    
    # plot single object
    # - basically modify the add_obstacle method

    # plot all objects in object list
    # - basically plot single object many times (across length of object list)

    # publish the updated occupancy grid
    # - publish self.occ_grid after finished

#____________________________________________________________________________
# OLD CODE SNIPPETS BELOW: TODO: modify them to integrate with new class

# 100 = obstacle, -1 = unexplored, 0 = free
def add_obstacle(occGrid, obx, oby, radius):
    # - if the occupancy grid maporigin is set to 'center' then the input obx and oby will reflect the necessary offset
    # - similarly, obx and oby will be input w.r.t. the proper frame reference already
    
    for theta in range(0,360,2):
        theta = theta*pi/180

        # convert meters to pixels
        obx_p = obx/occGrid.info.resolution
        oby_p = oby/occGrid.info.resolution
        radius_p = radius/occGrid.info.resolution

        circX = int((obx_p + radius_p*np.cos(theta)))  
        circY = int((oby_p + radius_p*np.sin(theta))) 
        
        # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
        if int(circY*occGrid.info.width + circX) >= 0 and int(circY*occGrid.info.width + circX) < occGrid.info.width*occGrid.info.height: 
            occGrid.data[int(circY*occGrid.info.width + circX)] = 100


def add_initial_obstacles(occGrid, baseFrameEntityName, coordinates):
    # - input argument coordinates is the array of coordinates produced by Models_state.show_gazebo_models()
    #   thus, the locations of each of these obstacles is already with respect to the correct base frame
    # - baseFrameEntityName is to ensure that the base frame object is not published as an obstacle if it is a rover, as that would likely disrupt navigation

    rover_radius = 1.5/2 #m

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

    # set up occupancy grid publisher
    
    # loop to keep updating the map
    while not rospy.is_shutdown():
        occGridPub.publish(occGrid)

        obstacleList = initializeObstacleList(baseFrameEntityName) # update the obstacleList by polling the updated gazebo coordinates
        updateMap(occGrid, obstacleList)
