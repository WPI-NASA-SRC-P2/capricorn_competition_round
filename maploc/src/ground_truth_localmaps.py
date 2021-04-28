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

# from tf import TransformListener

# tf2 reqs
import tf_conversions
import tf2_ros

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
        self.obj_list = []
        self.object_sub = rospy.Subscriber("/capricorn/small_scout_1/object_detection/objects", ObjectArray, self.object_cb)
        
        # occupancy grid publisher
        self.occGridPub = rospy.Publisher("/capricorn/object_detection_map", OccupancyGrid, queue_size=1)
    
    # NECESSARY FUNCTIONS

    # subscriber callback to robot pose, updates robot pose as it moves
    def robot_cb(self, odom):
        self.robot_pose = odom.pose

    # subscriber callback to object detection, updates detected obstacle list
    def object_cb(self, objlist):
        self.obj_list = objlist.obj
        self.update_map()
        

    # initialize/refresh the blank 20x20 map centered on the robot 
    def init_occ_grid(self):
        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        metadata.resolution = 0.05 # (m/pixel) 0.05 matches rtabmap resolution
        metadata.width = int(20/metadata.resolution) # sets the map to be 20m x 20m regardless of resolution
        metadata.height = int(20/metadata.resolution)

        # define origin of blank occupancy grid (should correspond to robot pose)
        base_frame = self.robot_pose.pose
        metadata.origin = base_frame

        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.info = metadata

        # initialize map data as all zeros
        self.occ_grid.data = [0] * self.occ_grid.info.width*self.occ_grid.info.height

    # transform the object list from camera frame to base frame
    # TODO: TEST THIS TO MAKE SURE IT WORKS PROPERLY
    def transform(self, old_x, old_y, robotname):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform(robotname+"_left_camera_optical", robotname+'_base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
  

        new_x = trans.transform.translation.x + old_x 
        new_y = trans.transform.translation.y + old_y
    
    # plot single object on the occupancy grid
    # 100 = obstacle, -1 = unexplored, 0 = free
    def add_obstacle(self, obx, oby, radius):
        # plot everything w.r.t. center of the grid (robot is at center)
        obx = obx + int(self.occ_grid.info.width/2) #- initial_origin_x
        oby = oby + int(self.occ_grid.info.height/2) #- initial_origin_y

        
        # plot a circle around the center point of the object
        for theta in range(0,360,2):
            theta = theta*pi/180

            # convert meters to pixels
            obx_p = obx/self.occ_grid.info.resolution
            oby_p = oby/self.occ_grid.info.resolution
            radius_p = radius/self.occ_grid.info.resolution

            circX = int((obx_p + radius_p*np.cos(theta)))  
            circY = int((oby_p + radius_p*np.sin(theta))) 
            
            # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
            if int(circY*self.occ_grid.info.width + circX) >= 0 and int(circY*self.occ_grid.info.width + circX) < self.occ_grid.info.width*self.occ_grid.info.height: 
                self.occ_grid.data[int(circY*self.occ_grid.info.width + circX)] = 100


    # plot all objects in object list
    # - basically plot single object many times (across length of object list)
    def add_all_obstacles(self):
        # add all of the obstacles from the obstacle list to the map (follow the dostuff function from the cpp file)
        
        for obj in self.obj_list:
            # TODO: TRANSFORM THE POINTS BEFORE RUNNING ADD_OBSTACLE
            obx = obj.point.x
            oby = obj.point.y
            radius = (obj.width)/2
            self.add_obstacle(obx, oby, radius)      

    # publish the updated occupancy grid
    # - publish self.occ_grid after finished
    def gridPublisher(self):
        self.occGridPub.publish(self.occ_grid)

    # overall map editing function:
    def update_map(self):
        self.init_occ_grid()
        self.add_all_obstacles()
        self.add_obstacle(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, 5)
        self.gridPublisher()

#____________________________________________________________________________
# OLD CODE SNIPPETS BELOW: TODO: modify them to integrate with new class



if __name__=="__main__":
    # can run the ground_truth_localmaps.py with 2 input args
    # in format ground_truth_localmaps.py baseFrameEntityName maporigin
    
    # TODO: modify this code snippet for input arg-based robot selection
    # if len(sys.argv) < 3:
    #     occGrid = setupLocalMap()  # DEFAULT CONDITION OF small_scout_1 AS BASE FRAME with ORIGIN AT BOTTOM LEFT
    #     baseFrameEntityName = 'small_scout_1'
    # elif len(sys.argv) == 2:
    #     occGrid = setupLocalMap(baseFrameEntityName=sys.argv[1], maporigin=sys.argv[2])
    #     baseFrameEntityName = sys.argv[1]
    #     maporigin = sys.argv[2]
    # else: 
    #     print("please enter the function with the args <baseFrameEntityName> and 'bottom_left'")
    #     exit

    # initialize the node 
    rospy.init_node("Ground_Truth_Localmaps")
    print("ground_truth_localmaps node, online")
    ObstacleMap = Object_Plotter()

    # set up occupancy grid publisher
    
    
    # loop to keep updating the map
    # while not rospy.is_shutdown():
    #     #rospy.sleep()
    #     rospy.wait_for_message("/capricorn/small_scout_1/camera/odom", Odometry)
    #     rospy.wait_for_message("/capricorn/small_scout_1/object_detection/objects", ObjectArray)
    #     ObstacleMap.update_map()
    rospy.spin()

        
