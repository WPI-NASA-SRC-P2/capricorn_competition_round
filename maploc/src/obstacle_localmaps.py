#!/usr/bin/env python3 
import math
import rospy
import actionlib
import numpy as np
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
from math import pi
import sys
from perception.msg import Object
from perception.msg import ObjectArray
# tf2 req
import tf_conversions
import tf2_ros
 
class ObjectPlotter:
    def __init__(self):
        # blank 20x20 occupancy grid fields
        # pose subscriber updates robot pose as it moves
        self.robot_name = str(sys.argv[1])
        #robot_name+'_base_footprint', robot_name+"_left_camera_optical"
        self.robot_pos_sub = rospy.Subscriber("/" + self.robot_name +"/camera/odom", Odometry, self.robotCb)
        self.robot_pose = PoseWithCovariance()# value indicates the robot base frame pose
        self.occ_grid = OccupancyGrid()
        # object detection data and subscriber
        self.obj_list = ObjectArray()
        self.object_sub = rospy.Subscriber("/capricorn/" + self.robot_name + "/object_detection/objects", ObjectArray, self.objectCb)
        # occupancy grid publisher
        self.occGridPub = rospy.Publisher("/capricorn/" + self.robot_name + "/object_detection_map", OccupancyGrid, queue_size=1)
    
    # subscriber callback to robot pose, updates robot pose as it moves
    def robotCb(self, odom):
        self.robot_pose = odom.pose
        # print(f'Robot Pose Received: {self.robot_pose}')
        # TODO: FOR TESTING PURPOSES ONLY:
        # rospy.loginfo("Robot Pose Received")
        # self.updateMap()

    # subscriber callback to object detection, updates detected obstacle list
    def objectCb(self, objlist):
        self.obj_list = objlist
        self.updateMap()
        # useful for debugging any issues with object detection
        #rospy.loginfo(f'Map has updated: First object at {self.obj_list.obj[0].center}, No. of objects: {len(self.obj_list.obj)}')
    
    # initialize/refresh the blank 20x20 map centered on the robot 
    def resetOccGrid(self):
        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        metadata.resolution = 0.20 # (m/pixel) 0.05 matches rtabmap resolution
        metadata.width = int(40/metadata.resolution) # sets the map to be 20m x 20m regardless of resolution
        metadata.height = int(40/metadata.resolution)

        # we use a blank pose centered in the map as opposed to the robot pose from odom at the moment. Everything is still w.r.t. robot base frame so the relative positions are correct
        base_frame = Pose()
        # offset the map such that the bottom left of the map is -10m x -10m to the bottom left of the robot
        # thus making the robot pose be the center of the map
        base_frame.position.x = -20
        base_frame.position.y = -20
        metadata.origin = base_frame 

        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.info = metadata
        # initialize map data as all zeros
        UNOCCUPIED = 0
        self.occ_grid.data = [UNOCCUPIED] * self.occ_grid.info.width*self.occ_grid.info.height
    
    # transform the object list from camera frame to base frame
    # TODO: This function is in progress, it is not called in the code at the moment so it should not impact the launch
    def transform(self, old_x, old_y, robotname):
        # rate = rospy.Rate(10)
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        new_x = old_x
        new_y = old_y
        print("inside transform function")
        #try:
        trans = tf_buffer.lookup_transform(robotname+'_base_footprint', robotname+"_left_camera_optical", rospy.Time())
        new_x = trans.transform.translation.x + old_x 
        new_y = trans.transform.translation.y + old_y
        print('Transform works')
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    rospy.sleep(0.1)
        return new_x, new_y

    # plot single object on the occupancy grid
    # 100 = obstacle, -1 = unexplored, 0 = free
    def addObstacle(self, obx, oby, radius):
        # plot everything w.r.t. center of the grid
        obx = obx + 20
        oby = oby + 20
        # convert meters to pixels
        no_zeroes_here = 0.0
        if(self.occ_grid.info.resolution == 0):                           #HOTFIX: TO PREVENT DIVIDE-BY-ZERO-ERROR
            no_zeroes_here = 0.05
        obx_p = obx/(self.occ_grid.info.resolution + no_zeroes_here)
        oby_p = oby/(self.occ_grid.info.resolution + no_zeroes_here)
        radius_p = radius/(self.occ_grid.info.resolution + no_zeroes_here)
        # rospy.loginfo("Im here")
        # fill in obstacles found within localmaps
        for val in range(0, 901):
            # counter-clockwise (right-bound)
            pos_theta = (val / 5.0)*pi/180 
            # clockwise (left-bound)
            neg_theta = -(val / 5.0)*pi/180 
            # # swap obx_p and oby_p
            # left_bound = int(obx_p + radius_p*np.sin(neg_theta))
            # right_bound = int(obx_p + radius_p*np.sin(pos_theta))
            # circY = int(oby_p + radius_p*np.cos(pos_theta))
            # swap obx_p and oby_p
            up_bound = int(oby_p + radius_p*np.sin(neg_theta))
            down_bound = int(oby_p + radius_p*np.sin(pos_theta))
            circX = int(obx_p + radius_p*np.cos(pos_theta))
            # check boundary cases
            # first case is if y value is not valid
            if circX not in range(0, self.occ_grid.info.width):
                continue
            # second case is if x values gotten from pos_theta and neg_theta are out of the grid
            if up_bound not in range(0, self.occ_grid.info.height) and down_bound not in range(0, self.occ_grid.info.height):
                continue
            if up_bound not in range(0, self.occ_grid.info.height):
                up_bound = 0
            if down_bound not in range(0, self.occ_grid.info.height):
                down_bound = self.occ_grid.info.height
            # fill in pixels in-between 
            pos = up_bound
            # rospy.loginfo("circX = " + str(circX) + " up_bound = " + str(up_bound) + " down_bound = " + str(down_bound))
            # circX = self.occ_grid.info.width - circ
            while pos < down_bound:
                OCCUPIED = 100
                # rospy.loginfo("printing within map")
                self.occ_grid.data[int(pos*(self.occ_grid.info.width) + circX)] = OCCUPIED
                pos += 1
        # for val in range(90, 991):
        #     # counter-clockwise (right-bound)
        #     pos_theta = (val / 5.0)*pi/180 
        #     # clockwise (left-bound)
        #     neg_theta = -(val / 5.0)*pi/180 
        #     # # swap obx_p and oby_p
        #     # left_bound = int(obx_p + radius_p*np.sin(neg_theta))
        #     # right_bound = int(obx_p + radius_p*np.sin(pos_theta))
        #     # circY = int(oby_p + radius_p*np.cos(pos_theta))
        #     # swap obx_p and oby_p
        #     left_bound = int(oby_p + radius_p*np.sin(neg_theta))
        #     right_bound = int(oby_p + radius_p*np.sin(pos_theta))
        #     circY = int(obx_p + radius_p*np.cos(pos_theta))
        #     # check boundary cases
        #     # first case is if y value is not valid
        #     if circY not in range(0, self.occ_grid.info.height + 1):
        #         continue
        #     # second case is if x values gotten from pos_theta and neg_theta are out of the grid
        #     if left_bound not in range(0, self.occ_grid.info.width + 1) and right_bound not in range(0, self.occ_grid.info.width + 1):
        #         continue
        #     if left_bound not in range(0, self.occ_grid.info.width + 1):
        #         left_bound = 0
        #     if right_bound not in range(0, self.occ_grid.info.width + 1):
        #         right_bound = self.occ_grid.info.width
        #     # fill in pixels in-between 
        #     pos = left_bound
        #     circY = self.occ_grid.info.height - circY
        #     while pos < right_bound:
        #         OCCUPIED = 100
        #         # rospy.loginfo("printing within map")
        #         self.occ_grid.data[int(circY*(self.occ_grid.info.width) + pos)] = OCCUPIED
        #         pos += 1
 


            # # for cos, range is 0-2PI
            # circX = int((obx_p + radius_p*np.cos(theta)))
            # # for sin, range is -PI - PI  
            # circY = int((oby_p + radius_p*np.sin(theta)))
            # # define bounds of if index value is valid 
            # voxel_not_negative = int(circY*self.occ_grid.info.width + circX) >= 0
            # voxel_not_outside = int(circY*self.occ_grid.info.width + circX) < self.occ_grid.info.width*self.occ_grid.info.height
            
            # # obstacle wrapping check (don't plot stuff behind when object has disappeared)
            # # first, check if the x value is within boundary 0 to self.occ_grid.info.width
            # voxel_not_wrapping = circX in range(0, self.occ_grid.info.width) and circY in range(0, self.occ_grid.info.height)
            # # next, check if the circY is also within boundary 0 to self.occ_grid.info.height
            # # voxel_not_wrapping = circY in range(0, self.occ_grid.info.height)
            # # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
            # if voxel_not_negative and voxel_not_outside and voxel_not_wrapping: 
            #     #rospy.loginfo("Obstacle Plotted")
            #     OCCUPIED = 100
            #     self.occ_grid.data[int(circY*self.occ_grid.info.width + circX)] = OCCUPIED

        # # plot everything w.r.t. center of the grid (robot is at center)
        # obx = obx + 10
        # oby = oby + 10
        # # plot a circle around the center point of the object
        # previous_node_x = 0
        # previous_node_y = 0
        # current_node_x = 0
        # current_node_y = 0
        # for theta in range(0,90,1):
        #     theta = theta*pi/180
        #     # convert meters to pixels
        #     obx_p = obx/self.occ_grid.info.resolution
        #     oby_p = oby/self.occ_grid.info.resolution
        #     radius_p = radius/self.occ_grid.info.resolution
        #     # for cos, range is 0-2PI
        #     circX = int((obx_p + radius_p*np.cos(theta)))
        #     # for sin, range is -PI - PI  
        #     circY = int((oby_p + radius_p*np.sin(theta)))
        #     # define bounds of if index value is valid 
        #     voxel_not_negative = int(circY*self.occ_grid.info.width + circX) >= 0
        #     voxel_not_outside = int(circY*self.occ_grid.info.width + circX) < self.occ_grid.info.width*self.occ_grid.info.height
            
        #     # obstacle wrapping check (don't plot stuff behind when object has disappeared)
        #     # first, check if the x value is within boundary 0 to self.occ_grid.info.width
        #     voxel_not_wrapping = circX in range(0, self.occ_grid.info.width) and circY in range(0, self.occ_grid.info.height)
        #     # next, check if the circY is also within boundary 0 to self.occ_grid.info.height
        #     # voxel_not_wrapping = circY in range(0, self.occ_grid.info.height)
        #     # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
        #     if voxel_not_negative and voxel_not_outside and voxel_not_wrapping: 
        #         #rospy.loginfo("Obstacle Plotted")
        #         OCCUPIED = 100
        #         self.occ_grid.data[int(circY*self.occ_grid.info.width + circX)] = OCCUPIED
        #         # rospy.loginfo('voxel_not_negative = ' + str(int(circY*self.occ_grid.info.width + circX)))
        #         if theta == 0:
        #             previous_node_x = circX
        #             previous_node_y = circY
        #         else:
        #             # current_node = (circY*self.occ_grid.info.width + circX)
        #             current_node_x = circX
        #             current_node_y = circY
        #             # x1, y1 = previous_node
        #             x1 = previous_node_x
        #             y1 = previous_node_y
        #             # x2, y2 = current_node
        #             x2 = current_node_x
        #             y2 = current_node_y
        #             for j in range(0, 5):
        #                 u = j / 5
        #                 x = int(x2 * u + x1 * (1 - u))
        #                 y = int(y2 * u + y1 * (1 - u))
        #                 self.occ_grid.data[int(y*self.occ_grid.info.width + x)] = OCCUPIED
        #             previous_node_x = current_node_x
        #             previous_node_y = current_node_y

    
    # plot all objects in object list
    # - basically plot single object many times (across length of object list)
    def addAllObstacles(self):
        # check if there are any observed objects
        #rospy.loginfo("Adding Obstacles from Obj list")
        if len(self.obj_list.obj) > 0:
            # rospy.loginfo("Adding Obstacles from Obj list")
            # loop through all objects in the object list and plot them
            for obj in self.obj_list.obj:
                # TODO: TRANSFORM THE POINTS BEFORE RUNNING addObstacle
                # head = self.obj_list.header
                # obj = self.transform(obj, head)
                
                # acquire the x and y position of the obstacle w.r.t the robot base by making a simple rotation transform from the camera frame
                # TODO: in progress is using an actual transform, but this method is sufficient for roughly accurate plotting as long as camera yaw is unchanged
                # - this is also assuming that the Object message uses a PoseStamped as opposed to a Point, (matching the latest commit)
                obx = obj.point.pose.position.z
                oby = -obj.point.pose.position.x
                
                # placeholder for transform method again
                #obx, oby = self.transform(obx, oby, self.robot_name)
                
                # set radius of object to be plotted based on the width of the bounding box observed
                radius = (obj.width)/2
                # plot the obstacle onto the occupancy grid
                self.addObstacle(obx, oby, radius)      
        
    # publish the updated occupancy grid
    # - publish self.occ_grid after finished
    def gridPublisher(self):
        self.occGridPub.publish(self.occ_grid)
    
    # overall map editing function:
    def updateMap(self):
        # set up and clear the 20x20 map
        self.resetOccGrid()
        # populate the map with the observed obstacles
        self.addAllObstacles()
        # add default obstacle for testing purposes
        #self.addObstacle(5, 5, 7)
        #rospy.loginfo("Obstacle Added")
        # publish the mroap  
        self.gridPublisher()

# set up the node and then spin to operate on callbacks
if __name__=="__main__":
    # initialize node
    rospy.init_node("Ground_Truth_Localmaps")
    # instantiate the object plotter to begin mapping
    rospy.loginfo("[MAPLOC | obstacle_localmaps.py | " + str(sys.argv[1]) + "]: ground_truth_localmaps node, online") 
    ObstacleMap = ObjectPlotter()
    rospy.sleep(5.)
    # this node works entirely based on the callback functions, thus just need to spin in main loop after creating the plotter object    
    rospy.spin()