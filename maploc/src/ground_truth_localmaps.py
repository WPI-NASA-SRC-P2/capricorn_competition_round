#!/usr/bin/env python3
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
from geometry_msgs.msg import PoseStamped
from math import pi
import sys
from perception.msg import Object
from perception.msg import ObjectArray
# tf2 reqs
import tf_conversions
import tf2_ros
 
class Object_Plotter:
    def __init__(self):
        # blank 20x20 occupancy grid fields
        # pose subscriber updates robot pose as it moves
        self.robot_name = str(sys.argv[1])
        #robot_name+'_base_footprint', robot_name+"_left_camera_optical"
        self.robot_pos_sub = rospy.Subscriber("/" + self.robot_name +"/camera/odom", Odometry, self.robot_cb)
        self.robot_pose = PoseWithCovariance()# value indicates the robot base frame pose
        self.occ_grid = OccupancyGrid()
        # object detection data and subscriber
        self.obj_list = ObjectArray()
        self.object_sub = rospy.Subscriber("/capricorn/" + self.robot_name + "/object_detection/objects", ObjectArray, self.object_cb)
        # occupancy grid publisher
        self.occGridPub = rospy.Publisher("/capricorn/" + self.robot_name + "object_detection_map", OccupancyGrid, queue_size=1)
    
    # NECESSARY FUNCTIONS
    # subscriber callback to robot pose, updates robot pose as it moves
    def robot_cb(self, odom):
        self.robot_pose = odom.pose
        # print(f'Robot Pose Received: {self.robot_pose}')
    
    # subscriber callback to object detection, updates detected obstacle list
    def object_cb(self, objlist):
        self.obj_list = objlist
        self.update_map()
        print(f'Map has updated: First object at {self.obj_list.obj[0].center}, No. of objects: {len(self.obj_list.obj)}')
    
    # initialize/refresh the blank 20x20 map centered on the robot 
    def init_occ_grid(self):
        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        metadata.resolution = 0.05 # (m/pixel) 0.05 matches rtabmap resolution
        metadata.width = int(20/metadata.resolution) # sets the map to be 20m x 20m regardless of resolution
        metadata.height = int(20/metadata.resolution)
        # # define origin of blank occupancy grid (should correspond to robot pose)
        # base_frame = Pose()
        # # offset the map such that the bottom left of the map is -10m x -10m to the bottom left of the robot
        # # thus making the robot pose be the center of the map
        # base_frame.position.x = self.robot_pose.pose.position.x - 10
        # base_frame.position.y = self.robot_pose.pose.position.y - 10
        # base_frame.position.z = self.robot_pose.pose.position.z
        # base_frame.orientation = self.robot_pose.pose.orientation
        # print("base frame x: ", base_frame.position.x)
        # print("base frame y: ", base_frame.position.y)
        # metadata.origin = base_frame 
        # define origin of blank occupancy grid (should correspond to robot pose)
        base_frame = Pose()
        # offset the map such that the bottom left of the map is -10m x -10m to the bottom left of the robot
        # thus making the robot pose be the center of the map
        base_frame.position.x = -10
        base_frame.position.y = -10
        #base_frame.position.z = self.robot_pose.pose.position.z
        #base_frame.orientation = self.robot_pose.pose.orientation
        print("base frame x: ", base_frame.position.x)
        print("base frame y: ", base_frame.position.y)
        metadata.origin = base_frame 
        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.info = metadata
        # initialize map data as all zeros
        self.occ_grid.data = [0] * self.occ_grid.info.width*self.occ_grid.info.height
    
    # transform the object list from camera frame to base frame
    # TODO: TEST THIS TO MAKE SURE IT WORKS PROPERLY
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
    def add_obstacle(self, obx, oby, radius):
        # plot everything w.r.t. center of the grid (robot is at center)
        obx = obx + 10
        oby = oby + 10
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
        for obj in self.obj_list.obj:
            # TODO: TRANSFORM THE POINTS BEFORE RUNNING ADD_OBSTACLE
            # head = self.obj_list.header
            # obj = self.transform(obj, head)
            obx = obj.point.z
            oby = -obj.point.x
            #obx, oby = self.transform(obx, oby, self.robot_name)
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
        # self.add_obstacle(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, 5)
        # self.add_obstacle(0, 0, 5)
        #print("robot pose: x: ", self.robot_pose.pose.position.x, " y: ", self.robot_pose.pose.position.y)
        self.gridPublisher()
#____________________________________________________________________________
# OLD CODE SNIPPETS BELOW: TODO: modify them to integrate with new class
if __name__=="__main__":

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