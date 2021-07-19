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
import tf2_geometry_msgs

# MAP CONSTANTS

MAP_RESOLUTION = 0.2
MAP_WIDTH  = 40
MAP_HEIGHT = 40
MAP_X_OFFSET = 20
MAP_Y_OFFSET = 20
OCCUPIED = 100
UNOCCUPIED = 0
OBSTACLE_SCALER = 4
ALL_OBSTACLE_LIST = []
OBSTACLE_REMEMBRING_FRAMES = 30 #10
SKIP_FRAME = 5 #3
CURRENT_FRAME = SKIP_FRAME + 1 # To make sure the first frame is plotted
 
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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
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
        global MAP_RESOLUTION                         # (m/pixel) 0.05 matches rtabmap resolution
        global MAP_WIDTH
        global MAP_HEIGHT
        global MAP_X_OFFSET
        global MAP_Y_OFFSET
        global UNOCCUPIED
        

        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        metadata.resolution = MAP_RESOLUTION
        metadata.width  = int(MAP_WIDTH/metadata.resolution) # sets the map to be 40m x 40m regardless of resolution
        metadata.height = int(MAP_HEIGHT/metadata.resolution)

        # we use a blank pose centered in the map as opposed to the robot pose from odom at the moment. Everything is still w.r.t. robot base frame so the relative positions are correct
        base_frame = Pose()
        # offset the map such that the bottom left of the map is -20m x -20m to the bottom left of the robot
        # thus making the robot pose be the center of the map
        base_frame.position.x = -MAP_X_OFFSET
        base_frame.position.y = -MAP_Y_OFFSET
        metadata.origin = base_frame 

        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.header.frame_id = self.robot_name + '_base_footprint'
        self.occ_grid.info = metadata
        # initialize map data as all zeros
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

    def transform(self, input_pose, target_frame):    
        # changes copied from Galen's branch tf_fix  
        # https://github.com/galenbr/capricorn_competition_round/tree/tf_fix  
        tries = 20
        # print("inside transform function")
        while(tries > 0):
            try:
                transform = self.tf_buffer.lookup_transform(target_frame,
                                       input_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
                return tf2_geometry_msgs.do_transform_pose(input_pose, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.1)
            tries -= 1

    # plot single object on the occupancy grid
    # 100 = obstacle, -1 = unexplored, 0 = free
    def addObstacle(self, obx, oby, radius):
        # plot everything w.r.t. center of the grid
        global MAP_X_OFFSET
        global MAP_Y_OFFSET
        global OCCUPIED

        obx = obx + MAP_X_OFFSET
        oby = oby + MAP_Y_OFFSET
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
                # rospy.loginfo("printing within map")
                self.occ_grid.data[int(pos*(self.occ_grid.info.width) + circX)] = OCCUPIED
                pos += 1

    
    # plot all objects in object list
    # - basically plot single object many times (across length of object list)
    def addAllObstacles(self):
        global OBSTACLE_SCALER
        global ALL_OBSTACLE_LIST
        global CURRENT_FRAME
        global SKIP_FRAME

        # check if there are any observed objects
        #rospy.loginfo("Adding Obstacles from Obj list")
        if len(self.obj_list.obj) > 0:
            # rospy.loginfo("Adding Obstacles from Obj list")
            # loop through all objects in the object list and plot them
            ALL_OBSTACLE_LIST.append([])
            if(CURRENT_FRAME>SKIP_FRAME):
                CURRENT_FRAME = 0
                for obj in self.obj_list.obj:
                    obj_wrt_map = self.transform(obj.point, "map")
                    obj_wrt_map.pose.position.z = obj.width
                    ALL_OBSTACLE_LIST[-1].append(obj_wrt_map)
            CURRENT_FRAME += 1
            if(len(ALL_OBSTACLE_LIST) > OBSTACLE_REMEMBRING_FRAMES):
                del ALL_OBSTACLE_LIST[0]

        for frame in ALL_OBSTACLE_LIST:
            for obj in frame:
                    # TODO: TRANSFORM THE POINTS BEFORE RUNNING addObstacle
                    # head = self.obj_list.header
                    # obj = self.transform(obj, head)
                    
                    # acquire the x and y position of the obstacle w.r.t the robot base by making a simple rotation transform from the camera frame
                    # TODO: in progress is using an actual transform, but this method is sufficient for roughly accurate plotting as long as camera yaw is unchanged
                    # - this is also assuming that the Object message uses a PoseStamped as opposed to a Point, (matching the latest commit)
                    obj_wrt_robot = self.transform(obj, self.robot_name + "_base_footprint")
                    obx = obj_wrt_robot.pose.position.x
                    oby = obj_wrt_robot.pose.position.y
                    
                    # placeholder for transform method again
                    #obx, oby = self.transform(obx, oby, self.robot_name)
                    
                    # set radius of object to be plotted based on the width of the bounding box observed
                    radius = (obj.pose.position.z + OBSTACLE_SCALER)/2 ####increasing the obstacle size
                    # plot the obstacle onto the occupancy grid
                    if(radius < 10) :
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