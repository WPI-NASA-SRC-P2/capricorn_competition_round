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
import tf2_msgs.msg
# from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import tf2_geometry_msgs


class ObjectPlotter:
    def __init__(self):
        # blank 20x20 occupancy grid fields
        # pose subscriber updates robot pose as it moves
        self.robot_name = str(sys.argv[1])
        # robot_name+'_base_footprint', robot_name+"_left_camera_optical"
        self.robot_pos_sub = rospy.Subscriber("/" + self.robot_name + "/camera/odom", Odometry, self.robotCb)
        # value indicates the robot base frame pose
        self.robot_pose = PoseWithCovariance()
        # actual map stored in memory for updating
        self.occ_grid = OccupancyGrid()        
        # object detection data and subscriber
        self.obj_list = ObjectArray()
        self.object_sub = rospy.Subscriber("/capricorn/" + self.robot_name + "/object_detection/objects", ObjectArray, self.objectCb)
        # occupancy grid publisher
        self.occGridPub = rospy.Publisher("/capricorn/" + self.robot_name + "/object_detection_map", OccupancyGrid, queue_size=1)
        #publisher for transformed frame
        # self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    # subscriber callback to robot pose, updates robot pose as it moves
    def robotCb(self, odom):
        self.robot_pose = odom.pose
        # print(f'Robot Pose Received: {self.robot_pose}')
        # TODO: FOR TESTING PURPOSES ONLY:
        # rospy.loginfo("Robot Pose Received")
        # self.new_frame()
        self.updateMap()

    # subscriber callback to object detection, updates detected obstacle list
    def objectCb(self, objlist):
        self.obj_list = objlist
        # self.updateMap()
        # useful for debugging any issues with object detection
        # rospy.loginfo(f'Map has updated: First object at {self.obj_list.obj[0].center}, No. of objects: {len(self.obj_list.obj)}')

    # initialize/refresh the blank 20x20 map centered on the robot
    def resetOccGrid(self):
        metadata = MapMetaData()
        # define dimensions of blank occupancy grid
        # resolution units are (m/pixel), value of 0.05 matches rtabmap resolution
        metadata.resolution = 0.25
        # sets the map to be 20m x 20m regardless of resolution
        metadata.width = int(40 / metadata.resolution)  
        metadata.height = int(20 / metadata.resolution)

        # offset the map such that the bottom left of the map is -10m x -10m to the bottom left of the robot
        # thus making the robot pose be the center of the map
        #Listening to transform between 
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        count = 0
        rate = rospy.Rate(10.0)
        while count < 5:
            try:
                trans_map = tf_buffer.lookup_transform(
                self.robot_name + "_base_footprint",
                self.robot_name + "_map_origin",
                # grabs most recent transform
                rospy.Time(0),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            count += 1
          
        # complete the offset using the transform  
        origin_pose = tf2_geometry_msgs.do_transform_pose(self.robot_pose, trans_map)
        # set the origin such that the center of the map correspond the robot base_footprint frame
        metadata.origin = origin_pose.pose

        # set up the 2D OccupancyGrid with robot base frame as origin
        # - the origin is at bottom left. The origin will be translated to the center of the map in plotting function and publishing
        self.occ_grid.info = metadata
        # initialize map data as all zeros
        UNOCCUPIED = 0
        self.occ_grid.data = ([UNOCCUPIED] * self.occ_grid.info.width * self.occ_grid.info.height)
        self.occ_grid.header.frame_id = self.robot_name + "_base_footprint"
        # self.occ_grid.header.frame_id = "map"
        # self.occ_grid.header.frame_id = self.robot_name + "_map_center"

    # transform the object list from camera frame to base frame
    # TODO: This function is in progress, it is not called in the code at the moment so it should not impact the launch
    def transform(self, old_x, old_y):
        # rate = rospy.Rate(10)
        old_pose = PoseStamped()
        old_pose.pose.position.x = old_x
        old_pose.pose.position.y = old_y
        old_pose.pose.orientation.w = 1.0
        old_pose.header.frame_id = self.robot_name + "_map_origin"
        old_pose.header.stamp = rospy.Time(0)

        #Listening to transform between 
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        count = 0
        rate = rospy.Rate(10.0)
        while count < 5:
            try:
                trans_obj = tf_buffer.lookup_transform(
                self.robot_name + "_map_origin",
                self.robot_name + "_base_footprint",
                # grabs most recent transform
                rospy.Time(0),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            count += 1
          
        # origin_pose = self.robot_pose.pose         
        centered_obj = tf2_geometry_msgs.do_transform_pose(old_pose, trans_obj)
        
        return centered_obj.pose.position.x, centered_obj.pose.position.y

    # plot single object on the occupancy grid
    # 100 = obstacle, -1 = unexplored, 0 = free
    def addObstacle(self, obx, oby, radius):
        # plot everything w.r.t. center of the grid
        # obx = obx + 10
        # oby = oby + 10
        # convert meters to pixels
        obx_p = obx / self.occ_grid.info.resolution
        oby_p = oby / self.occ_grid.info.resolution
        radius_p = radius / self.occ_grid.info.resolution
        # rospy.loginfo("Im here")
        # fill in obstacles found within localmaps
        for val in range(0, 901):
            # counter-clockwise (right-bound)
            pos_theta = (val / 5.0) * pi / 180
            # clockwise (left-bound)
            neg_theta = -(val / 5.0) * pi / 180
            # # swap obx_p and oby_p
            # left_bound = int(obx_p + radius_p*np.sin(neg_theta))
            # right_bound = int(obx_p + radius_p*np.sin(pos_theta))
            # circY = int(oby_p + radius_p*np.cos(pos_theta))
            # swap obx_p and oby_p
            up_bound = int(oby_p + radius_p * np.sin(neg_theta))
            down_bound = int(oby_p + radius_p * np.sin(pos_theta))
            circX = int(obx_p + radius_p * np.cos(pos_theta))
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
                self.occ_grid.data[int(pos * (self.occ_grid.info.width) + circX)] = OCCUPIED
                pos += 1

    # plot all objects in object list
    # - basically plot single object many times (across length of object list)
    def addAllObstacles(self):
        # check if there are any observed objects
        # rospy.loginfo("Adding Obstacles from Obj list")
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
                obx, oby = self.transform(obx, oby)

                # set radius of object to be plotted based on the width of the bounding box observed
                radius = (obj.width) / 2
                # plot the obstacle onto the occupancy grid   
                if(radius >= 0.68):  #change the radius tolerance as per need.
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
        # self.addObstacle(0, 0, 5)
        # rospy.loginfo("Obstacle Added")
        # publish the mroap
        self.gridPublisher()


# set up the node and then spin to operate on callbacks
if __name__ == "__main__":
    # initialize node
    rospy.init_node("Ground_Truth_Localmaps")
    rospy.loginfo("ground_truth_localmaps node, online")
    # instantiate the object plotter to begin mapping
    ObstacleMap = ObjectPlotter()
    # this node works entirely based on the callback functions, thus just need to spin in main loop after creating the plotter object
    rospy.spin()
