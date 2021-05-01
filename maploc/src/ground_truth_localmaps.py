#!/usr/bin/env python3

"""
This script runs the rosnode that generates local maps for the rovers to use 
"""

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
from math import pi
import sys
import tf_conversions
import tf2_ros

# object detection dependencies
from perception.msg import Object
from perception.msg import ObjectArray

"""
ObjectPlotter class organizes the local map operations. 
- Has subscriber for robot pose as well as for object list provided by object detection node. 
- Stores the local map and updates it during each callback of the object list. 
- Publishes the rover's local map during each callback to ensure map is up-to-date
"""
class ObjectPlotter:
    # constants corresponding to occupied or unoccupied cell values in occupancy grid
    OCCUPIED = 100
    UNOCCUPIED = 0

    def __init__(self):
        """
        Initialize the ObjectPlotter using the rover specified in the launch file
        - includes rover stereo odometry callback to keep track of rover's global position
        - includes object detection callback to store object list
        - includes map publisher to publish the local obstacle map
        """
        # specify the desired rover from input args from launch file
        robotname = str(sys.argv[1])
        rospy.loginfo(robotname)

        # set up odometry subscriber for global pose tracking
        self.robot_pos_sub = rospy.Subscriber("/" + robotname +"/camera/odom", Odometry, self.robotCb)
        self.robot_pose = PoseWithCovariance()
        
        # object detection data and subscriber
        self.obj_list = []
        self.object_sub = rospy.Subscriber("/capricorn/" + robotname + "/object_detection/objects", ObjectArray, self.objectCb)
        
        # occupancy grid data and publisher
        self.occ_grid = OccupancyGrid()
        self.occGridPub = rospy.Publisher("/capricorn/" + robotname + "object_detection_map", OccupancyGrid, queue_size=1)
    
    def robotCb(self, odom):
        """
        Subscriber callback to robot pose, updates robot pose in memory as it moves
        """
        self.robot_pose = odom.pose

    def objectCb(self, objlist):
        """
        Subscriber callback to object detection, updates detected obstacle list and then 
        updates the map (refreshes map, populates with obstacles, then publishes map)
        """
        self.obj_list = objlist.obj
        self.update_map()

    def initOccGrid(self):
        """
        Initialize/refresh the blank 20m x 20m map, centered on the robot 
        """
        # define dimensions of blank occupancy grid
        metadata = MapMetaData()
        # resolution units in (m/pixel), 0.05m/pixel matches rtabmap resolution
        metadata.resolution = 0.05 
        # sets the map to be 20m x 20m regardless of resolution
        metadata.width = int(20/metadata.resolution) 
        metadata.height = int(20/metadata.resolution)

        # define origin of blank occupancy grid (should correspond to robot pose)
        base_frame = Pose()
        # offset the map such that the bottom left of the map is -10m x -10m to the bottom left of the robot
        # thus making the robot pose w.r.t. the robot base frame (0,0) be the center of the map
        base_frame.position.x = -10
        base_frame.position.y = -10
        # set the origin of the occupancy grid
        metadata.origin = base_frame 
        
        # define the dimensions and origin of the occupancy grid
        self.occ_grid.info = metadata
        # initialize map data as all unoccupied spaces
        self.occ_grid.data = [UNOCCUPIED] * self.occ_grid.info.width*self.occ_grid.info.height

    def transform(self, old_x, old_y, robotname):
        """
        Transform the object list from camera frame to base frame
        (currently not used, using simplified transform in addAllObstacles instead)
        """
        rate = rospy.Rate(10)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        new_x = old_x
        new_y = old_y

        try:
            trans = tfBuffer.lookup_transform(robotname+'_base_footprint', robotname+"_left_camera_optical", rospy.Time())
            new_x = trans.transform.translation.x + old_x 
            new_y = trans.transform.translation.y + old_y
            rospy.loginfo('Transform works')
            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()

        return new_x, new_y
    
    def addObstacle(self, obx, oby, radius):
        """
        Plot single object on the occupancy grid w.r.t. the rover
        - center of the grid represents the rover's location
        """
        # plot everything w.r.t. center of the grid (since the origin of the map is transformed by (-10,-10) to bottom left)
        obx = obx + 10
        oby = oby + 10

        # plot a circle around the center point of the object
        for theta in range(0,360,2):
            theta = theta*pi/180

            # convert meters to pixels
            obx_p = obx/self.occ_grid.info.resolution
            oby_p = oby/self.occ_grid.info.resolution
            radius_p = radius/self.occ_grid.info.resolution

            # calculate the point on the circumference of the circle corresponding to the theta value
            circX = int((obx_p + radius_p*np.cos(theta)))  
            circY = int((oby_p + radius_p*np.sin(theta))) 
            
            # only add obstacle to grid if its index value is valid (else, not in view of robot anyways)
            if int(circY*self.occ_grid.info.width + circX) >= 0 and int(circY*self.occ_grid.info.width + circX) < self.occ_grid.info.width*self.occ_grid.info.height: 
                self.occ_grid.data[int(circY*self.occ_grid.info.width + circX)] = OCCUPIED
                
    def addAllObstacles(self):
        """
        Plot all objects in object detection object list
        - basically plot single object many times (across length of object list)
        """        
        for obj in self.obj_list:
            # transform the points from the camera frame to the base frame
            obx = obj.point.x
            oby = obj.point.z

            # placeholder for using the transform method if determined that the tf transform is required
            #obx, oby = self.transform(obx, oby, "small_scout_1")
            
            # radius of the object for plotting
            radius = (obj.width)/2

            # add the obstacle to the map
            self.addObstacle(obx, oby, radius)      

    def gridPublisher(self):
        """
        Publish the populated occupancy grid
        """
        self.occGridPub.publish(self.occ_grid)

    def updateMap(self):
        """
        Overall map producing function to populate and refresh the map
        """
        # initialize the occupancy grid (also refreshes the grid to remove old positions)
        self.initOccGrid()
        # add all of the detected objects to the occupancy grid
        self.addAllObstacles()
        # publish the populated occupancy grid
        self.gridPublisher()

if __name__=="__main__":
    
    # initialize the node 
    rospy.init_node("Ground_Truth_Localmaps")
    print("ground_truth_localmaps node, online")
    # Construct the map plotter object (uses the input argument of robot_name from the launch file)
    ObstacleMap = ObjectPlotter()
    # Spin to receive callback information. Code plots map based on each object list callback
    rospy.spin()

        
