#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from state_machines.msg import robot_desired_state
import sys

def targetPoseCallback(target_odom):
  rospy.loginfo("pose callback")

  # convert robot odometry to output message type
  desired_state = robot_desired_state()
  desired_state.robot_name = driving_robot_name
  desired_state.robot_desired_state = driving_robot_desired_state

  target_pose = PoseStamped()
  target_pose.header.frame_id = "odom"
  target_pose.pose = target_odom.pose.pose

  desired_state.goal_pose = target_pose

  # publish robot_desired_state message
  desired_state_pub.publish(desired_state)

if __name__=="__main__":
  # initialize node
  rospy.init_node("desired_state_node")
  # instantiate the object plotter to begin mapping
  rospy.loginfo("[STATE_MACHINES | desired_state_node.py | " + str(sys.argv[1]) + "]: desired_state_node, online") 

  # take input arguments describing target robot for pose obtaining (default is small_scout_1)
  target_robot_name = str(sys.argv[1])
  driving_robot_name = str(sys.argv[2])
  driving_robot_desired_state = int(sys.argv[3])
  
  desired_state_pub = rospy.Publisher("/capricorn/robot_desired_state", robot_desired_state, queue_size=1)

  # subscribe to target robot odometries
  if target_robot_name != "":
    robot_pos_sub = rospy.Subscriber("/" + target_robot_name +"/camera/odom", Odometry, targetPoseCallback)
  else:
    # if no target robot (i.e., desired state is GoToProcPlant), publish desired goal
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      desired_state = robot_desired_state()
      desired_state.robot_name = driving_robot_name
      desired_state.robot_desired_state = driving_robot_desired_state
      desired_state_pub.publish(desired_state)
      r.sleep()

  # this node works entirely based on the callback functions, thus just need to spin in main loop after initializing    
  rospy.sleep(5.)
  rospy.spin()