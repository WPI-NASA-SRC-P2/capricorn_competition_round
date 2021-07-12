#!/usr/bin/env python3 
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from state_machines.msg import robot_state_status
import sys

if __name__ == "__main__":
  # initialize node
  rospy.init_node("desired_state_node")
  # instantiate the object plotter to begin mapping
  rospy.loginfo("[STATE_MACHINES | desired_state_node.py | all_robots]: desired_state_node, online") 

  desired_state_pub = rospy.Publisher("/capricorn/robot_state_status", robot_state_status, queue_size=1)

  input('Enter for Search Function')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_scout_1"
  desired_state.robot_current_state = 0
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)

  input("MACRO STATE CHANGED TO SCOUT_WAITING")

  desired_state = robot_state_status()
  desired_state.robot_name = "small_scout_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)

#####################################################################################

  input('Enter for Locate Function')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_scout_1"
  desired_state.robot_current_state = 3
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator Go to Loc')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 12
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Undock Function')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_scout_1"
  desired_state.robot_current_state = 4
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Park and Pub')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 13
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input("MACRO STATE CHANGED TO EXCAVATING")

  desired_state = robot_state_status()
  desired_state.robot_name = "small_scout_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)

  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)


#####################################################################################


  input('Enter for Hauler finish Dumping')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 25
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input("MACRO STATE DUMPING FINISHED")

  input('Enter for Idle Function')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  

#####################################################################################


  input('Enter for Hauler Go To Excavator')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 26
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator Pre-Park Maneuver')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 14
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Hauler Park at Excavator')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 27
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator DnD')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 15
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input("MACRO STATE DUMPING FINISHED")

  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator Idle')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  

#####################################################################################


  input('Enter for Hauler Park at Proc Plant')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 38
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator Park at Proc Plant')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 21
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Hauler Park at Proc Plant')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_hauler_1"
  desired_state.robot_current_state = 39
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  
  input('Enter for Excavator Park at Proc Plant')
  desired_state = robot_state_status()
  desired_state.robot_name = "small_excavator_1"
  desired_state.robot_current_state = 22
  desired_state.current_state_done = True
  desired_state.last_state_succeeded = True
  desired_state_pub.publish(desired_state)
  

  # this node works entirely based on the callback functions, thus just need to spin in main loop after initializing    