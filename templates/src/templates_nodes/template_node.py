#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg    import Float64
from sensor_msgs.msg import Imu

robot_name = ""
UPDATE_HZ = 10.0

# Put Doxygen comment block here
def imu_callback(data):
   pass

# Put Doxygen comment block here
def template():
    update_rate = rospy.Rate(UPDATE_HZ)

    #Get robot name from parameters, and store it globally.
    robot_name = sys.argv[1]

    #Initialize subscribers. This example subscribes to this robot's imu topic.
    rospy.Subscriber('/' + robot_name + '/imu', Imu, imu_callback)

    #Initialize publishers. This example create publishers to command each wheel on this robot.
    fl_speed = rospy.Publisher('/' + robot_name + '/front_left_wheel/drive/command/velocity', Float64, queue_size=10)
    fr_speed = rospy.Publisher('/' + robot_name + '/front_right_wheel/drive/command/velocity', Float64, queue_size=10)
    bl_speed = rospy.Publisher('/' + robot_name + '/back_left_wheel/drive/command/velocity', Float64, queue_size=10)
    br_speed = rospy.Publisher('/' + robot_name + '/back_right_wheel/drive/command/velocity', Float64, queue_size=10)
    
    #Initialize outside of try/except
    wheel_effort = 20.0

    try:
        wheel_effort = rospy.get_param('~wheel_effort')
        print("Wheel effort set to " + str(wheel_effort) + " by parameter for " + robot_name)
    except KeyError:
        print("No wheel_effort param set for " + robot_name + ", defaulting to 20.0.")
        wheel_effort = 0.0
        
    while not rospy.is_shutdown():
        
        #Publish a constant wheel speed to each wheel.
        speed = Float64()
        speed.data = wheel_effort

        fl_speed.publish(speed)
        fr_speed.publish(speed)
        bl_speed.publish(speed)
        br_speed.publish(speed)

        #Rate limit the loop to UPDATE_HZ
        update_rate.sleep()


if __name__ == "__main__":
    #Initialize the node
    rospy.init_node('template', anonymous = True)
    
    template()