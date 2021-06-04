#!/usr/bin/env python3

'''
This node will use the IMU to detect if the rover is moving
 - if the rover is not moving, then store it's pose in this service ASAP
 - once rover starts moving again (detected by IMU), reset its odometry to the stored pose
'''

'''
if implementing as a class, then in __init__
'''
# set up map that stores the rover poses (total of 6)

# IMU subscriber

# class variable: IMU data

'''
class functions
'''
# IMU callback to set IMU class variable value

# function for calling resetOdom service

# function for determining if robot is in motion or not (IMU)

