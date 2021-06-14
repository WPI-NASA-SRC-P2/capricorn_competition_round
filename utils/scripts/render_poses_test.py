#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header

if __name__ == "__main__":
	rospy.init_node("render_test")

	pub = rospy.Publisher("/vis_poses", MarkerArray, queue_size=10)

	markers = MarkerArray()

	markers.markers.append(Marker(pose=Pose(position=Point(z=10.0))))

	input()

	pub.publish(markers)

	print("Published poses")
