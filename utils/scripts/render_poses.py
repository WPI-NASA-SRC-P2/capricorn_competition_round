#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetPhysicsProperties, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

rospack = rospkg.RosPack()

models = []

def render_poses(pose_list):
	for i,marker in enumerate(pose_list.markers[:]):
		marker.pose.position.z = 10
		pose_list.markers[i] = marker

	print(f"Deleting {len(models)} spheres...")

	# Remove all current models
	for model in models:
		delete_model_client(model[0])

	print(f"Adding {len(pose_list.markers)} spheres...")

	# For each pose in pose_list
	for idx,model in enumerate(pose_list.markers):
		# Spawn a new sphere at the requested location
		spawn_model_client(
			model_name='sphere_' + str(idx),
			model_xml=open(rospack.get_path("utils") + '/urdf/sphere.urdf', 'r').read(),
			robot_namespace="sphere_" + str(idx),
			initial_pose=model.pose,
			reference_frame="heightmap"
		)

		models.append(["sphere_" + str(idx), model.pose])

	print("Models added.")

def set_model_pose(model_name, model_pose):
	state = ModelState()

	state.model_name = model_name
	state.pose = model_pose
	state.reference_frame = "heightmap"

	set_model_state_client(state)

if __name__ == "__main__":
	rospy.init_node("pose_vis")

	rospy.Subscriber("/vis_poses", MarkerArray, render_poses)

	while not rospy.is_shutdown():
		for model in models[:]:
			set_model_pose(model[0], model[1])
			rospy.sleep(0.25)