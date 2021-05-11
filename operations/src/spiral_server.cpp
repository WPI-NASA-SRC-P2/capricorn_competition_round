#include "ros/ros.h"
#include <operations/navigation_server.h>
#include <operations/navigation_algorithm.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<operations::NavigationAction> g_client;
using namespace COMMON_NAMES;
geometry_msgs::PoseStamped robot_pose_;
bool cb_init = false;

void updateRobotPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	robot_pose_.header = msg->header;
	robot_pose_.pose = msg->pose.pose;
  // ROS_WARN_STREAM(cb_init);
  cb_init = true;
  // ROS_WARN_STREAM(cb_init);
	return;
}

geometry_msgs::Quaternion getOrientation(const geometry_msgs::Point& p0, const geometry_msgs::Point& p2)
{
  float d_x = p2.x - p0.x;
  float d_y = p2.y - p0.y;

  double yaw = atan2(d_y, d_x);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( 0, 0, yaw);

  geometry_msgs::Quaternion quat_msg;
  tf2::convert(myQuaternion, quat_msg);
  return quat_msg;
}

geometry_msgs::PointStamped getCenterOfRotation(const std::vector<geometry_msgs::PointStamped>& spiral_points)
{
  std::vector<geometry_msgs::PointStamped> temp_points;
  temp_points.push_back(spiral_points.at(0));
  temp_points.push_back(spiral_points.at(1));
  temp_points.push_back(spiral_points.at(2));
  
  double radius = NavigationAlgo::getRadiusOfThreePointsCircle(temp_points);
  geometry_msgs::PointStamped center_of_rot;
  center_of_rot.point.y = radius;
  return center_of_rot;
}

int main(int argc, char** argv)
{
  if(argc < 2)
  {
      ROS_ERROR_STREAM("This node must be launched with the robotname and the target object detection class that you want to go passed asfirst and second command line arguments!");
      return -1;
  }

  std::string robot_name(argv[1]);
  ros::init(argc,argv,robot_name + "_spiral_tester");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, updateRobotPose);
  g_client client(CAPRICORN_TOPIC + robot_name + "/" + NAVIGATION_ACTIONLIB, true);
  
  // ROS_WARN_STREAM("Waiting for server");
  client.waitForServer();
  // ROS_WARN("Waiting for cheat odom");
  
  while (ros::ok() && !cb_init)
  {
    ros::Duration(0.1).sleep();
    ROS_WARN_STREAM(cb_init);
    ros::spinOnce();
  }

  // ROS_WARN("Spiral Server Started");
  geometry_msgs::PointStamped zero_point;
  zero_point.header.frame_id = MAP;
  static std::vector<geometry_msgs::PointStamped> spiral_points = NavigationAlgo::getNArchimedeasSpiralPoints(zero_point, 100, 1);
  double last_dist = 0.0;
  bool going_to_goal = false;

  while(spiral_points.size()>=3 && ros::ok())
  {
    double dist = NavigationAlgo::changeInPosition(robot_pose_, spiral_points.at(1));
    const double CHECKPOINT_THRESHOLD = 2.0;
    bool done_driving = client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    if(going_to_goal && !done_driving)
    {
      last_dist = dist;
      ros::Duration(1).sleep();
      continue;
    }
    else
      going_to_goal = false;

    if (dist < CHECKPOINT_THRESHOLD)
    {    
      geometry_msgs::PointStamped center_of_rot = getCenterOfRotation(spiral_points);
      spiral_points.erase(spiral_points.begin());
      operations::NavigationGoal goal;
      goal.drive_mode = NAV_TYPE::REVOLVE;
      goal.point = center_of_rot;
      goal.forward_velocity = 2.0;
      ROS_WARN_STREAM("Circular Motion radius: "<<center_of_rot.point.y<<"\tdist: "<<dist);
      client.sendGoal(goal);
      last_dist = dist;
      ros::Duration(1).sleep();
    }
    else if (dist > last_dist)
    {
      geometry_msgs::Point point_0, point_2;
      point_0 = spiral_points.at(0).point;
      point_2 = spiral_points.at(2).point;

      ROS_WARN_STREAM("Going to goal dist:"<<dist);
      

      operations::NavigationGoal goal;
      goal.drive_mode = NAV_TYPE::GOAL;
      goal.pose.pose.position = spiral_points.at(1).point;
      goal.pose.pose.orientation = getOrientation(point_0, point_2);
      goal.pose.header.frame_id = MAP;
      client.sendGoal(goal);
      ros::Duration(1).sleep();
      going_to_goal = true;
    }
    else
    {
      ROS_WARN_STREAM("Dist:"<<dist<<"\tLast Dist:"<<last_dist);
      last_dist = dist;
      ros::Duration(1).sleep();
    }
    ros::spinOnce();
  }

	ROS_WARN_STREAM("Spiraling Complete");
	return 0;
  }