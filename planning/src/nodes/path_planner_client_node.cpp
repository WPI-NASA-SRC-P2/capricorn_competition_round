#include "ros/ros.h"
#include "planning/trajectory.h"
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_client");

  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<planning::trajectory>("trajectory_client");

  planning::trajectory srv;

  //probably needs to not use argument
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 9;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 0;
  

  srv.request.targetPose = pose;

  if (client.call(srv))
  {
    ROS_INFO("client call succeeded");
  }
  else
  {
    ROS_ERROR("Failed to call service trajectory generator");
    return 1;
  }

  return 0;
}
