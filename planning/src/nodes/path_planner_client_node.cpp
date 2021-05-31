#include "ros/ros.h"
#include "planning/trajectory.h"
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_client");

  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  std::string robot_name(argv[1]);

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<planning::trajectory>("/capricorn/" + robot_name + "/trajectoryGenerator");

  client.waitForExistence();

  planning::trajectory srv;

  //probably needs to not use argument
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = robot_name + "_small_chassis";
  pose.pose.position.x = 1;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 1;
  pose.pose.orientation.w = 0;
  srv.request.targetPose = pose;

  //printf(client.isValid());

  if (client.call(srv))
  {
    ROS_INFO("client call succeeded");
    for(auto i : srv.response.trajectory.waypoints) {
      ROS_INFO("%f, %f\n", i.pose.position.x, i.pose.position.y);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service trajectory generator");
    return 1;
  }

  return 0;
}
