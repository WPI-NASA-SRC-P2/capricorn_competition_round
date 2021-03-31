#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_client");

  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<path_planner_client_node::TrajectoryGenerator>("trajectory_client");

  planning::trajectory srv;

  //probably needs to not use argument
  srv.request.targetPose = argv[1];

  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.trajectory);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
