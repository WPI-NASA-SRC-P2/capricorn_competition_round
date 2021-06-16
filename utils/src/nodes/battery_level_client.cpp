#include "ros/ros.h"
#include "utils/battery_deadlines.h"
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_deadlines_client");

  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  std::string robot_name(argv[1]);

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<utils::battery_deadlines>("/capricorn/" + robot_name + "/deadlines_server");

  client.waitForExistence();

  utils::battery_deadlines srv;

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
