#include <operations/HaulerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionClient<operations::HaulerAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hauler_bin_client");
  Client client(HAULER_ACTIONLIB, true);
  client.waitForServer();
  operations::HaulerGoal goal;
  goal.desired_angle = 1; //true
  client.sendGoal(goal);
  std::string message1(client.getState().toString().c_str());
  ROS_INFO_STREAM("Current State: " + message1);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Yay! The hauler is now empty");
  std::string message2(client.getState().toString().c_str());
  ROS_INFO_STREAM("Current State: " + message2);
  return 0;
}