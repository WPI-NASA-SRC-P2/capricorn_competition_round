#include <operations/ExcavatorAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

int SLEEP_DURATION = 5; // The sleep duration

typedef actionlib::SimpleActionClient<operations::ExcavatorAction> Client;

/**
 * @brief The main method for excavator client
 * 
 * @param argc The number of arguments passed
 * @param argv The array of arguments passed
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "excavator_client");
  Client client(EXCAVATOR_ACTIONLIB, true);
  client.waitForServer();

  operations::ExcavatorGoal goal;
  //goal.task = START_DIGGING; // START_DIGGING = 1
  goal.task = CHECK_VOLATILE; // START_DIGGING = 1
  //goal.target.x = 0.7; // set of target digging values to the left of the excavator
  goal.target.x = 0;
  goal.target.y = 2;
  goal.target.z = 0;
  client.sendGoal(goal);

  std::string message1(client.getState().toString().c_str());
  ROS_INFO_STREAM("Current State: " + message1);
  client.waitForResult(ros::Duration(SLEEP_DURATION));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Yay! The scoop is now full");

  ros::Duration(SLEEP_DURATION).sleep(); // Delay between digging and unloading tasks

  goal.task = START_UNLOADING; // START_UNLOADING = 2
  //goal.target.x = 0.7; // set of target dumping values to the right of the excavator
  goal.target.x = 0;
  goal.target.y = -2;
  goal.target.z = 0;
  client.sendGoal(goal);
  
  std::string message2(client.getState().toString().c_str());
  ROS_INFO_STREAM("Current State: " + message2);
  client.waitForResult(ros::Duration(SLEEP_DURATION));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Yay! The scoop is now empty");

  std::string message3(client.getState().toString().c_str());
  ROS_INFO_STREAM("Current State: " + message3);

  return 0;
}