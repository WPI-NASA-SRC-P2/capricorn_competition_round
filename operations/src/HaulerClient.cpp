#include <operations/HaulerAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<operations::HaulerAction> Client;

int main(int argc, char** argv)
{ // Use as hauler_bin_pos desires_pos
  ros::init(argc, argv, "hauler_bin_client");
  Client client("hauler_bin", true); // true -> don't need ros::spin()
  client.waitForServer();
  operations::HaulerGoal goal;
  goal.desired_pos = 1;
  // Fill in goal here
  client.sendGoal(goal);
  printf("Current State: %s\n", client.getState().toString().c_str());
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The hauler is now empty");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}