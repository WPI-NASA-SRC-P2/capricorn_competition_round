#include <operations/NavigationVisionAction.h> // Note: "Action" is appended
#include <operations/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<operations::navigationVisionAction> Client;
Client* client;

int main(int argc, char** argv)
{
  robot_name = argv[1];
  ros::init(argc, argv, "navigation_vision_client");
  Client client("navigation_vision", true); // true -> don't need ros::spin()
  client.waitForServer();

  operations::NavigationVisionGoal goal; 
  // Fill in goal here
  goal = argv[0]; //?????????

  client.sendGoal(goal);
  //client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Reached goal");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
