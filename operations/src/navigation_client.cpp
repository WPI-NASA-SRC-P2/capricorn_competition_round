#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_action_client");
  Client client("navigation", true);

  client.waitForServer();
  operations::NavigationGoal goal;
  goal.velocity = 50;
  
  client.sendGoal(goal);
  
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! Robot should be moving\n");
  printf("Robot navigation state: %s\n", client.getState().toString().c_str());
  return 0;
}