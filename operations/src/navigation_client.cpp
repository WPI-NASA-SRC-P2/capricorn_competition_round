#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
using namespace COMMON_NAMES;

int main(int argc, char** argv)
{
  // // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  // if (argc != 4)
  // {
  //     // Displaying an error message for correct usage of the script, and returning error.
  //     ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>";);
  //     return -1;
  // }
  // else
  // {
    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_navigation_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    Client client(NAVIGATION_ACTIONLIB, true);
    client.waitForServer();

    operations::NavigationGoal goal;
    goal.forward_velocity = std::atof(argv[2]);
    goal.sideways_velocity = std::atof(argv[3]);
    
    client.sendGoal(goal);
    
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Yay! Robot should be moving\n");
    printf("Robot navigation state: %s\n", client.getState().toString().c_str());
    return 0;
  // }
}