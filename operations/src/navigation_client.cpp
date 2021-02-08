#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

using namespace COMMON_NAMES;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    operations::NavigationGoal goal;
    goal.forward_velocity = twist->linear.x * 50;
    goal.sideways_velocity = twist->angular.z;
    
    client->sendGoal(goal);
    
    client->waitForResult(ros::Duration(5.0));
    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Yay! Robot should be moving\n");
    printf("Robot navigation state: %s\n", client->getState().toString().c_str());

}

int main(int argc, char** argv)
{
  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  if (argc != 4)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>";);
      return -1;
  }
  else
  {
    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_navigation_action_client";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, chatterCallback);

    client = new Client(NAVIGATION_ACTIONLIB, true);
    client->waitForServer();

    ros::spin();
    return 0;
  }
}