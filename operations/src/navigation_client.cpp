#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <geometry_msgs/Twist.h>

// typedef for the Action Server
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

using namespace COMMON_NAMES;

/**
 * @brief Callback for the twist message of teleop message twist
 * 
 * @param twist twist message from teleop
 */
void chatterCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    // Action message goal
    operations::NavigationGoal goal;
    
    // Diverting values from twist to navigation
    goal.forward_velocity = twist->linear.x * 5;
    goal.sideways_velocity = twist->angular.z;
    
    client->sendGoal(goal);
    
    // Not really needed here. 
    // This can be better used as an actual feedback.
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
    // Robot Name from argument
    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_navigation_action_client";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Subscribing to teleop topic
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, chatterCallback);

    // initialize client
    client = new Client(NAVIGATION_ACTIONLIB, true);
    client->waitForServer();

    ros::spin();
    return 0;
  }
}