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
    goal.forward_velocity = 0;
    goal.angular_velocity = 0;
    
    client->sendGoal(goal);
    
    // // Not really needed here. 
    // // This can be better used as an actual feedback.
    // client->waitForResult(ros::Duration(5.0));
    // if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //   printf("Yay! Robot should be moving\n");
    // printf("Robot navigation state: %s\n", client->getState().toString().c_str());

}

int main(int argc, char** argv)
{

  std::cout << argc << std::endl;
  // Ensure the robot name is passed in
  if (argc != 2)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
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

    std::cout << "Nav client: Instantiating client instance" << std::endl;

    // initialize client
    client = new Client(NAVIGATION_ACTIONLIB, true);
    std::cout << "Waiting for server..." << std::endl;
    client->waitForServer();
    std::cout << "Done waiting. Spinning" << std::endl;

    ros::spin();
    return 0;
  }
}