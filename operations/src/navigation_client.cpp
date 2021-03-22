#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// typedef for the Action Server
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

using namespace COMMON_NAMES;

std::string robot_name;

//tf2_ros::Buffer buffer;
//tf2_ros::TransformListener* listener;

/**
 * @brief Callback for the twist message of teleop message twist
 * 
 * @param twist twist message from teleop
 */
void chatterCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    // Action message goal
    operations::NavigationGoal goal;
    
    // No manual driving
    goal.forward_velocity = 0;
    goal.angular_velocity = 0;
    
    // Simple waypoint 2 meters in front of the robot
    geometry_msgs::PoseStamped t1;
    t1.header.frame_id = robot_name + "_small_chassis";
    t1.pose.position.x = 2.0;
    t1.pose.position.y = 2.0;
    t1.pose.position.z = 0;

    t1.pose.orientation.w = 1;
    t1.pose.orientation.x = 0;
    t1.pose.orientation.y = 0;
    t1.pose.orientation.z = 0;

    //geometry_msgs::PoseStamped map_frame_waypoint = buffer.transform(t1, MAP);

    goal.pose = t1;

    printf("Sending goal to actionlib server");
    client->sendGoal(goal);
}

int main(int argc, char** argv)
{

  std::cout << argc << std::endl;
  // Ensure the robot name is passed in
  if (argc != 2 && argc != 4)
  {
      std::cout << argc << std::endl;
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
      return -1;
  }
  else
  {
    // Robot Name from argument
    robot_name = argv[1];
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

    //listener = new tf2_ros::TransformListener(buffer);

    ros::spin();
    return 0;
  }
}