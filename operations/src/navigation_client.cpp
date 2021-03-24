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

/**
 * @brief Callback for the twist message of teleop message twist
 * 
 * @param twist twist message from teleop
 */
void chatterCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    // Action message goal
    operations::NavigationGoal goal;
    
    //Simple waypoint 2 meters in front of the robot
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
    goal.manual_driving = false;

    printf("Sending auto goal to actionlib server\n");
    client->sendGoal(goal);
    ros::Duration(3).sleep();

    // No manual driving
    goal.manual_driving = true;
    goal.forward_velocity = 0;
    goal.angular_velocity = -1;
    printf("Sending manual goal to actionlib server\n");
    client->sendGoal(goal);
    ros::Duration(3).sleep();

    printf("Re-sending auto goal to actionlib server\n");
    goal.manual_driving = false;
    goal.forward_velocity = 0;
    goal.angular_velocity = 0;
    client->sendGoal(goal);
}

int main(int argc, char** argv)
{
  // Ensure the robot name is passed in
  if (argc != 2 && argc != 4)
  {
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

    printf("Nav client: Instantiating client instance\n");

    // initialize client
    client = new Client(NAVIGATION_ACTIONLIB, true);
    printf("Waiting for server...\n");
    client->waitForServer();
    printf("Done waiting. Spinning\n");

    ros::spin();
    return 0;
  }
}