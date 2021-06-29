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
Client *client;

using namespace COMMON_NAMES;

std::string robot_name;

/**
 * @brief Callback for the twist message of teleop message twist
 *        Inspite of us commenting again and again that callback 
 *        must be short, this in fact can be longer callbacks
 * 
 *        We only trigger these when needed, and when called on 
 *        top of each other, they need to be overwriting the 
 *        ongoing functionality
 * 
 * @param twist twist message from teleop
 */
void teleopCB(const geometry_msgs::Twist::ConstPtr &twist)
{
  // Action message goal
  operations::NavigationGoal goal;
  goal.point.header.frame_id = robot_name + ROBOT_CHASSIS;

  if (twist->linear.x == 0 || twist->angular.z == 0)
  {
    // Manual driving
    goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = twist->linear.x;
    goal.angular_velocity = twist->angular.z;
  }
  else
  {
    // Radial Turn
    goal.drive_mode = NAV_TYPE::REVOLVE;

    // Hardcoded for a good enough radial turn
    // Taking radius sign depending on the direction of turn
    double radial_turn_radius = std::copysign(2.0, twist->angular.z);
    goal.point.point.y = radial_turn_radius;
    goal.forward_velocity = twist->linear.x;
  }

  printf("Teleop twist forward: %f\n", twist->linear.x);

  client->sendGoal(goal);
  ros::Duration(0.1).sleep();
}

/**
 * @brief Callback for the navigation testing topic
 * 
 * @param twist geometry_msgs::Point for the goal 
 */
void navigationCB(const geometry_msgs::Point::ConstPtr &goal_point)
{
    // Action message goal
    operations::NavigationGoal goal;
    
    //Simple waypoint x meters in front of the robot
    geometry_msgs::PoseStamped t1;
    t1.header.frame_id =  "map";
    t1.header.stamp = ros::Time::now();
    t1.pose.position.x = goal_point->x;
    t1.pose.position.y = goal_point->y;
    t1.pose.position.z = 0;

    t1.pose.orientation.w = 1;
    t1.pose.orientation.x = 0;
    t1.pose.orientation.y = 0;
    t1.pose.orientation.z = 0;

    goal.pose = t1;

    // goal.point.point.x = goal_point->x;
    // goal.forward_velocity = goal_point->z;

    // goal.point.header.frame_id = robot_name + ROBOT_CHASSIS;

    goal.drive_mode = NAV_TYPE::GOAL_SMOOTH;

    printf("Sending auto goal to actionlib server\n");
    client->sendGoal(goal);
    // ros::Duration(0.1).sleep();

    // printf("Re-sending auto goal to actionlib server\n");
    // goal.manual_driving = false;
    // goal.forward_velocity = 0;
    // goal.angular_velocity = 0;
    // client->sendGoal(goal);
}

int main(int argc, char **argv)
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
    ros::Subscriber navigation_sub = nh.subscribe("/capricorn/" + robot_name + "/navigation_tester_topic", 1000, navigationCB);
    ros::Subscriber teleop_sub = nh.subscribe("/cmd_vel", 1000, teleopCB);

    printf("Nav client: Instantiating client instance\n");

    // initialize client
    client = new Client(CAPRICORN_TOPIC + robot_name + "/" + NAVIGATION_ACTIONLIB, true);

    printf("Waiting for server...\n");

    bool serverExists = client->waitForServer(ros::Duration(5.0));

    if (!serverExists)
    {
      ROS_ERROR_STREAM("Server does not exist! Exiting.\n");
      return -1;
    }

    printf("Done waiting. Spinning\n");

    ros::spin();
    return 0;
  }
}