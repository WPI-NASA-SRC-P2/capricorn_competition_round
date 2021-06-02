#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <operations/SolarRechargeAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <srcp2_msgs/VolSensorMsg.h>   //need help
#include <utils/common_names.h>

// typedef for the Action Server and Client
typedef actionlib::SimpleActionServer<operations::SolarRechargeAction> SolarRechargeServer;
typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
NavigationClient_ *navigation_client_;

using namespace COMMON_NAMES;

double ROTATION_VELOCITY = 0.2;
double DRIVING_VELOCITY = 0.2;
double MAX_DETECT_DIST = 2.0;
double VOLATILE_DISTANCE_THRESHOLD = 0.005;
int FLIP_ROTATION_COUNT_MAX = 2;
int REPEAT_COUNT_MAX = 2;
bool near_volatile_ = false;
bool new_message_received = false;
bool g_robot_solar_okay = false;
double volatile_distance_;
std::string robot_name_;

/**
 * @brief enum for rotation direction
 *        When clockwise, send direction as it is
          When Anticlockwise, flip the direction with -1
 * 
 */
enum DrivingDirection
{
  POSITIVE = 1,
  NEGATIVE = -1
};

enum DrivingMode
{
  ROTATE_ROBOT = 0,
  DRIVE_ROBOT_STRAIGHT
};

/**
 * @brief Rotate the robot in the given direction
 * 
 * @param rotate_direction   direction of rotation
 */
void rotateRobot(const DrivingDirection rotate_direction, const float rotational_velocity_multiplier)
{
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = rotate_direction * ROTATION_VELOCITY * rotational_velocity_multiplier;

  navigation_client_->sendGoal(goal);
  ros::Duration(0.1).sleep();
}

/**
 * @brief Stop the robot
 * 
 *
 */
void stopRobot()
{
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = 0;

  navigation_client_->sendGoal(goal);
  ros::Duration(0.5).sleep();
}


/**
 * @brief Actionlib callback to rotate the robot until indicating that there was solar charge
 * 
 * @param goal 
 * @param server 
 */
void solarRecharge(const operations::SolarRechargeGoalConstPtr &goal, SolarRechargeServer *server)
{
  DrivingDirection driving_direction = POSITIVE;

  ROS_INFO("Starting transitiong to solar recharge");

  rotateRobot(driving_direction, 1.0);
  while(!g_robot_solar_ok && ros::ok()){} // while there is no solar keep rotating 
  stopRobot();

  ROS_INFO("Now Solar Recharging");

  server->setSucceeded();
}

/**
 * @brief updates the Status of solar_ok
 * 
 * @param msg msg from system monitor
 */
void updateRobotSolarOk(const srcp2_msgs::SystemMonitorMsg &msg)
{
  g_robot_solar_okay = msg->sensor_ok;
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
    robot_name_ = std::string(argv[1]);
    std::string node_name = robot_name_ + "_solar_recharge_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Subscriber subscriber = nh.subscribe("/" + robot_name_ + SYSTEM_MONITOR_TOPIC, 1000, updateRobotSolarOk);

    SolarRechargeServer solar_recharge_server(nh, SOLAR_RECHARGE_ACTIONLIB, boost::bind(&solarRecharge, _1, &solar_recharge_server), false);
    solar_recharge_server.start();

    ROS_INFO("Connecting to nav server...");

    navigation_client_ = new NavigationClient_(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
    navigation_client_->waitForServer();

    ROS_INFO("Connected. Waiting for a solar recharge request.");

    ros::spin();

    delete navigation_client_;

    return 0;
  }
}