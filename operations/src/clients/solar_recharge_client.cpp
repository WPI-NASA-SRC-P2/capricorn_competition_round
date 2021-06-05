#include <operations/NavigationAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

// lets just consider the idea of "do we need a serice"
// the only thing we are checking is when to turn or not,
// we don't really have a purpose for the message


typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
NavigationClient_ *navigation_client_;

using namespace COMMON_NAMES;

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
  //actionlib goal
  operations::NavigationGoal goal;

    double ROTATION_VELOCITY = 0.2;
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
void stopRobot(void)
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
void solarChargeInitiate(void)
{
  DrivingDirection driving_direction = POSITIVE;

  ROS_INFO("Starting transitiong to solar recharge");

  rotateRobot(driving_direction, 1.0);

  while(!solar_ok && ros::ok()) // while there is no solar keep rotating 
  {
    ROS_INFO("Rotating the robot");
  } 

  stopRobot();
  ROS_INFO("Now Solar Recharging");
  
}

void solarChargeInitiate_2(const srcp2_msgs::SystemMonitorMsg &msg)
{
    bool solar_ok = msg.solar_ok;

    DrivingDirection driving_direction = POSITIVE;

    ROS_INFO("Starting transitiong to solar recharge");

    if(!solar_ok)
    {
        ROS_INFO("Rotating the robot");
        rotateRobot(driving_direction, 1.0);
    }
    else
    {
        stopRobot();
    }

    ROS_INFO("Now Solar Recharging");
}

void systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg){
  solar_ok = msg.solar_ok;
}

// int main(int argc, char **argv)
// {
//   // Ensure the robot name is passed in
//   if (argc != 2 && argc != 4)
//   {
//     // Displaying an error message for correct usage of the script, and returning error.
//     ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
//     return -1;
//   }
//   else
//   {
//     // Robot Name from argument
//     robot_name_ = std::string(argv[1]);
//     std::string node_name = robot_name_ + "_solar_recharge_action_server";
//     ros::init(argc, argv, node_name);
//     ros::NodeHandle nh;

//     ros::Subscriber subscriber = nh.subscribe("/" + robot_name_ + SYSTEM_MONITOR_TOPIC, 1000, updateRobotSolarOk);

//     SolarRechargeServer solar_recharge_server(nh, SOLAR_RECHARGE_ACTIONLIB, boost::bind(&solarRecharge, _1, &solar_recharge_server), false);
//     solar_recharge_server.start();

//     ROS_INFO("Connecting to nav server...");

//     navigation_client_ = new NavigationClient_(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
//     navigation_client_->waitForServer();

//     ROS_INFO("Connected. Waiting for a solar recharge request.");

//     ros::spin();

//     delete navigation_client_;

//     return 0;
//   }
// }

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "solar_charging_server");

  std::string robot_name(argv[1]);   

  //ROS Topic names
  std::string system_monitor_topic_ = "/capricorn/" + robot_name + "/system_monitor";

  //create a nodehandle
  ros::NodeHandle nh;

  // initializing the client
  NavigationClient_ navigation_client_(robot_name + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    // wait for the server to run
    navigation_client_.waitForServer();

  ros::Subscriber systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, solarChargeInitiate_2);
  
  ros::spin();

  return 0;
}
