#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <future>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <srcp2_msgs/SystemMonitorMsg.h>
#include <srcp2_msgs/SystemPowerSaveSrv.h>
#include <utils/common_names.h>

#include "operations/SolarCharge.h"
#include "operations/solar_charging_server.h"

// typedef for the Action Server and Client
//typedef actionlib::SimpleActionServer<operations::SolarRechargeAction> SolarRechargeServer;
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

double volatile_distance_;
std::string robot_name_;

/**
 * @brief enum for rotation direction
 *        When clockwise, send direction as it is
          When Anticlockwise, flip the direction with -1
 * 
 */
// enum DrivingDirection
// {
//   POSITIVE = 1,
//   NEGATIVE = -1
// };

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
void SolarChargingServer::rotateRobot(const DrivingDirection rotate_direction, const float rotational_velocity_multiplier)
{
  operations::NavigationGoal goal;

 // Manual driving  
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = rotate_direction * ROTATION_VELOCITY * rotational_velocity_multiplier;

  navigation_client_->sendGoal(goal);
  ROS_INFO("Send Nav Goal to Turn");
  ros::Duration(0.1).sleep();
}

/**
 * @brief Stop the robot
 * 
 *
 */
void SolarChargingServer::stopRobot(void)
{
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = 0;

  navigation_client_->sendGoal(goal);
  ROS_INFO("Send Nav Goal to Stop");
  ros::Duration(0.5).sleep();
}

// #TODO
void SolarChargingServer::setPowerSaveMode(bool state){
  //call the server "small_scout_1/system_monitor/power_saver"
  //set the request.power_save
    // true: Power Save will be active
    // false:: power Save will be deactivated

  srcp2_msgs::SystemPowerSaveSrv srv;

  srv.request.power_save = state;

  powerMode_client.call(srv);
}


/**
 * @brief based on the request, either Acitivates Solar_Charge_Mode, or diactives it
 * 
 * @param request solarChargeRequest 
 * @param reponse solarChargeResponse
 */
bool SolarChargingServer::solarChargeInitiate(operations::SolarChargeRequest &request, operations::SolarChargeResponse &response)
{
  should_turn = request.solar_charging_status;
  if(should_turn)
  {
    if(!is_turning) {
      std::async(std::launch::async, turnRobot());
      is_turning = true;
    }
  }
  else
  {
    is_turning = false;
    stopRobot();
    setPowerSaveMode(false);
    ROS_INFO("Solar_Charging_Mode: OFF: Ending");
    response.result = "Solar_Charging_Mode: OFF: Ending";
  }


  

  ROS_INFO("Starting transitiong to solar recharge");

  res.result.data = "The Robot is in Solar Charging Mode";
  ROS_INFO("Now Solar Recharging");
}

bool SolarChargingServer::turnRobot() {
  ROS_INFO("Solar_Charging_Mode: ON: Starting");
  DrivingDirection driving_direction = POSITIVE;
  rotateRobot(driving_direction, 1.0);
  while(!solar_ok && ros::ok() && should_turn) // while there is no solar keep rotating 
  {
    ROS_INFO("Solar_Charging_Mode: ON: Rotate Into Position");
  }
  stopRobot();
  setPowerSaveMode(true);
  ROS_INFO("Solar_Charging_Mode: ON: Charging");
  //response.result = "Solar_Charging_Mode: ON: Power_rate:" + power_rate.c_str();
}


void SolarChargeingServer::systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg){
  solar_ok = msg.solar_ok;
  power_rate = msg.power_rate;
  power_saver = msg.power_saver;   //i assume to check of we are in or out of power save mode
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

  SolarChargingServer server;

  //create a nodehandle
  ros::NodeHandle nh;

  server.systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, &SolarChargingServer::SystemMonitorCB, &server);

  //srv result
  debug_solar_charging_mode = nh.advertise<std_msgs::String>("/galaga/debug_solar_charging_status", 1000);
  
  //client for power saving mode
  server.powerMode_client = nh.serviceClient<srcp2_msgs::SystemPowerSave>("power_mode");

  // operations::SolarChargeRequest req = true;
  // operations::SolarChargeResponse res;
  // client.call(req, res);


  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("solar_charger", &SolarChargingServer::solarChargeInitiate, &server);

  ros::spin();

  return 0;
}