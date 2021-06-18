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
void SolarChargingServer::rotateRobot()
{
//   ROS_INFO("Setup Nav Goal to Turn");
//   operations::NavigationGoal goal;
//   typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
//   Client* client;
//  // Manual driving  
//   goal.drive_mode = NAV_TYPE::MANUAL;

//   goal.forward_velocity = 0;
//   goal.angular_velocity = rotate_direction * ROTATION_VELOCITY * rotational_velocity_multiplier;

//   navigation_client_->sendGoal(goal);
//   ROS_INFO("Send Nav Goal to Turn");
//   ros::Duration(0.1).sleep();operations::NavigationGoal goal;
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0.6; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}

/**
 * @brief Stop the robot
 * 
 *
 */
void SolarChargingServer::stopRobot(void)
{
  // operations::NavigationGoal goal;

  // // Manual driving
  // goal.drive_mode = NAV_TYPE::MANUAL;

  // goal.forward_velocity = 0;
  // goal.angular_velocity = 0;

  // navigation_client_->sendGoal(goal);
  // ROS_INFO("Send Nav Goal to Stop");
  // ros::Duration(0.5).sleep();
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
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
bool SolarChargingServer::solarChargeInitiate(operations::SolarCharge::Request& request, operations::SolarCharge::Response& response)
{
  rotateRobot();
  // operations::NavigationGoal goal;
  // goal.drive_mode = NAV_TYPE::MANUAL;

  //   // Diverting values from twist to navigation
  //   goal.forward_velocity = 0;
  //   goal.angular_velocity = 0.6; //units radian per sec ???

  // navigation_client_->sendGoal(goal);
  

  // ROS_INFO("in the callback - 1");

  // should_turn = request.solar_charge_status;
  // ROS_INFO("in the callback - 2");
  // if(should_turn)
  // {
  // ROS_INFO("in the callback - 3");
  //   if(!is_turning) {
  //     // std::async(std::launch::async, turnRobot());
  //     turnRobot();
  //     ROS_INFO("in the callback - 4");
  //     is_turning = true;
  //   }
  //   ROS_INFO("Solar_Charging_Mode: ON: STARTING");
  //   response.result.data = "Solar_Charging_Mode: ON: STARTING";
  //   return true;
  // }
  // else
  // {
  //   is_turning = false;
  //   stopRobot();
  //   setPowerSaveMode(false);
  //   ROS_INFO("Solar_Charging_Mode: OFF: Ending");
  //   response.result.data = "Solar_Charging_Mode: OFF: Ending";
  //   return true;
  // }

  //ROS_INFO("Starting transitiong to solar recharge");

  
  //ROS_INFO("Now Solar Recharging");
  //response.result.data = "Solar_Charging_Mode: ON: Power_rate:" + power_rate.c_str();
}

bool SolarChargingServer::turnRobot() {
  ROS_INFO("Solar_Charging_Mode: ON: Turning");
  DrivingDirection driving_direction = POSITIVE;
  rotateRobot();
  while(!solar_ok && ros::ok() && should_turn) // while there is no solar keep rotating 
  {
    ROS_INFO("Solar_Charging_Mode: ON: Rotate Into Position");
  }
  stopRobot();
  setPowerSaveMode(true);
  ROS_INFO("Solar_Charging_Mode: ON: Charging");
  return true;
}


void SolarChargingServer::systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg){
  ROS_INFO("in the callback - 1");
  solar_ok = msg.solar_ok;
  power_rate = msg.power_rate;
  power_saver = msg.power_saver;   //i assume to check of we are in or out of power save mode
  ROS_INFO("in the callback - 2");
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
  //std::string robot_name = "small_scout_1";

  //ROS Topic names
  std::string system_monitor_topic_ = "/capricorn/" + robot_name + "/system_monitor";

  //ROS ServerService
  std::string power_saver_service_ =  "/" + robot_name + "/system_monitor/power_saver";
 
  SolarChargingServer server;

  //create a nodehandle
  ros::NodeHandle nh;

  //Instantiating ROS server for solar charge mode
  //server.solarChargerService = nh.advertiseService("solar_charger", &SolarChargingServer::solarChargeInitiate, &server);
  ros::ServiceServer serviceD = nh.advertiseService("solar_charger", &SolarChargingServer::solarChargeInitiate, &server);

  // initialize client
  navigation_client_= new NavigationClient_(CAPRICORN_TOPIC + robot_name + "/" + NAVIGATION_ACTIONLIB, true);
  //printf("Waiting for server...\n");
  ROS_INFO_STREAM("[ operations | solar_recharge | "<<robot_name<<" ] "<< "Waiting for navigation server...");
  bool serverExists = navigation_client_->waitForServer();
  ROS_INFO_STREAM("[ operations | solar_recharge | "<<robot_name<<" ] "<< "Server Connected");
  //server.systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, &SolarChargingServer::systemMonitorCB, &server);

  //srv result
  //debug_solar_charging_mode = nh.advertise<std_msgs::String>("/galaga/debug_solar_charging_status", 1000);
  
  //client for power saving mode
  //server.powerMode_client = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>(power_saver_service_);

  // operations::SolarChargeRequest req = true;
  // operations::SolarChargeResponse res;
  // client.call(req, res);

  ros::spin();

  return 0;
}