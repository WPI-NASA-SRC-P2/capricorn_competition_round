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

//double ROTATION_VELOCITY = 0.2;
//double DRIVING_VELOCITY = 0.2;
//double MAX_DETECT_DIST = 2.0;
//double VOLATILE_DISTANCE_THRESHOLD = 0.005;
//int FLIP_ROTATION_COUNT_MAX = 2;
//int REPEAT_COUNT_MAX = 2;
//bool near_volatile_ = false;
//bool new_message_received = false;


std::string robot_name_;


enum DrivingMode
{
  ROTATE_ROBOT = 0,
  DRIVE_ROBOT_STRAIGHT
};

void SolarChargingServer::rotateRobot()
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0.6; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarChargingServer::stopRobot(void)
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarChargingServer::setPowerSaveMode(bool state){

  srcp2_msgs::SystemPowerSaveSrv srv;
  srv.request.power_save = state;
  powerMode_client.call(srv);
}

bool SolarChargingServer::solarChargeInitiate(operations::SolarCharge::Request& request, operations::SolarCharge::Response& response)
{
  //response.result.data = "Solar_Charging_Mode: ON: Power_rate:" + power_rate.c_str(); << Maybe we will use this at some point
  // 
  ROS_INFO("entered solarChargeInit");
  
  if(request.solar_charge_status){
    ROS_INFO("entered solarChargeInit request true");
    setPowerSaveMode(false);
    rotateRobot();
    response.result.data = "Started Solar Recharge Mode";
    should_turn = true;
    return true;
  }
  if(!request.solar_charge_status){
    ROS_INFO("entered solarChargeInit request false");
    setPowerSaveMode(false);
    stopRobot();
    response.result.data = "Ended/Killed Solar Recharge Mode";
    return true;
  }
  return false;
}


void SolarChargingServer::systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg){
  //ROS_INFO("in the system monitor callback - 1");
  solar_ok = msg.solar_ok;
  power_rate = msg.power_rate;
  power_saver = msg.power_saver;   //i assume to check of we are in or out of power save mode
  
  if(solar_ok && should_turn){
    stopRobot();
    setPowerSaveMode(true);
    ROS_INFO("solar okay seen");
    should_turn = false;
  }

}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "solar_charging_server");

  std::string robot_name(argv[1]);   
  //std::string robot_name = "small_scout_1";

  //ROS Topic names
  //std::string system_monitor_topic_ = "/capricorn/" + robot_name + "/system_monitor";
  std::string system_monitor_topic_ = robot_name + "/system_monitor";

  //ROS ServerService
  std::string power_saver_service_ =  robot_name + "/system_monitor/power_saver";
 
  SolarChargingServer server;

  //create a nodehandle
  ros::NodeHandle nh;
  
  ros::ServiceServer serviceD = nh.advertiseService("solar_charger", &SolarChargingServer::solarChargeInitiate, &server);

  // initialize client
  navigation_client_= new NavigationClient_(CAPRICORN_TOPIC + robot_name + "/" + NAVIGATION_ACTIONLIB, true);
  ROS_INFO_STREAM("[ operations | solar_recharge | "<<robot_name<<" ] "<< "Waiting for navigation server...");
  bool serverExists = navigation_client_->waitForServer();
  ROS_INFO_STREAM("[ operations | solar_recharge | "<<robot_name<<" ] "<< "Server Connected");

  server.systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, &SolarChargingServer::systemMonitorCB, &server);

  //debugging purposes
  //debug_solar_charging_mode = nh.advertise<std_msgs::String>("/galaga/debug_solar_charging_status", 1000);
  
  //client for power saving mode
  server.powerMode_client = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>(power_saver_service_);

  ros::spin();

  return 0;
}