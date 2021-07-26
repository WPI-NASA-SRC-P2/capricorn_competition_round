#include "operations/solar_charging_server.h"

// SolarModeAction::SolarModeAction(ros::NodeHandle& nh, std::string robot_name)
// {
// 	robot_name_ = robot_name;
// 	std::string node_name = robot_name + "_solar_mode_action_server";

// 	// Initialise the publishers for steering and wheel velocites
// 	initPowerSaverPublisher(nh, robot_name);
// 	initSystsemMonitorSubscriber(nh, robot_name);

// 	ros::Duration(1).sleep();

// 	//nh.param("crab_drive", CRAB_DRIVE_, false);

// 	ROS_INFO("[operations | solar_mode_server | %s]: Starting solar mode server...\n", robot_name_.c_str());

// 	// Action server
// 	solarServer_ = new Server(nh, action_name_, boost::bind(&SolarModeAction::executeCB, this, _1), false);
// 	solarServer_->registerPreemptCallback(boost::bind(&SolarModeAction::cancelGoal, this));
// 	solarServer_->start();

// 	ROS_INFO("[operations | solar_mode_server | %s]: Navigation solar mode server started.\n", robot_name_.c_str());
// }

void SolarModeAction::initPowerSaverPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  std::string power_saver_service_ =  "/" + robot_name + "/system_monitor/power_saver";
  powerMode_client = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>(power_saver_service_);
}

void SolarModeAction::initSystsemMonitorSubscriber(ros::NodeHandle &nh, const std::string &robot_name)
{
  std::string system_monitor_topic_ = "/" + robot_name + "/system_monitor";
  systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, &SolarModeAction::systemMonitorCB, solarServer_);
}


void SolarModeAction::rotateRobot(void)
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0.6; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarModeAction::stopRobot(void)
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarModeAction::setPowerSaveMode(bool state)
{

  srcp2_msgs::SystemPowerSaveSrv srv;
  srv.request.power_save = state;
  powerMode_client.call(srv);
}

void SolarModeAction::startSolarMode()
{
  ROS_INFO("Starting Solar Charge Mode");
  setPowerSaveMode(false);
  rotateRobot();
  should_turn = true;
}

void SolarModeAction::stopSolarMode()
{
  ROS_INFO("Stopping Solar Charge Mode");
  setPowerSaveMode(false);
  stopRobot();
  should_turn = false;
}

void SolarModeAction::initMode(bool initModeGoal)
{
  
  ROS_INFO("entered solarChargeInit");

  success = true;
 
  if(initModeGoal){
    startSolarMode();
  }
  else{
    stopSolarMode();
  }
}

void SolarModeAction::systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg)
{
  //ROS_INFO("in the system monitor callback - 1");
  
  solar_ok = msg.solar_ok;
  power_rate = msg.power_rate;
  power_saver = msg.power_saver;   //i assume to check of we are in or out of power save mode

}


void SolarModeAction::executeCB(const operations::SolarModeGoalConstPtr &goal)
{
  ros::Rate r(1);

  initMode(goal.solar_charge_status);

  while(goal.solar_charge_status == true && !solar_ok){
    
    //in case there is no solar charging, change nothing
    //Maybe TODO:: if turing takes too long, set success = false

    feedback_.solar_ok_ = solar_ok;
    solarServer.publishFeedback(feedback_);

    if(solar_ok && should_turn)
    {
    stopRobot();
    setPowerSaveMode(true);
    ROS_INFO("solar okay seen");
    should_turn = false;
    }
    
  
    r.sleep();

    //could put feedback if it ends up being useful
  }

  if(success){

    result_.solar_ok_ = solar_ok;
    result_.power_rate_= power_rate;
    result_.power_saver_ = power_saver;

    solarServer_.setSucceeded(result_);

  }

}

SolarModeAction::cancelGoal(){
  stopSolarMode();
}



