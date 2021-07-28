#include "operations/solar_charging_server.h"

SolarModeServer::SolarModeServer(ros::NodeHandle nh, std::string robot_name):nh_(nh), robot_name_(robot_name)
{
	std::string node_name = robot_name + "_solar_mode_action_server";

	// Initialise the publishers for steering and wheel velocites
	initPowerSaverPublisher(nh_, robot_name);
	initSystsemMonitorSubscriber(nh_, robot_name);

	ros::Duration(1).sleep();

	//nh_.param("crab_drive", CRAB_DRIVE_, false);

	ROS_INFO("[operations | solar_mode_server | %s]: Starting solar mode server...\n", robot_name_.c_str());

  navigation_client_ = new NavigationClient_(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
  navigation_client_->waitForServer();

	// Action server
	solarServer_ = new SolarServer_(nh_, COMMON_NAMES::SOLAR_RECHARGE_ACTIONLIB, boost::bind(&SolarModeServer::executeCB, this, _1), false);
	solarServer_->registerPreemptCallback(boost::bind(&SolarModeServer::cancelGoal, this));
	solarServer_->start();

  ros::Duration(1).sleep();

	ROS_INFO("[operations | solar_mode_server | %s]: Navigation solar mode server started.\n", robot_name_.c_str());
}

void SolarModeServer::initPowerSaverPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  std::string power_saver_service_ =  "/" + robot_name + "/system_monitor/power_saver";
  powerMode_client = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>(power_saver_service_);
}

void SolarModeServer::initSystsemMonitorSubscriber(ros::NodeHandle &nh, const std::string &robot_name)
{
  std::string system_monitor_topic_ = "/" + robot_name + "/system_monitor";
  systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, &SolarModeServer::systemMonitorCB, this);
}


void SolarModeServer::rotateRobot(void)
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0.25; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarModeServer::stopRobot(void)
{
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

    // Diverting values from twist to navigation
    goal.forward_velocity = 0;
    goal.angular_velocity = 0; //units radian per sec ???

  navigation_client_->sendGoal(goal);
  ros::Duration(0.05).sleep();
}


void SolarModeServer::setPowerSaveMode(bool state)
{

  srcp2_msgs::SystemPowerSaveSrv srv;
  srv.request.power_save = state;
  powerMode_client.call(srv);
}

void SolarModeServer::startSolarMode()
{
  ROS_INFO("[operations | solar_mode_server | %s]: Starting Solar Charge Mode", robot_name.c_str());
  setPowerSaveMode(false);
  rotateRobot();
  should_turn = true;
}

void SolarModeServer::stopSolarMode()
{
  ROS_INFO("[operations | solar_mode_server | %s]: Stopping Solar Charge Mode", robot_name.c_str());
  setPowerSaveMode(false);
  if(robot_moving)
    stopRobot();
  should_turn = false;
}

void SolarModeServer::initMode(bool initModeGoal)
{
  ROS_INFO("[operations | solar_mode_server | %s]: entered solarChargeInit", robot_name.c_str());

  success = true;
 
  if(initModeGoal){
    startSolarMode();
  }
  else{
    stopSolarMode();
  }
}

void SolarModeServer::systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg)
{
  //ROS_INFO("in the system monitor callback - 1");
  
  solar_ok = msg.solar_ok;
  power_rate = msg.power_rate;
  power_saver = msg.power_saver;   //i assume to check of we are in or out of power save mode

}


void SolarModeServer::executeCB(const operations::SolarModeGoalConstPtr &goal)
{
  ros::Rate r(10);

  

  initMode(goal->solar_charge_status);
  while(goal->solar_charge_status == true){
    robot_moving = true;
    ros::spinOnce();
    ROS_INFO("[operations | solar_mode_server | %s]: Rotating robot for solar charging", robot_name.c_str());
    //in case there is no solar charging, change nothing
    //Maybe TODO:: if turing takes too long, set success = false

    feedback_.solar_ok_ = solar_ok;
    solarServer_->publishFeedback(feedback_);    
  
    if(solar_ok)
    {
      stopRobot();
      setPowerSaveMode(true);
      should_turn = false;
      break;
    }

    r.sleep();

    //could put feedback if it ends up being useful
  }

    robot_moving = false;
    if(success){
    ROS_INFO("[operations | solar_mode_server | %s] : Done rotating robot", robot_name.c_str());

    result_.solar_ok_ = solar_ok;
    result_.power_rate_= power_rate;
    result_.power_saver_ = power_saver;
    result_.result = COMMON_RESULT::SUCCESS;

    solarServer_->setSucceeded(result_);

  }

}

void SolarModeServer::cancelGoal(){
    stopSolarMode();
}



