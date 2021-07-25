#pragma once

#include <ros/ros.h>
#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <operations/SolarModeAction.h>

#include <future>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <srcp2_msgs/SystemMonitorMsg.h>
#include <srcp2_msgs/SystemPowerSaveSrv.h>
#include <utils/common_names.h>

#include "operations/SolarCharge.h"


using namespace COMMON_NAMES;


class SolarModeAction
{
public:
  SolarModeAction(ros::NodeHandle& nh, std::string robot_name);
  ~SolarModeAction();

protected:
      ros::NodeHandle nh_;

      actionlib::SimpleActionServer<operations::SolarModeAction> SolarServer_;// NodeHandle instance must be created before this line. Otherwise strange error occurs.
      actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
      
      std::string action_name_;
      action_name_ = "SolarModeAction";
      // create messages that are used to published feedback/result
      operations::SolarModeFeedback feedback_;
      operations::SolarModeResult result_;


private:

  // setting up actionlib service and client
  SolarServer_ *solarServer_;
  NavigationClient_ *navigation_client_;

  // subscriber for system monitor
    ros::Subscriber systemMonitor_subscriber;

  // piblisher for powermode
    ros::ServiceClient powerMode_client; 


  std::string robot_name;
  bool solar_ok = false;  
  float_t power_rate; 
  bool power_saver = false;
  bool should_turn = false;
  bool success = false;

    //done
    void initPowerSaverPublisher(ros::NodeHandle &nh, const std::string &robot_name);

    //done 
    void initSystsemMonitorSubscriber(ros::NodeHandle &nh, const std::string &robot_name);

    //dont change
    void rotateRobot(void);

    //dont change
    void stopRobot(void);

    //dont change
    void setPowerSaveMode(bool state);

    void initMode(bool initModeGoal);

    //dont change
    void systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg);

    void executeCB(const operations::SolarModeGoalConstPtr &goal);

    void cancelGoal();
  
};