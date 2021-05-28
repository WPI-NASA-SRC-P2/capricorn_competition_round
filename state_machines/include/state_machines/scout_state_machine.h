#pragma once

#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <operations/NavigationVisionAction.h>
#include <operations/Spiral.h>
#include <operations/ResourceLocaliserAction.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <operations/obstacle_avoidance.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> SCOUT_TASKS = {
    STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_STOP_SEARCH,
    STATE_MACHINE_TASK::SCOUT_LOCATE_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_UNDOCK,
};

class ScoutBaseState
{
private:
  ros::Subscriber volatile_sub_;
  ros::Subscriber objects_sub_;

protected:
  ros::NodeHandle nh_;

  std::string robot_name_;

  ros::ServiceClient spiralClient_;

  bool near_volatile_ = false;

  std::mutex objects_mutex_;
  perception::ObjectArray::ConstPtr vision_objects_;
  bool objects_msg_received_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

  /**
   * @brief Volatile sensor callback
   * 
   * @param msg 
   */
  void volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

public:
  ScoutBaseState(ros::NodeHandle nh, const std::string &robot_name);
  ~ScoutBaseState();

  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class Undock: public ScoutBaseState
{
public:
  Undock(ros::NodeHandle nh, const std::string &robot_name);
  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class Search: public ScoutBaseState
{
public:
  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class Locate: public ScoutBaseState
{
public:
  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class SolarCharge: public ScoutBaseState
{
public:
  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class RepairRobot: public ScoutBaseState
{
public:
  bool entryPoint();
  bool exec();
  bool exitPoint();
};

class ResetOdom: public ScoutBaseState
{
public:
  bool entryPoint();
  bool exec();
  bool exitPoint();
};
