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
  ros::NodeHandle nh_;

  std::string robot_name_;

  ros::ServiceClient spiralClient_;
  ros::Subscriber volatile_sub_;

  bool near_volatile_ = false;
  bool new_message_received = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

  ScoutBaseState(ros::NodeHandle nh, const std::string &robot_name);

  /**
   * @brief Volatile sensor callback
   * 
   * @param msg 
   */
  void volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

public:
  /**
   * @brief Get the Base State object. Use this function to access a pointer to the object of
   * ScoutBaseState. Only one object of this class is created and it is shared with all the classes that use
   * ScoutBaseState.
   *
   * @param nh ros Nodehandle
   * @return ScoutBaseState*
   */
  static ScoutBaseState* getScoutBaseState(ros::NodeHandle nh, const std::string &robot_name);
  ~ScoutBaseState();

  /**
   * @brief disable copy constructor. This is required for singleton pattern
   *
   */
  ScoutBaseState(ScoutBaseState const&) = delete;

  /**
   * @brief disable assignment operator. This is required for singleton pattern
   *
   */
  void operator=(ScoutBaseState const&) = delete;
};
