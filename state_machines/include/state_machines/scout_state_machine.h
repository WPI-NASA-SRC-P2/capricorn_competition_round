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

class ScoutStateMachine
{
private:
  ros::NodeHandle nh_;

  std::string robot_name_;

  ros::ServiceClient spiralClient_;
  ros::Subscriber volatile_sub_;
  bool near_volatile_ = false;
  bool new_message_received = false;
  bool continue_spiral_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

  /**
   * @brief Start the searching algorithm
   * 
   */
  bool startSearchingVolatile();

  /**
   * @brief Stop the searching algorithm
   * 
   */
  bool stopSearchingVolatile();

  /**
   * @brief Sets the status of searching algorithm according to argument resume
   * 
   */
  bool resumeSearchingVolatile(const bool resume);

  /**
   * @brief Pinpoint the location of the volatile, and stand on it
   * 
   */
  bool locateVolatile();

  /**
   * @brief Move away from the excavator to continue on the trajectory
   *        CURRENTLY NOT IMPLEMENTED
   * 
   */
  bool undockRobot();

  /**
   * @brief Volatile sensor callback
   * 
   * @param msg 
   */
  void volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

public:
  ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name);

  ~ScoutStateMachine();

  friend void cancelGoal(ScoutStateMachine *sm);
  friend void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ScoutStateMachine *sm);
};

// #endif // SCOUT_STATE_MACHINE_H