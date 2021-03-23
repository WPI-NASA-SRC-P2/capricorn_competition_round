#ifndef SCOUT_STATE_MACHINE_H
#define SCOUT_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

enum LOCATOR_STATES
{
  INIT = 0,   // At the start of each session or after recharging, 
              // we would like to be at a certain location. Hence this state to get there
  SEARCH ,    // SEARCH ALGORITHM
  LOCATE,     // Rotate and drive until distance minimises
  FOUND,      // Stop at the location for leading Excavator to the location
  MOVE_OUT,    // Make way for Excavator
  RECHARGE    // Find and go to Recharge Station
};


class ScoutStateMachine
{

private:
  ros::NodeHandle nh_;

  LOCATOR_STATES robot_state_ = LOCATOR_STATES::INIT;
  std::string robot_name_;
  bool state_machine_continue_ = true;

  operations::NavigationGoal navigation_action;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient* navigation_client;

  


public:
  ScoutStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ScoutStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // SCOUT_STATE_MACHINE_H