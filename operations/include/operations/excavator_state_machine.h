#ifndef EXCAVATOR_STATE_MACHINE_H
#define EXCAVATOR_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <operations/ResourceLocaliserAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

enum EXCAVATOR_STATES
{
  INIT = 0,         // Wait for Instructions
                        // or it may get close to scout
  GO_TO_VOLATILE,   // Get close to the volatile when it is detected
  PARK_AND_PUB,     // Publish a message that excavator has reached, 
                        // And park where the scout was located. 
  FIND_VOLATILE,    // Dig and see if any volatile is detected. 
                        // If no volatile found, change the orientation slightly
                        // Else change state
  DIG_AND_DUMP,     // Start digging and dumping into the hauler
                        // This must check if hauler is close, else wait
  NEXT_QUE_TASK     // Inform the team level state machine that task completed, 
                        // Follow further instructions
};


class ExcavatorStateMachine
{

private:
  ros::NodeHandle nh_;

  EXCAVATOR_STATES robot_state_ = EXCAVATOR_STATES::INIT;
  std::string robot_name_;
  bool state_machine_continue_ = true;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
  NavigationClient_* navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient_;
  ExcavatorClient_* excavator_arm_client_;
  operations::ExcavatorGoal excavator_arm_goal_;

  


public:
  ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ExcavatorStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // EXCAVATOR_STATE_MACHINE_H