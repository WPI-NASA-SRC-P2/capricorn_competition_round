#pragma once

#include <ros/ros.h>
#include <utils/common_names.h>
#include <mutex>
#include <vector>
#include <state_machines/RobotStateMachineTaskAction.h>  
#include <actionlib/client/simple_action_client.h>

using namespace COMMON_NAMES;

enum SCHEDULER_STATES
{
  INIT,
  SEARCHING,
  INIT_ODOM_AT_HOPPER,
  GO_TO_VOLATILE,
  PARK_ROBOTS,
  EXCAVATE_VOLATILE,
  DEPOSIT_VOLATILE,
  RESET_HAULER_ODOM,
  REUNITE_TEAM,
  // RESET_ROBOT_ODOM,
};

class Scheduler
{
private:
  ros::NodeHandle nh_;

  std::string SCOUT, EXCAVATOR, HAULER;

  SCHEDULER_STATES robot_state_ = SCHEDULER_STATES::INIT;
  
  bool start_scheduler_ = false;

  typedef actionlib::SimpleActionClient<state_machines::RobotStateMachineTaskAction> RobotClient;
  RobotClient* scout_client_;
  RobotClient* excavator_client_;
  RobotClient* hauler_client_;

  state_machines::RobotStateMachineTaskGoal scout_goal_;
  state_machines::RobotStateMachineTaskGoal excavator_goal_;
  state_machines::RobotStateMachineTaskGoal hauler_goal_;

  bool scout_done_ = false, excavator_done_ = false, hauler_done_ = false;

  void initTeam(const int team_number);

  void initClients();

  void schedulerLoop();

  void init();

  void startSearching();

  void updateRobotStatus();

  void updateScout();

  void updateExcavator();

  void updateHauler();

  void startScout();

  void sendRobotGoal(RobotClient* robot_client, state_machines::RobotStateMachineTaskGoal& robot_goal, const STATE_MACHINE_TASK task);
  
  void sendScoutGoal(const STATE_MACHINE_TASK task);
  
  void sendExcavatorGoal(const STATE_MACHINE_TASK task);
  
  void sendHaulerGoal(const STATE_MACHINE_TASK task);

public:
  Scheduler(ros::NodeHandle nh, const int team_number = 1);

  ~Scheduler();

  void startScheduler();
  void stopScheduler(); // Do we need it though?
};