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

  /**
   * @brief The sequence MUST not be changed:
   *        SEQUENCE: 0 = SCOUT
   *                  1 = EXCAVATOR
   *                  2 = HAULER
   * 
   */
  std::vector<std::string> robots_in_team_;

  SCHEDULER_STATES robot_state_ = SCHEDULER_STATES::INIT;
  
  bool start_scheduler_ = false;

  typedef actionlib::SimpleActionClient<state_machines::RobotStateMachineTaskAction> RobotClient;
  std::map<std::string, RobotClient*> map_of_clients_;

  void initTeam(const int team_number);

  void initClients();

  void schedulerLoop();

  void init();

  void startSearching();

  void initOdomAtHopper();

  void goToVolatile();

  void parkRobots();

  void excavateVolatile();

  void depositVolatile();

  void resetHaulerOdom();

  void reuniteTeam();

public:
  Scheduler(ros::NodeHandle nh, const int team_number = 1);

  ~Scheduler();

  void startScheduler();
  void stopScheduler(); // Do we need it though?
};