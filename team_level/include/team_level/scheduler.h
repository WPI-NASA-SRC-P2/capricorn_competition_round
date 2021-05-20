#pragma once

#include <ros/ros.h>
#include <utils/common_names.h>
#include <mutex>
#include <vector>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <operations/navigation_algorithm.h>

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
  RobotClient *scout_client_;
  RobotClient *excavator_client_;
  RobotClient *hauler_client_;

  state_machines::RobotStateMachineTaskGoal scout_goal_;
  state_machines::RobotStateMachineTaskGoal excavator_goal_;
  state_machines::RobotStateMachineTaskGoal hauler_goal_;

  geometry_msgs::PoseStamped scout_pose_;
  geometry_msgs::PoseStamped excavator_pose_;
  geometry_msgs::PoseStamped hauler_pose_;

  ros::Subscriber scout_odom_sub_;
  ros::Subscriber excavator_odom_sub_;
  ros::Subscriber hauler_odom_sub_;
  
  std::mutex scout_pose_mutex;
  std::mutex excavator_pose_mutex;
  std::mutex hauler_pose_mutex;

  STATE_MACHINE_TASK scout_desired_task;
  STATE_MACHINE_TASK excavator_desired_task;
  STATE_MACHINE_TASK hauler_desired_task;

  bool hauler_got_stuff_ = false;

  bool scout_task_completed_ = false, excavator_task_completed_ = false, hauler_task_completed_ = false;

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

  void startExcavator();

  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task);
  
  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task, const geometry_msgs::PoseStamped& goal_loc);

  void sendScoutGoal(const STATE_MACHINE_TASK task);

  void sendExcavatorGoal(const STATE_MACHINE_TASK task);

  void sendHaulerGoal(const STATE_MACHINE_TASK task);

  void setScoutGoal(const STATE_MACHINE_TASK task);

  void setExcavatorGoal(const STATE_MACHINE_TASK task);

  void setHaulerGoal(const STATE_MACHINE_TASK task);

  void updateScoutPose(const nav_msgs::Odometry::ConstPtr &msg);

  void updateExcavatorPose(const nav_msgs::Odometry::ConstPtr &msg);

  void updateHaulerPose(const nav_msgs::Odometry::ConstPtr &msg);

public:
  Scheduler(ros::NodeHandle nh, const int team_number = 1);

  ~Scheduler();

  void startScheduler();
  void stopScheduler(); // Do we need it though?
};