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

  // name of robots depeding on team number, eg. if team_number = 2, SCOUT = small_scout_2
  std::string SCOUT, EXCAVATOR, HAULER;

  SCHEDULER_STATES robot_state_ = SCHEDULER_STATES::INIT;

  bool start_scheduler_ = false;

  typedef actionlib::SimpleActionClient<state_machines::RobotStateMachineTaskAction> RobotClient;
  RobotClient *scout_client_;
  RobotClient *excavator_client_;
  RobotClient *hauler_client_;

  // variables to hold current task given to each robot in the team
  state_machines::RobotStateMachineTaskGoal scout_goal_;
  state_machines::RobotStateMachineTaskGoal excavator_goal_;
  state_machines::RobotStateMachineTaskGoal hauler_goal_;

  // variables to keep track of each robot's pose in the team
  geometry_msgs::PoseStamped scout_pose_;
  geometry_msgs::PoseStamped excavator_pose_;
  geometry_msgs::PoseStamped hauler_pose_;

  // robot odom subscribers
  ros::Subscriber scout_odom_sub_;
  ros::Subscriber excavator_odom_sub_;
  ros::Subscriber hauler_odom_sub_;

  std::mutex scout_pose_mutex;
  std::mutex excavator_pose_mutex;
  std::mutex hauler_pose_mutex;

  // variables to hold the desired new tasks to be given to each robot
  STATE_MACHINE_TASK scout_desired_task;
  STATE_MACHINE_TASK excavator_desired_task;
  STATE_MACHINE_TASK hauler_desired_task;

  bool hauler_got_stuff_ = false;

  bool scout_task_completed_ = false, excavator_task_completed_ = false, hauler_task_completed_ = false;

  /**
   * @brief This function intializes subscriber for scout, excavator and hauler's clients
   * and respective robot state machine's client
   * All three robots will be corresponding to the team number
   * 
   * @param team_number 
   */
  void initTeam(const int team_number);

  /**
   * @brief Initializes all three robots's state machine's client which will be used to 
   * give goals to respective robots
   * 
   */
  void initClients();

  /**
   * @brief This is the main loop for scheduler, which calls respective function for update robot status,
   * gives robot's respective goal if they are not sent previously
   * 
   */
  void schedulerLoop();

  /**
   * @brief Wait for every robot state machine client to connect to server, before starting the scheduler and
   * intialize current robot tasks to a default value = -1, which is not a task of any robot in state machine, 
   * just to keep things from breaking at start
   * 
   */
  void init();

  /**
   * @brief checks if the task given to all robots is completed,
   * bug in here, it does not consider if the task was successful or failure
   * 
   */
  void updateRobotStatus();

  /**
   * @brief send goal to scout if a new goal is available
   * 
   */
  void updateScout();

  /**
   * @brief send goal to excavator if a new goal is available
   * 
   */
  void updateExcavator();

  /**
   * @brief send goal to hauler if a new goal is available
   * 
   */
  void updateHauler();

  /**
   * @brief at the start of simulation, orders scout to start searching
   * 
   */
  void startScout();

  /**
   * @brief at the start of simulation, orders excavator to take arm to default location
   * 
   */
  void startExcavator();

  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task);

  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task, const geometry_msgs::PoseStamped &goal_loc);

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