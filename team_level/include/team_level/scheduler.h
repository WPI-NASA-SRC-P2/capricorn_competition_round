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

  /**
   * @brief Common Function for sending the robot's desired goal to their respective actionlibs
   * 
   * @param robot_name      The robot for which task is given
   * @param robot_client    Pointer of client of the robot's actionlib server
   * @param robot_goal      Goal to be sent to the actionlib server
   * @param task            Robot's desired task to be set
   */
  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task);
  
  /**
   * @brief Common Function for sending the robot to the given location. 
   *        This function is specifically for GO_TO_LOC as that state also requires the location of the goal
   *        It will take the current and goal location into consideration, and calculate a point lying on a 
   *        line connecting the goals. That goal is sent to the server
   * 
   * @param robot_name      The robot for which task is given
   * @param robot_client    Pointer of client of the robot's actionlib server
   * @param robot_goal      Goal to be sent to the actionlib server
   * @param task            Robot's desired task to be set
   * @param goal_loc    
   */
  void sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task, const geometry_msgs::PoseStamped& goal_loc);


  /**
   * @brief Sends the scout_desired_goal to scout state machine actionlib
   * 
   * @param task    Desired task to be sent to the scout
   */
  void sendScoutGoal(const STATE_MACHINE_TASK task);


  /**
   * @brief Sends the excavator_desired_goal to excavator state machine actionlib
   * 
   * @param task    Desired task to be sent to the excavator
   */
  void sendExcavatorGoal(const STATE_MACHINE_TASK task);


  /**
   * @brief Sends the hauler_desired_goal to hauler state machine actionlib
   * 
   * @param task    Desired task to be sent to the hauler
   */
  void sendHaulerGoal(const STATE_MACHINE_TASK task);

  /**
 * @brief Callback to the scout pose topic
 * 
 * @param msg 
 */
  void updateScoutPose(const nav_msgs::Odometry::ConstPtr &msg);

  /**
 * @brief Callback to the Excavator pose topic
 * 
 * @param msg 
 */
  void updateExcavatorPose(const nav_msgs::Odometry::ConstPtr &msg);

  /**
 * @brief Callback to the Hauler pose topic
 * 
 * @param msg 
 */
  void updateHaulerPose(const nav_msgs::Odometry::ConstPtr &msg);

public:
  /**
   * @brief Construct a new Scheduler object  
   *        This scheduler only works for a team of scout-excav-hauler.
   *        For 6 robots, launch this script twice and give team_number arg 2
   * @param nh 
   * @param team_number 
   */
  Scheduler(ros::NodeHandle nh, const int team_number = 1);

  ~Scheduler();

  void startScheduler();
  void stopScheduler(); // Do we need it though?
};