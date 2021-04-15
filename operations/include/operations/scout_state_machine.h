#ifndef SCOUT_STATE_MACHINE_H
#define SCOUT_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <operations/ResourceLocaliserAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <utils/common_names.h>
#include <mutex>

using namespace COMMON_NAMES;

enum LOCATOR_STATES
{
  INIT = 0,   // At the start of each session or after recharging, 
              // we would like to be at a certain location. Hence this state to get there
  SEARCH ,    // SEARCH ALGORITHM
  LOCATE,     // Rotate and drive until distance minimises
  FOUND,      // Stop at the location for leading Excavator to the location
  MOVE_OUT,    // Make way for Excavator
  RECHARGE,    // Find and go to Recharge Station
  WAIT_FOR_STATE_UPDATE   // Do nothing
};


class ScoutStateMachine
{

private:
  ros::NodeHandle nh_;

  LOCATOR_STATES robot_state_ = LOCATOR_STATES::INIT;
  std::string robot_name_;
  bool state_machine_continue_ = true;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
  NavigationClient_* navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_* resource_localiser_client_;
  operations::ResourceLocaliserGoal resource_localiser_goal_;

  // Listens to the volatile sensor
  ros::Subscriber volatile_sub_;

  // Publishes the exact location of a localised volatile
  ros::Publisher  volatile_pub_;

  // Listens for excavators indicating they are ready to pick up a volatile.
  ros::Subscriber excavator_ready_sub_;

  // Listens for robot odometry. Used to publish volatile location in FOUND
  ros::Subscriber robot_odom_sub_;

  void processVolatileMessage(const srcp2_msgs::VolSensorMsg::ConstPtr& vol_msg);
  void processOdomMessage(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void processExcavatorMessage(const std_msgs::Empty::ConstPtr& excavator_msg);

  // Whether or not we are on the first iteration of a state.
  // Reset to true on state transition, set to false after first iteration.
  bool first_iter_ = true;

  // Whether a volatile is currently detected. Reset to false at the beginning of SEARCH
  bool vol_detected_ = false;

  // Whether an excavator has arrived and is ready to pick up a volatile. Reset to false at the beginning of FOUND
  bool excavator_ready_ = false;

  // Most recent robot pose from odometry
  geometry_msgs::PoseStamped robot_pose_;

  // Mutex-protected way to get robot_pose_
  geometry_msgs::PoseStamped getRobotPose();

  // Mutexex to protect flags set in subscribers and read in the SM
  std::mutex vol_flag_mutex_;
  std::mutex robot_pose_mutex_;
  std::mutex excavator_ready_mutex_;

public:
  ScoutStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ScoutStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // SCOUT_STATE_MACHINE_H