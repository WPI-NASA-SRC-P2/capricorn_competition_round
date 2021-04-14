#ifndef HAULER_STATE_MACHINE_H
#define HAULER_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <std_msgs/Empty.h>
#include <operations/HaulerAction.h>
#include <operations/NavigationVisionAction.h>
#include <operations/ParkRobotAction.h>
#include <srcp2_msgs/ScoreMsg.h>

using namespace COMMON_NAMES;

enum HAULER_STATES
{
  INIT = 0,         // Wait for Instructions
                        // or it may get close to scout
  GO_TO_DIG_SITE,   // Get close to the volatile when it is detected
  FOLLOW_EXCAVATOR, // Navigation method to 'follow' excavator
  PARK_AT_EXCAVATOR,     // Park the hauler when it is good to do so
                        // Needs a 'park' publisher by excavator
  ACCEPT_VOLATILE,    // Dig and see if any volatile is detected. 
                        // If no volatile found, change the orientation slightly
                        // Else change state
  GO_TO_PROC_PLANT,     // Go to processing plant (visual navigation)
  PARK_AT_HOPPER,       // Park hopper to dump the volatiles
  DUMP_VOLATILE       // Dump volatile into the hopper
};


class HaulerStateMachine
{

private:
  ros::NodeHandle nh_;

  ros::Subscriber dig_site_location_;   // Redundant name? site and location?
  ros::Subscriber src_score_sub_;
  ros::Publisher hauler_parked_pub_;

  HAULER_STATES robot_state_ = HAULER_STATES::INIT;
  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  
  bool state_machine_continue_ = true;
  bool volatile_found_ = false;
  bool nav_server_idle_ = true;
  bool nav_vis_server_idle_ = true;
  // bool excavator_server_idle_ = true;
  
  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient* navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::HaulerAction> HaulerClient;
  HaulerClient* hauler_client_;
  operations::HaulerGoal hauler_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient* navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::ParkRobotAction> ParkRobotClient;
  ParkRobotClient* park_robot_client_;
  operations::ParkRobotGoal park_robot_goal_;

  geometry_msgs::PoseStamped dig_site_pose_;


  /**
   * @brief Callback for the location of found digging site
   * 
   * @param msg 
   */
  void digSiteLocCB(const geometry_msgs::PoseStamped &msg);

  /**
   * @brief Waits for the scout to find the volatile
   *        Basically does nothing
   *        Ideally, should be used to stay close to Excavator
   *          for minimising the time when the volatile is found
   * 
   */
  void initState();

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  void goToDigSite();

  /**
   * @brief Goes to the actual location where the volatile was predicted. 
   * 
   */
  void parkHauler();

  /**
   * @brief This is required to make sure the hauler is close enough to the excavator
   * 
   */
  void followExcavator();

  // /**
  //  * @brief Dig the volatile location
  //  * 
  //  */
  // void digVolatile();

  // /**
  //  * @brief Dump the volatile at Hauler Location
  //  * 
  //  */
  // void dumpVolatile();

public:
  HaulerStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~HaulerStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // HAULER_STATE_MACHINE_H