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
  INIT = 0,           // Wait for Instructions
  GO_TO_DIG_SITE,     // Get close to the digging site when it is detected
  FOLLOW_EXCAVATOR,   // Currently sets hauler up to park. 
                      // Is necessary for parking method)
  PARK_AT_EXCAVATOR,  // Park the hauler when it is good to do so
                      // TODO: Needs a 'park' publisher by excavator
  ACCEPT_VOLATILE,    // Just wait till volatile is completely dug
  GO_TO_PROC_PLANT,   // Go to processing plant (visual navigation)
  PARK_AT_HOPPER,     // Park hopper to dump the volatiles
  DUMP_VOLATILE       // Dump volatile into the hopper
};


class HaulerStateMachine
{

private:
  ros::NodeHandle nh_;

  ros::Subscriber dig_site_location_;   // Redundant name? site and location?
  ros::Subscriber hauler_filled_sub_;   // Subscriber to wait till hauler is filled

  HAULER_STATES robot_state_ = HAULER_STATES::INIT;
  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  
  // To start and stop the state machine
  bool state_machine_continue_ = true;

  // Status of actionlib servers
  bool nav_server_idle_ = true;
  bool nav_vis_server_idle_ = true;
  bool hauler_server_idle_ = true;
  bool park_server_idle_ = true;
    
  // Variables to be set by subscriber callback
  bool volatile_found_ = false;
  bool hauler_filled_ = false;
  geometry_msgs::PoseStamped dig_site_pose_;

  // Actionlib servers' defining
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


  /**
   * @brief Callback for the location of found digging site
   * 
   * @param msg 
   */
  void digSiteLocCB(const geometry_msgs::PoseStamped &msg);


  /**
   * @brief Callback, as the signal for hauler to go back to hopper for dumping volatiles
   * 
   * @param msg 
   */
  void haulerFilledCB(const std_msgs::Empty &msg);


  /**
   * @brief Waits for the scout to find the volatile
   *        Basically does nothing
   *        Ideally, should be used to stay close to Excavator
   *          for minimising the time when the volatile is found
   * 
   */
  void initState();


  /**
   * @brief Currently, there are specific requirements for the parking to be successful. 
   *          These conditions are taken care of in this function.
   *          Condition: The Hauler should be in the lower 3rd quadrant (bottom right) of the excavator
   * 
   */
  void goToDigSite();


  /**
   * @brief Visual Navigation: Currently, takes hauler close to the excavator for parking 
   * 
   */
  void followExcavator();


  /**
   * @brief Parks the excavator for dumping
   * 
   */
  void parkAtExcavator();


  /**
   * @brief Hang on until excavator publishes a message denoting digging complete
   * 
   */
  void waitTillFilled();


  /**
   * @brief Visual navigation: Will take the hauler to processing plant
   * 
   */
  void goToProcPlant();

  /**
   * @brief Parks the hauler at the hopper for dumping the volatiles
   * 
   */
  void parkAtHopper();

  /**
   * @brief Dump the volatile in hopper
   * 
   */
  void dumpVolatile();

public:
  HaulerStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~HaulerStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // HAULER_STATE_MACHINE_H