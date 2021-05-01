#ifndef EXCAVATOR_STATE_MACHINE_H
#define EXCAVATOR_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <operations/ResourceLocaliserAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <operations/ExcavatorAction.h>
#include <std_msgs/Empty.h>

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
  DIG_VOLATILE,
  DUMP_VOLATILE,    // Start digging and dumping into the hauler
                        // This must check if hauler is close, else wait
  NEXT_QUE_TASK     // Inform the team level state machine that task completed, 
                        // Follow further instructions
};


class ExcavatorStateMachine
{

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_scout_vol_location_;
  ros::Publisher excavator_ready_pub_;

  EXCAVATOR_STATES robot_state_ = EXCAVATOR_STATES::INIT;
  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  
  bool state_machine_continue_ = true;
  bool volatile_found_ = false;
  bool nav_server_idle_ = true;
  bool excavator_server_idle_ = true;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient* navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient* excavator_arm_client_;
  operations::ExcavatorGoal excavator_arm_goal_;

  geometry_msgs::PoseStamped vol_pose_;


  /**
   * @brief Callback for the location of found volatile
   * 
   * @param msg 
   */
  void scoutVolLocCB(const geometry_msgs::PoseStamped &msg);


  /**
   * @brief Waits for the scout to find the volatile
   *        Basically does nothing
   *        Ideally, should be used to stay close to scout
   *          for minimising the time when the volatile is found
   */
  void initState();

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  void goToVolatile();

  /**
   * @brief Goes to the actual location where the volatile was predicted. 
   * 
   */
  void parkExcavator();

  /**
   * @brief Dig the volatile location
   * 
   */
  void digVolatile();

  /**
   * @brief Dump the volatile at Hauler Location
   * 
   */
  void dumpVolatile();

public:
  ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ExcavatorStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // EXCAVATOR_STATE_MACHINE_H