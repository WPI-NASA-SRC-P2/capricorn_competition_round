#ifndef EXCAVATOR_STATE_MACHINE_H
#define EXCAVATOR_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <operations/ExcavatorAction.h>
#include <std_msgs/Empty.h>
#include <perception/ObjectArray.h>
#include <geometry_msgs/PointStamped.h>


using namespace COMMON_NAMES;

enum EXCAVATOR_STATES
{
  INIT = 0,         // Wait for Instructions
                        // or it may get close to scout
  KEEP_LOOKOUT,     // Takes Excavator to a location from which it will
                        // be quicker to get to the digging location
  FIND_SCOUT,
  GO_TO_SCOUT,   // Get close to the volatile when it is detected
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
  ros::Subscriber lookout_pos_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber hauler_parked_sub_;

  ros::Publisher excavator_ready_pub_;
  ros::Publisher park_hauler_pub_;
  ros::Publisher hauler_go_back_;   // WTF is this name??

  EXCAVATOR_STATES robot_state_ = EXCAVATOR_STATES::INIT;
  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  const double ROTATION_SPEED = 0.5;
  
  const int digging_tries_ = 2; // BIG HACK FOR DEMO
  int digging_attempt_ = 0;

  bool state_machine_continue_ = true;
  bool lookout_loc_received_ = false;
  bool lookout_reached_ = false;
  bool volatile_found_ = false;
  bool nav_server_idle_ = true;
  bool nav_vis_server_idle_ = true;
  bool excavator_server_idle_ = true;
  bool hauler_parked_ = false;
  bool clods_in_scoop_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient* navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient* excavator_arm_client_;
  operations::ExcavatorGoal excavator_arm_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient* navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  geometry_msgs::PoseStamped next_nav_goal_;  // Having a shared goal is not a good idea
                                              // This is very problematic when messages
                                              // are received out of expected order
  geometry_msgs::PointStamped scout_loc_stamp_;

  perception::ObjectArray g_objects_;
  
  std::mutex g_objects_mutex;
  std::mutex navigation_mutex;

  
  /**
   * @brief Callback for the location of lookout pose
   * 
   * @param msg 
   */
  void lookoutLocCB(const geometry_msgs::PoseStamped &msg);

  void haulerParkedCB(std_msgs::Empty msg);

  /**
   * @brief Callback for the location of found volatile
   * 
   * @param msg 
   */
  void scoutVolLocCB(const geometry_msgs::PoseStamped &msg);

  /**
   * @brief Callback function which subscriber to Objects message published from object detection
   * 
   * @param objs 
   */
  void objectsCallback(const perception::ObjectArray& objs);

  /**
   * @brief Waits for the scout to find the volatile
   *        Basically does nothing
   *        Ideally, should be used to stay close to scout
   *          for minimising the time when the volatile is found
   * 
   */
  void initState();

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  void goToScout();

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  void goToLookout(); //Needs comment

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

  bool updateScoutLocation();

  void findScout();

public:
  ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ExcavatorStateMachine();

  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // EXCAVATOR_STATE_MACHINE_H