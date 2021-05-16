#ifndef EXCAVATOR_STATE_MACHINE_H
#define EXCAVATOR_STATE_MACHINE_H

#include <ros/ros.h>
#include <iostream>
#include <operations/NavigationAction.h> 
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <operations/ExcavatorAction.h>
#include <operations/ExcavatorStateMachineTaskAction.h>
#include <std_msgs/Empty.h>
#include <perception/ObjectArray.h>
#include <geometry_msgs/PointStamped.h>


using namespace COMMON_NAMES;
 
std::string g_robot_name;
typedef actionlib::SimpleActionServer<operations::ExcavatorStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> EXCAVATOR_TASKS = {
  STATE_MACHINE_TASK::EXCAVATOR_INIT, 
  STATE_MACHINE_TASK::EXCAVATOR_KEEP_LOOKOUT, 
  STATE_MACHINE_TASK::EXCAVATOR_FIND_SCOUT, 
  STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT, 
  STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB, 
  STATE_MACHINE_TASK::EXCAVATOR_FIND_VOLATILE, 
  STATE_MACHINE_TASK::EXCAVATOR_DIG_VOLATILE, 
  STATE_MACHINE_TASK::EXCAVATOR_DUMP_VOLATILE, 
  STATE_MACHINE_TASK::EXCAVATOR_NEXT_QUE_TASK, 
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
  ros::Publisher return_hauler_pub_;   // WTF is this name??

  STATE_MACHINE_TASK robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_INIT;
  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  const double ROTATION_SPEED = 0.5;
  
  const int DIGGING_TRIES_ = 2; // BIG HACK FOR DEMO
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
  std::mutex hauler_parked_mutex;

  
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

  /**
   * @brief Subscribes to detected object topic and find scout's location.
   *        Updates the global stamped scout location.
   * 
   * @return true if scout was found
   * @return false if scout wasn't found
   */
  bool updateScoutLocation();

  /**
   * @brief Excavator starts rotating about the center, stops when scout is found.
   * 
   */
  void findScout();

public:
  ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name);

  ~ExcavatorStateMachine();

  friend void execute(const operations::ExcavatorStateMachineTaskGoalConstPtr& goal, SM_SERVER* as, ExcavatorStateMachine* sm);
  void startStateMachine();
  void stopStateMachine(); // Do we need it though?
};

#endif // EXCAVATOR_STATE_MACHINE_H