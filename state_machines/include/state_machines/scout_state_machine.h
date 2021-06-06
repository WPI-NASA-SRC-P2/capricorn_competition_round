#pragma once

#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <operations/NavigationVisionAction.h>
#include <operations/Spiral.h>
#include <operations/ResourceLocaliserAction.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <maploc/ResetOdom.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> SCOUT_TASKS = {
    STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_STOP_SEARCH,
    STATE_MACHINE_TASK::SCOUT_LOCATE_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_UNDOCK,
    STATE_MACHINE_TASK::SCOUT_RESET_ODOM_GROUND_TRUTH,
    STATE_MACHINE_TASK::SCOUT_RESET_ODOM,
    STATE_MACHINE_TASK::SCOUT_SYNC_ODOM,
    STATE_MACHINE_TASK::SCOUT_FACE_PROCESSING_PLANT};

class ScoutStateMachine
{
private:
  ros::NodeHandle nh_;

  std::string robot_name_;

  ros::ServiceClient spiralClient_;
  ros::ServiceClient resetScoutOdometryClient_;
  ros::Subscriber volatile_sub_;
  bool near_volatile_ = false;
  bool new_message_received = false;
  bool continue_spiral_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

  /**
   * @brief Start the searching algorithm
   * 
   */
  bool startSearchingVolatile();

  /**
   * @brief Stop the searching algorithm
   * 
   */
  bool stopSearchingVolatile();

  /**
   * @brief Sets the status of searching algorithm according to argument resume
   * 
   */
  bool resumeSearchingVolatile(const bool resume);

  /**
   * @brief Pinpoint the location of the volatile, and stand on it
   * 
   */
  bool locateVolatile();

  /**
   * @brief Move away from the excavator to continue on the trajectory
   *        CURRENTLY NOT IMPLEMENTED
   * 
   */
  bool undockRobot();

  bool undockRobot(const geometry_msgs::PoseStamped &POSE);

  /**
   * @brief Volatile sensor callback
   * 
   * @param msg 
   */

  void volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

  /**
   * @brief resets odometry, used after parking is done for scout, the oone withj pose signature is used when ground truth is not being used.
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted, or if the service is called for a second time in one simulation session for the ground truth version.
   */
  bool resetOdometry(const geometry_msgs::PoseStamped &POSE);

  /**
   * @brief resets odometry, used after parking is done for scout, the oone withj pose signature is used when ground truth is not being used.
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted, or if the service is called for a second time in one simulation session for the ground truth version.
   */
  bool resetOdometry();

  /**
   * @brief centers scout wrt processing plant and then resets the odometry according to whatever pose we pass it.
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted
   */
  bool syncOdometry(const geometry_msgs::PoseStamped &POSE);

  /**
   * @brief centers scout wrt processing plant.
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted
   */
  bool faceProcessingPlant();

  /**
   * @brief Wait till all the actionlib and service servers are started
   * 
   */
  void waitForServerConnections();

  bool goToLocObject(const geometry_msgs::PoseStamped &target_loc, std::string target_object);
  /**
   * @brief Goes to location with a combination of navigation and navigation vision 
   * 
   * @return true 
   * @return false 
   */

public:
  ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name);

  ~ScoutStateMachine();

  friend void cancelGoal(ScoutStateMachine *sm);
  friend void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ScoutStateMachine *sm);
};

// #endif // SCOUT_STATE_MACHINE_H