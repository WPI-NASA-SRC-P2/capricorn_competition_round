#ifndef EXCAVATOR_STATE_MACHINE_H
#define EXCAVATOR_STATE_MACHINE_H

#include <iostream>
#include <operations/NavigationAction.h>
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <operations/ExcavatorAction.h>
#include <geometry_msgs/PointStamped.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <maploc/ResetOdom.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> EXCAVATOR_TASKS = {
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC,
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT,
    STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB,
    STATE_MACHINE_TASK::EXCAVATOR_DIG_AND_DUMP_VOLATILE,
    STATE_MACHINE_TASK::EXCAVATOR_GOTO_DEFAULT_ARM_POSE,
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_GROUND_TRUTH,
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_SYNC_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_FACE_PROCESSING_PLANT,
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR};

class ExcavatorStateMachine
{
private:
  ros::NodeHandle nh_;

  std::string robot_name_;

  ros::ServiceClient resetExcavatorOdometryClient_;

  const double SLEEP_TIME = 0.5;
  const double ROTATION_SPEED = 0.5;

  const int DIGGING_TRIES_ = 2; // BIG HACK FOR DEMO
  int digging_attempt_ = 0;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient *excavator_arm_client_;
  operations::ExcavatorGoal excavator_arm_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  bool goToScout();

  /**
   * @brief Once the goal is received, go close to the predicted volatile location. 
   *          Publish that the excavator has reached close enough for scout to move out
   * 
   */
  bool goToLoc(const geometry_msgs::PoseStamped &loc); //Needs comment

  /**
   * @brief Goes to the actual location where the volatile was predicted. 
   * 
   */
  bool parkExcavator();

  /**
   * @brief Dig the volatile location
   * 
   */
  bool digVolatile();

  /**
   * @brief Dump the volatile at Hauler Location
   * 
   */
  bool dumpVolatile();

  /**
   * @brief Digs and dump continuously until volatile is available
   * 
   * @return true 
   * @return false 
   */
  bool digAndDumpVolatile();

  /**
   * @brief Takes excavator arm to default arm position (used for object detection)
   * 
   * @return true 
   * @return false 
   */
  bool goToDefaultArmPosition();

  bool resetOdometry(const geometry_msgs::PoseStamped &POSE);
  bool resetOdometry();

  bool faceProcessingPlant();
  /**
   * @brief centers excavator wrt processing plant
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted
   */

  bool syncOdometry(const geometry_msgs::PoseStamped &POSE);
  /**
   * @brief centers excavator wrt processing plant and then resets the odometry according to whatever pose we pass it.
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted
   */
  bool goToRepairStation();
  /**
   * @brief REACHES THE REPAIR STATION
   * 
   * @return true : if task is successful.
   * @return false : if task is failed or aborted or interrupted
   */

public:
  ExcavatorStateMachine(ros::NodeHandle nh, const std::string &robot_name);

  ~ExcavatorStateMachine();

  friend void cancelGoal(ExcavatorStateMachine *sm);
  friend void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ExcavatorStateMachine *sm);
};

#endif // EXCAVATOR_STATE_MACHINE_H