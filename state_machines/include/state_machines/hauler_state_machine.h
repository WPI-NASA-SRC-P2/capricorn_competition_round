/**
 * @file hauler_state_machine.h
 * @author Mahimana Bhatt (mbhatt@wpi.edu)
 * @brief Hauler state machine which controls all the operations done by hauler
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef HAULER_STATE_MACHINE_H
#define HAULER_STATE_MACHINE_H

#include <iostream>
#include <operations/NavigationAction.h>
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <operations/HaulerAction.h>
#include <geometry_msgs/PointStamped.h>
#include <state_machines/HaulerStateMachineTaskAction.h>
#include <operations/ParkRobotAction.h>
#include <srcp2_msgs/ScoreMsg.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::HaulerStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> HAULER_TASKS = {
    STATE_MACHINE_TASK::HAULER_GO_TO_LOC,
    STATE_MACHINE_TASK::HAULER_FOLLOW_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_PARK_AT_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_GO_TO_PROC_PLANT,
    STATE_MACHINE_TASK::HAULER_PARK_AT_HOPPER,
    STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE,
    STATE_MACHINE_TASK::HAULER_UNDOCK_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_UNDOCK_HOPPER,
    STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT,
    STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR,
};

class HaulerStateMachine
{

private:
  ros::NodeHandle nh_;

  std::string robot_name_;

  const double SLEEP_TIME = 0.5;

  // Actionlib servers' defining
  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::HaulerAction> HaulerClient;
  HaulerClient *hauler_client_;
  operations::HaulerGoal hauler_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::ParkRobotAction> ParkRobotClient;
  ParkRobotClient *park_robot_client_;
  operations::ParkRobotGoal park_robot_goal_;

  /**
   * @brief Currently, there are specific requirements for the parking to be successful. 
   *          These conditions are taken care of in this function.
   *          Condition: The Hauler should be in the lower 3rd quadrant (bottom right) of the excavator
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool goToLoc(const geometry_msgs::PoseStamped &loc); //Needs comment

  /**
   * @brief Visual Navigation: Currently, takes hauler close to the excavator for parking 
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool followExcavator();

  /**
   * @brief Parks the excavator for dumping
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool parkAtExcavator(std::string excavator_name = OBJECT_DETECTION_EXCAVATOR_CLASS);

  /**
   * @brief Visual navigation: Will take the hauler to processing plant
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool goToProcPlant();

  /**
   * @brief Parks the hauler at the hopper for dumping the volatiles
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool parkAtHopper();

  /**
   * @brief Dump the volatile in hopper
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool dumpVolatile();

  /**
   * @brief Undocks hauler from excavator
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool undockExcavator();

  /**
   * @brief Undocks hauler from hopper
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool undockHopper();

  /**
   * @brief Goes to processing plant using navigation or navigation vision,
   * parks the hauler wrt hopper, dumps the volatile and undocks the hauler from hopper
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool dumpVolatileToProcPlant();

  /**
   * @brief Goes back to excavator using navigation or navigation vision
   * 
   * @return true : if task is successful
   * @return false : if task is failed or aborted or interrupted
   */
  bool goBackToExcavator();

public:
  HaulerStateMachine(ros::NodeHandle nh, const std::string &robot_name);

  ~HaulerStateMachine();

  friend void cancelGoal(HaulerStateMachine *sm);
  friend void execute(const state_machines::HaulerStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, HaulerStateMachine *sm);
};

#endif // HAULER_STATE_MACHINE_H