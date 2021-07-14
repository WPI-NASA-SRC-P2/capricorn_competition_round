/**
 * @file excavator_state_machine.h
 * @author Team Bebop(mmuqeetjibran@wpi.edu)
 * @brief Excavator state machine which controls all the operations done by excavator
 * @version 0.1
 * @date 2021-06-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// #ifndef EXCAVATOR_STATE_MACHINE_H
// #define EXCAVATOR_STATE_MACHINE_H

#include <iostream>
#include <operations/NavigationAction.h>
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <operations/ExcavatorAction.h>
#include <geometry_msgs/PointStamped.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <state_machines/common_robot_state_machine.h>
// #include <operations/obstacle_avoidance.h>
#include <maploc/ResetOdom.h>
#include "perception/ObjectArray.h"                    //Not sure why this had to be added, wasnt needed in scout sm, including the obstacle_avoidance header causes problems
#include <operations/ParkRobotAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

using namespace COMMON_NAMES;


const std::set<STATE_MACHINE_TASK> EXCAVATOR_TASKS = {
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC,
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT,
    STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB,
    STATE_MACHINE_TASK::EXCAVATOR_PRE_HAULER_PARK_MANEUVER,
    STATE_MACHINE_TASK::EXCAVATOR_DIG_AND_DUMP_VOLATILE,
    STATE_MACHINE_TASK::EXCAVATOR_GOTO_DEFAULT_ARM_POSE,
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_GROUND_TRUTH,
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_SYNC_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_FACE_PROCESSING_PLANT,
    STATE_MACHINE_TASK::EXCAVATOR_VOLATILE_RECOVERY,
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR

/****************************************/
/****************************************/
};

class ExcavatorState : public State {
   
private:
  ros::Subscriber objects_sub_;
  ros::Subscriber odom_sub_;

   /*** @param objs 
   */
  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

  /** @param odom
   * Odometry callback for excavator_pose_
  */
  void odomCallback(const nav_msgs::Odometry odom);

public:
   
   ExcavatorState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name);
   
   ~ExcavatorState();

   //UNDERSTANDING: Trigerred in the setInitialState()
   void entryPoint() override {
      m_unCount = 0;
      ROS_INFO_STREAM("[ STATE_MACHINES | excavator_state_machine |  [" << getName() << "] - entry point ]");
   }
   //UNDERSTANDING: Every state might HAVE its own exitpoint. 
   void exitPoint() override {
      ROS_INFO_STREAM("[ STATE_MACHINES | excavator_state_machine |  [" << getName() << "] - exit point ]");
   }

   void step() override {
      ++m_unCount;
      ROS_INFO_STREAM("[ STATE_MACHINES | excavator_state_machine |  [" << getName() << "] - count = " << m_unCount << " ]");
   }

protected:

  uint32_t m_unMaxCount;
  uint32_t m_unCount;

  ros::NodeHandle nh_;

  std::string robot_name_;

  // For odometry callback
  nav_msgs::Odometry odom_;
//   geometry_msgs::Pose excavator_pose_;
  geometry_msgs::PoseStamped excavator_pose_;

  std::mutex objects_mutex_;
  perception::ObjectArray::ConstPtr vision_objects_;
  bool objects_msg_received_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;
  operations::NavigationVisionResult navigation_vision_result_;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::ExcavatorGoal excavator_arm_goal_;
//   operations::ExcavatorResult excavator_arm_result_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient *excavator_arm_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ParkRobotAction> ParkRobotClient;
  ParkRobotClient *park_robot_client_;
  operations::ParkRobotGoal park_robot_goal_;

/** @brief:
 * Parameters that have to be tuned through testing.*/ 
   float PARAM_EXCAVATOR_HIT_SCOUT = 1.2;  //In STATE 10: EXCAVATOR_PARK_AND_PUB, how far the excavator should go so that it 'just touches' the scout, assuming it is centered to scout.

};

/**
 * @brief GoToScout Excavator attempts to go to the scout's location by:
 *    [1] using scout pose to get to relative location
 *    [2] use nav vision to more reliably get within range of scout
 * 
 *  @param isDone() naviagtion vision completed the task
 *  @param hasSucceeded() naviation vision succeeded in going to Scout
 */
class GoToScout : public ExcavatorState {
public:   
   GoToScout(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_GO_TO_SCOUT, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   // void entryPoint(const geometry_msgs::PoseStamped &target_loc) override;
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool first_;
   geometry_msgs::PoseStamped target_loc_;
};

/**
 * @brief GoToDefaultArmPosition send excavator's arm to our 
 *       default position to avoid collion with other robots and 
 *       for easier detection by other robots
 * 
 * @param isDone() excavator arm is done going to default position
 * @param hasSucceded() excavator arm succeeded in going to it's default arm position
 */
class GoToDefaultArmPosition : public ExcavatorState {
public:   
   GoToDefaultArmPosition(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
  bool first_;
};

/**
 * @brief ParkAndPub docks the excavator to the scout by:   
 *    [1] Centers the excavator to the scout
 *    [2] Moves forward some meters 
 *       
 * @param isDone() navagation vision completded task as mentioned
 * @param hasSucceded() navaigation vision succeeded in both task
 * 
 */
class ParkAndPub : public ExcavatorState {
public:
   ParkAndPub(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_PARK_AND_PUB, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {};
   
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

   //helper functions to navigate to toward Scout 
   void navToScout();       //centering code (isnt really needed if the previous state is already centering)
   void closeInToScout();   //moves forward for X duration

private: 
  bool first_;
  /** only used for timed stuff-- TODO: delete when no longer needed*/
  double begin_;
  double current_;
   //   int step_;
   const float crash_time_ = 2.7;  //time to move forward
};

/**
 * @brief DigAndDump digs and dumps voltiles into hauler until no volatile remains in that area
 * 
 * @param isDone() :  when there is no volatile in scoop
 * @param hasSucceded() : When there have been volatiles that have been dug, and no more remain
 */
class DigAndDump : public ExcavatorState {
public:
   DigAndDump(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_DIG_AND_DUMP_VOLATILE, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

   // helper functions for digging and dumping the volatile
   void digVolatile();
   void dumpVolatile();

private: 
   bool volatile_found_;
   bool done_digging_;
   bool digging_server_succeeded_;
   bool last_state_dig_;
   bool dig_;
   bool dump_;
   int new_vol_loc_flag_;
   int digging_attempt_ = 0;
};

/**
 * @brief IdleState Robot should stop and do nothing ==> CANCELS ALL GOALS SENT BY ALL CLIENTS 
 * 
 * @param isDone() goals have been send to stop the robot
 * @param hasSucceded() goals sent have been successfull
 * 
 */
class IdleState : public ExcavatorState {
public:   
   IdleState(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(ROBOT_IDLE_STATE, nh, robot_name) {}

   bool isDone() override{ 
      current_state_done_ = true;
      return true; }
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override{ 
      last_state_succeeded_ = true;
      return true; }

   void entryPoint() override 
   {
   excavator_arm_client_->cancelGoal();
   navigation_vision_client_ ->cancelGoal();
   navigation_client_->cancelGoal();
   park_robot_client_->cancelGoal();

   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Excavator has entered idle state, awaiting new state...");
   }
   void step() override{}
   void exitPoint() override{}
   State& transition() override{} 
};

/**
 * @brief PreParkHauler docks the excavator to the scout by: 
 * 
 *    [1] Moves forward some distance to be ontop of volatile
 *    [2] Rotates until excavator's vision is centered to hauler
 *    [3] Moves back to have excavator's arm over volatile spot
 *     
 *  @param isDone() if arm is above the voltile 
 *  @param hasSucceeded() excavator is centered to hauler and arm is above volatile
 * 
 */
class PreParkHauler : public ExcavatorState {
public:   
   PreParkHauler(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_PRE_HAULER_PARK_MANEUVER, nh, robot_name) {}

   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;
   State& transition() override{}; 

   // movements for getting into position at volatile w.r.t. hauler
   void goToVolatile();
   void centerHauler();
   void getInArmPosition();

private:
   bool first_;
   int goal_;
   enum goal_states_
   {
      GO_TO_VOLATILE = 1,
      CENTER_TO_HAULER = 2,
      GET_IN_DIGGING_POSITION = 3
   };

   bool goToVolatileDone_;
   bool centerHaulerDone_;
   bool getInArmPositionDone_;
};

/**
 * @brief ExcavatorGoToLoc go to a pose 
 * 
 * @param isDone() navigation is done
 * @param hasSucceeded()  navigation succeeds to get to pose with referance to odom
 */
class ExcavatorGoToLoc : public ExcavatorState {
public:   
   ExcavatorGoToLoc(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_GO_TO_LOC, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {};
   
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   // void entryPoint(const geometry_msgs::PoseStamped &target_loc) override;
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool first_;
   geometry_msgs::PoseStamped target_loc_;
};

/**
 * @brief ExcavatorResetOdomAtHopper navigates to hopper, and then resets odom
 * 
 * @param isDone() when naviagtion vision is complete
 * @param hasSucceeded(); when navigation vision succeeds, and resetOdom msg was sent 
 * 
 */
class ExcavatorResetOdomAtHopper: public ExcavatorState {
public:   
   ExcavatorResetOdomAtHopper(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_RESET_ODOM_AT_HOPPER, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {};
   
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   // void entryPoint(const geometry_msgs::PoseStamped &target_loc) override;
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool first_;
   geometry_msgs::PoseStamped target_loc_;
   void goToProcPlant();
   void parkAtHopper();
   void undockFromHopper();
   void resetOdom();
   void goToRepair();
   void idleExcavator(){}

   bool first_GTPP, first_PAH, first_UFH, first_GTR, resetOdomDone_, macro_state_succeeded, macro_state_done;
   
   bool state_done;

   enum RESET_ODOM_MICRO_STATES{
      GO_TO_PROC_PLANT,
      PARK_AT_HOPPER,
      UNDOCK_FROM_HOPPER,
      RESET_ODOM_AT_HOPPER,
      GO_TO_REPAIR_STATION,
      EXCAVATOR_IDLE 
   };

   RESET_ODOM_MICRO_STATES micro_state;

};

/**
 * @brief ExcavatorGoToRepairStation travel to the Repair Station
 * 
 * @param isDone() navigation vision is complete
 * @param hasSucceeded() navigation succeeded in going to Repair Station
 * 
 */ 
class ExcavatorGoToRepairStation : public ExcavatorState {
public:   
   ExcavatorGoToRepairStation(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_GO_TO_REPAIR, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;
   State& transition() override{}

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool first_;
};

/**
 * @brief If GoToScout fails, this recovery state creates 4 targets located at 4 corners from it
 *        and looks for the scout at each of those corners. If the scout is found at those corners, it exits the state.
 * 
 * @param search_offset_ This is the offset of those targets from initial excavator position. Currently = 10.0 m.
 */
class ExcavatorGoToScoutRecovery : public ExcavatorState {
public:   
   ExcavatorGoToScoutRecovery(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_GO_TO_SCOUT_RECOVERY, nh, robot_name) {}
   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;
   State& transition() override{}
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

   void createPoses();
   void searchForScout(int index);

private:
   bool first_;
   // geometry_msgs::PoseArray recovery_poses_;
   geometry_msgs::PoseStamped recovery_poses_[4];
   geometry_msgs::PoseStamped recovery_pose_;
   geometry_msgs::PoseStamped target_loc_;

   int pose_index_;
   float search_offset_;
   bool search_done_, scout_found_, searches_exhausted_;

};

/**
 * @brief ExcavatorGoToRepairStation travel to the Repair Station
 * 
 * @param isDone() navigation vision is complete
 * @param hasSucceeded() navigation succeeded in going to Repair Station
 * 
 */ 
class VolatileRecovery : public ExcavatorState {
public:   
   VolatileRecovery(ros::NodeHandle nh, std::string robot_name) : ExcavatorState(EXCAVATOR_VOLATILE_RECOVERY, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;
   State& transition() override{}

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

   void checkVolatile();
   void crossMovement(const int& trial);
   void goToDefaultArmPose();

private:
   bool first_;
   int trial_;
   bool movement_done_;
   bool default_arm_done_;
   bool volatile_check_done_;
   bool volatile_found_, macro_state_done_, trials_exhausted_;
   enum VOLATILE_RECOVERY_MICRO_STATES{
      CROSS_MOVEMENT,
      CHECK_VOLATILE,
      DEFAULT_ARM_POSE
   };

   VOLATILE_RECOVERY_MICRO_STATES substate_;
};

// #endif

