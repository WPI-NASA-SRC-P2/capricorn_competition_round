/**
 * @file hauler_state_machine.h
 * @author Team Bebop(mmuqeetjibran@wpi.edu)
 * @brief Hauler state machine which controls all the operations done by hauler
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

using namespace COMMON_NAMES;

// typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

/****************************************/
/****************************************/

// class ExcavatorScheduler : public RobotScheduler {
// public:
//    ExcavatorScheduler(uint32_t un_max_t) :
//       m_unT(0),
//       m_unMaxT(un_max_t) {}

//    void step() override {
//       /* Increase time counter */
//       ++m_unT;
//       //std::cout << "t = " << m_unT << std::endl;
//       /* Call parent class step */
//       RobotScheduler::step();
//    }
   
//    bool done() override {
//       // return m_unT >= m_unMaxT;
//       return false;
//    }

//    // void setInterrupt(STATE_MACHINE_TASK interrupt_state) override{  //Add only if its there in the base class. 
//    //    interrupt_state_ = interrupt_state;
//    //    m_bInterrupt = true;
//    // }

// private:

//    uint32_t m_unT;
//    uint32_t m_unMaxT;
// };


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
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR};

/****************************************/
/****************************************/

class ExcavatorState : public State {
   
private:
  ros::Subscriber objects_sub_;
   /*** @param objs 
   */
  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

public:
   
   ExcavatorState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name);
   
   ~ExcavatorState();

   //UNDERSTANDING: Trigerred in the setInitialState()
   void entryPoint() override {
      m_unCount = 0;
      ROS_INFO_STREAM("  [" << getName() << "] - entry point");
   }
   //UNDERSTANDING: Every state might HAVE its own exitpoint. 
   void exitPoint() override {
      ROS_INFO_STREAM("  [" << getName() << "] - exit point");
   }

   void step() override {
      ++m_unCount;
      ROS_INFO_STREAM("  [" << getName() << "] - count = " << m_unCount);
   }

protected:

  uint32_t m_unMaxCount;
  uint32_t m_unCount;

  ros::NodeHandle nh_;

  std::string robot_name_;

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

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient *excavator_arm_client_;
  operations::NavigationGoal navigation_action_goal_;

};

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
 * @brief the previous state "go to scout" needs to be centered with the scout for this statemachine to work successfully 
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
   bool digging_server_succeeded_;
   bool last_state_dig_;
   bool dig_;
   bool dump_;
   int new_vol_loc_flag_;
   int digging_attempt_ = 0;
};

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

   void entryPoint() override{}
   void step() override{}
   void exitPoint() override{}
   State& transition() override{} 
};

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

private:
   bool first_;
   


};

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
// #endif

