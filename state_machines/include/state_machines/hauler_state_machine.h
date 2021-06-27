#pragma once 
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
#include <state_machines/RobotStateMachineTaskAction.h>
#include <operations/ParkRobotAction.h>
#include <srcp2_msgs/ScoreMsg.h>
#include <maploc/ResetOdom.h>

#include <state_machines/common_robot_state_machine.h>
#include "perception/ObjectArray.h"     

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

// class HaulerScheduler : public RobotScheduler {
// public:
//    HaulerScheduler(uint32_t un_max_t) :
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

//    // void setInterrupt(STATE_MACHINE_TASK interrupt_state) override{
//    //    interrupt_state_ = interrupt_state;
//    //    m_bInterrupt = true;
//    // }

// private:

//    uint32_t m_unT;
//    uint32_t m_unMaxT;
// };

const std::set<STATE_MACHINE_TASK> HAULER_TASKS = {
    STATE_MACHINE_TASK::HAULER_GO_TO_LOC,         
    STATE_MACHINE_TASK::HAULER_FOLLOW_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_PARK_AT_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_GO_TO_PROC_PLANT,
    STATE_MACHINE_TASK::HAULER_PARK_AT_HOPPER,
    STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE,
    STATE_MACHINE_TASK::HAULER_UNDOCK_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_UNDOCK_HOPPER,                //-> 2
    STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT,
    STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR,
    STATE_MACHINE_TASK::HAULER_RESET_ODOM,
    STATE_MACHINE_TASK::HAULER_RESET_ODOM_AT_HOPPER,         // -> 1
    STATE_MACHINE_TASK::HAULER_FACE_PROCESSING_PLANT,
    STATE_MACHINE_TASK::ROBOT_IDLE_STATE}; 

class HaulerState : public State {
   
private:
  ros::Subscriber objects_sub_;
   /*** @param objs 
   */
  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

public:
   
   HaulerState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name);
   
   ~HaulerState();

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

  ros::ServiceClient resetHaulerOdometryClient_;

  const double SLEEP_TIME = 0.5;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::HaulerAction> HaulerClient;
  HaulerClient *hauler_client_;
  operations::HaulerGoal hauler_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::ParkRobotAction> ParkRobotClient;
  ParkRobotClient *park_robot_client_;
  operations::ParkRobotGoal park_robot_goal_;
};

class GoToProcPlant : public HaulerState {
public:   
   GoToProcPlant(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_GO_TO_PROC_PLANT, nh, robot_name) {}

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

class ParkAtHopper : public HaulerState {
public:   
   ParkAtHopper(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_PARK_AT_HOPPER, nh, robot_name) {}

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
};

class ResetOdom : public HaulerState {
public:   
   ResetOdom(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_RESET_ODOM, nh, robot_name) {}

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
   maploc::ResetOdom reset_srv_;
};

class UndockHopper : public HaulerState {
public:   
   UndockHopper(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_UNDOCK_HOPPER, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {} ;

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
   double begin_;
   double current_;
};

// class FollowExcavator : public HaulerState {
// public:   
//    FollowExcavator() : HaulerState(HAULER_FOLLOW_EXCAVATOR, 10) {}

//    // define transition check conditions for the state (transition() is overriden by each individual state)
//    State& transition() override ;
   
//    // void entryPoint(const geometry_msgs::PoseStamped &target_loc) override;
//    void entryPoint() override;
//    void step() override;
//    void exitPoint() override;

// private: 
//    bool first_;
//    geometry_msgs::PoseStamped target_loc_;
// };

class GoToExcavator : public HaulerState {
public:   
   GoToExcavator(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_GO_BACK_TO_EXCAVATOR, nh, robot_name) {}

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

class ParkAtExcavator : public HaulerState {
public:   
   ParkAtExcavator(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_PARK_AT_EXCAVATOR, nh, robot_name) {}

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

class UndockExcavator : public HaulerState {
public:   
   UndockExcavator(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_UNDOCK_EXCAVATOR, nh, robot_name) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {} ;

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
   double begin_;
   double current_;
};

class DumpVolatile : public HaulerState {
public:   
   DumpVolatile(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_DUMP_VOLATILE, nh, robot_name) {}

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
};

class IdleState : public HaulerState {
public:   
   IdleState(ros::NodeHandle nh, std::string robot_name) : HaulerState(ROBOT_IDLE_STATE, nh, robot_name) {}

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


class ResetOdomMacro : public HaulerState {
public:   
   ResetOdomMacro(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_RESET_ODOM_AT_HOPPER, nh, robot_name) {}

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
};

// class HaulerIdle : public HaulerState {
// public:   
//    HaulerIdle(ros::NodeHandle nh, std::string robot_name) : HaulerState(ROBOT_IDLE_STATE, nh, robot_name) {}

//    // define transition check conditions for the state (transition() is overriden by each individual state)
//    State& transition() override ;

//    // define transition check conditions for the state (isDone() is overriden by each individual state)
//    bool isDone() override;
//    // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
//    bool hasSucceeded() override;

//    // void entryPoint(const geometry_msgs::PoseStamped &target_loc) override;
//    void entryPoint() override;
//    void step() override;
//    void exitPoint() override;

// private: 
//    bool first_;
// };

#endif // HAULER_STATE_MACHINE_H