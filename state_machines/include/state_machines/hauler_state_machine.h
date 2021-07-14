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
      ROS_INFO_STREAM("[ STATE_MACHINES | hauler_state_machine | " << getName() << "] - entry point ]");
   }
   //UNDERSTANDING: Every state might HAVE its own exitpoint. 
   void exitPoint() override {
      ROS_INFO_STREAM("[ STATE_MACHINES | hauler_state_machine | " << getName() << "] - exit point ]");
   }

   void step() override {
      ++m_unCount;
      ROS_INFO_STREAM("[ STATE_MACHINES | hauler_state_machine | " << getName() << "] - count = " << m_unCount << " ]");
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
  operations::NavigationVisionResult navigation_vision_result_;

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

/**
 * @brief GoToProcPlant navigate to proccessing plant
 * 
 * @param isDone() navigation vision (NAV+NAV_VISION) attempts are done. 
 * @param hasSucceeded() navigation vision (NAV+NAV_VISION) has succeeded
 */
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

/**
 * @brief HaulerGoToScout navigate to scout's location
 * 
 * @param isDone() navigation vision (NAV+NAV_VISION) is done
 * @param hasSucceeded() navigation vision (NAV+NAV_VISION) has succeeded
 */
class HaulerGoToScout : public HaulerState {
public:   
   HaulerGoToScout(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_GO_TO_SCOUT, nh, robot_name) {}

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
 * @brief ParkAtHopper dock at hopper
 * 
 * @param isDone() park robot actionlib is done
 * @param hasSucceeded()  park robot actionlib has succeeded
 */
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
   // reset odom service is called in the exitstate
   maploc::ResetOdom reset_srv_;
};

// class ResetOdom : public HaulerState {
// public:   
//    ResetOdom(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_RESET_ODOM, nh, robot_name) {}

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
//    maploc::ResetOdom reset_srv_;
// };


/**
 * @brief UndockHopper hauler undocks from hopper
 * 
 * @param isDone() navigation vision is done undocking from hopper
 * @param hasSucceeded() navigation vision succeeds at undocking from hopper 
 *                       and odom successfully reset
 * 
 */
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

   // reset odometry helper function
   void resetOdom();

private: 
   bool first_;
   geometry_msgs::PoseStamped target_loc_;
   double begin_;
   double current_;
   bool reset_succeeded_;
   bool undock_done_;
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

/**
 * @brief GoToExcavator navigates to excavator
 * 
 * @param isDone() navigation vision (V_NAV_AND_NAV_VISION) is done getting to excavator
 * @param hasSucceeded() naviagtion vision (V_NAV_AND_NAV_VISION) succeeds at getting to excavator
 * 
 */
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

/**
 * @brief ParkAtExcavator park in front of excavator robot to be within range of the arm
 * 
 * @param isDone park robot actionlib is done
 * @param hasSucceeded park robot actionlib has succeeded
 */
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
   std::string excavator_name_;
};

/**
 * @brief UndockExcavator undocks from excavator
 * 
 * @param isDone() navigation vision is done undocking
 * @param hasSucceeded navigation vision has succeeded in undocking
 */
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

/**
 * @brief DunpVolatile dumps volatiles
 * 
 * @param isDone() hauler dumping of volatile is done.
 * @param hasSucceeded() hauler dumping of volatile has succeeded.
 * 
 */
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

/**
 * @brief HaulerGoToLoc naviagte to pose using NAV_TYPE::GOAL
 * 
 * @param isDone() navigation is done getting to position
 * @param hasSucceeded() navigation has succeeded at getting to position
 */
class HaulerGoToLoc : public HaulerState {
public:   
   HaulerGoToLoc(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_GO_TO_LOC, nh, robot_name) {}

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
 * @brief IdleState Robot should stop and do nothing (Useful to discontinue a bad state)
 * 
 * @param isDone() goals have been send to stop the robot
 * @param hasSucceded() goals sent have been successfull
 */
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

   void entryPoint() override 
   {
      hauler_client_->cancelGoal();
      navigation_vision_client_ ->cancelGoal();
      navigation_client_->cancelGoal();
      park_robot_client_->cancelGoal();
   }
   void step() override{}
   void exitPoint() override{}
   State& transition() override{} 
};

/**
 * @brief DumpVolatileAtHopper Executes the following:
 *    [0] Navigates to Proc_plant
 *    [1] Parks to the hopper
 *    [2] Dunps volatile
 *    [3] Undocks from hopper to prevent drift accumulation after resetting odom due to 180 deg turn
 *    [4] Resets odom
 * 
 * @param isDone() is done when it has reset Odom
 * @param hasSucceeded() has successfully reset odom
 */
class DumpVolatileAtHopper : public HaulerState {
public:   
   DumpVolatileAtHopper(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_DUMP_VOLATILE_TO_PROC_PLANT, nh, robot_name) {}

   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;
   State& transition() override {}

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   void goToProcPlant();
   void parkAtHopper();
   void dumpVolatile();
   void undockFromHopper();
   void resetOdom();
   void goToRepairStation();
   void idleScout(){}

   bool first_GTPP, first_PAH, first_UFH, first_DV, first_ROH, first_GTRS, macro_state_succeeded, macro_state_done;
   
   enum RESET_ODOM_MICRO_STATES{
      GO_TO_PROC_PLANT,
      PARK_AT_HOPPER,
      DUMP_VOLATILE,
      UNDOCK_FROM_HOPPER,
      RESET_ODOM_AT_HOPPER,
      GO_TO_REPAIR_STATION,
      HAULER_IDLE
   };

   bool state_done = false;
   RESET_ODOM_MICRO_STATES micro_state;
};

/**
 * @brief HaulerGoToRepairStation naviagte to repair station
 * 
 * @param isDone() navigation vision is done getting to the repair station
 * @param hasSucceeded() navigation vision has succeeded at getting to the repair station
 */
class HaulerGoToRepairStation : public HaulerState {
public:   
   HaulerGoToRepairStation(ros::NodeHandle nh, std::string robot_name) : HaulerState(HAULER_GOTO_REPAIR_STATION, nh, robot_name) {}

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