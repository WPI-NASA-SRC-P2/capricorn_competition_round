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

class ExcavatorScheduler : public RobotScheduler {
public:
   ExcavatorScheduler(uint32_t un_max_t) :
      m_unT(0),
      m_unMaxT(un_max_t) {}

   void step() override {
      /* Increase time counter */
      ++m_unT;
      //std::cout << "t = " << m_unT << std::endl;
      /* Call parent class step */
      RobotScheduler::step();
   }
   
   bool done() override {
      // return m_unT >= m_unMaxT;
      return false;
   }

   void setInterrupt(STATE_MACHINE_TASK interrupt_state) override{
      interrupt_state_ = interrupt_state;
      m_bInterrupt = true;
   }

private:

   uint32_t m_unT;
   uint32_t m_unMaxT;
};


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

/****************************************/
/****************************************/


class ExcavatorState : public State {
   
private:
  ros::Subscriber objects_sub_;
   /*** @param objs 
   */
  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

public:
   
   ExcavatorState(uint32_t un_id,
           uint32_t un_max_count);
   
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

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::ExcavatorGoal excavator_arm_goal_;

  typedef actionlib::SimpleActionClient<operations::ExcavatorAction> ExcavatorClient;
  ExcavatorClient *excavator_arm_client_;
  operations::NavigationGoal navigation_action_goal_;

};

class GoToScout : public ExcavatorState {
public:   
   GoToScout() : ExcavatorState(EXCAVATOR_GO_TO_SCOUT, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
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
   GoToDefaultArmPosition() : ExcavatorState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
  bool first_;
  int counter_;
};

class ParkAndPub : public ExcavatorState {
public:
   ParkAndPub() : ExcavatorState(EXCAVATOR_PARK_AND_PUB, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
  double begin_;
  double current_;
};

class DigAndDump : public ExcavatorState {
public:
   DigAndDump() : ExcavatorState(EXCAVATOR_DIG_AND_DUMP_VOLATILE, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool volatile_found_;
   bool dig_;
   bool dump_;
   int new_vol_loc_flag_;
   int digging_attempt_;
};
// #endif

