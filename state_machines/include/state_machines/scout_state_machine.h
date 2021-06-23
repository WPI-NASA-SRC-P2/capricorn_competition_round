#pragma once

#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <operations/NavigationVisionAction.h>
#include <operations/NavigationAction.h>
#include <operations/Spiral.h>
#include <operations/ResourceLocaliserAction.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <operations/obstacle_avoidance.h>
#include <state_machines/common_robot_state_machine.h>
#include <maploc/ResetOdom.h>

using namespace COMMON_NAMES;


/****************************************/
/****************************************/

class ScoutScheduler : public RobotScheduler {
   
public:

   ScoutScheduler(uint32_t un_max_t) :
      m_unT(0),
      m_unMaxT(un_max_t) {}

   void step() override {
      /* Increase time counter */
      ++m_unT;
      // std::cout << "t = " << m_unT << std::endl;
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
const std::set<STATE_MACHINE_TASK> SCOUT_TASKS = {
    STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_STOP_SEARCH,
    STATE_MACHINE_TASK::SCOUT_LOCATE_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_UNDOCK,
    STATE_MACHINE_TASK::SCOUT_RESET_ODOM_GROUND_TRUTH,
    STATE_MACHINE_TASK::SCOUT_RESET_ODOM,
    STATE_MACHINE_TASK::SCOUT_SYNC_ODOM,
    STATE_MACHINE_TASK::SCOUT_FACE_PROCESSING_PLANT};



/****************************************/
/****************************************/


class ScoutState : public State {
   
private:
  ros::Subscriber volatile_sub_;
  ros::Subscriber objects_sub_;

  /**
   * @brief Volatile sensor callback
   * 
   * @param msg 
   */
  void volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

  /**
   * @brief Object detection objects callback
   * 
   * @param objs 
   */
  void objectsCallback(const perception::ObjectArray::ConstPtr objs);

public:
   
   ScoutState(uint32_t un_id,
           uint32_t un_max_count);
   
   ~ScoutState();

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

  ros::ServiceClient spiralClient_;
  
  bool near_volatile_ = false;       //
  bool new_volatile_msg_ = false; 

  std::mutex objects_mutex_;
  perception::ObjectArray::ConstPtr vision_objects_;
  bool objects_msg_received_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

};

class Undock : public ScoutState {
public:   
   Undock() : ScoutState(SCOUT_UNDOCK, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool first_;
   operations::NavigationGoal navigation_action_goal_;
};

class Search : public ScoutState {
public:   
   Search() : ScoutState(SCOUT_SEARCH_VOLATILE, 10) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   operations::Spiral srv;

};

class Locate : public ScoutState {
public:   
   Locate() : ScoutState(SCOUT_LOCATE_VOLATILE, 3) {}

   // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   operations::ResourceLocaliserGoal goal;
   bool first_;
};



// class Undock: public ScoutState
// {
// public:
//   Undock();
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
// };

// class Search: public ScoutState
// {
// public:
//   Search();
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
//   bool resumeSearchingVolatile(bool resume);
// };

// class Locate: public ScoutState
// {
// public:
//   Locate();
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
// };

// class SolarCharge: public ScoutState
// {
// public:
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
// };

// class RepairRobot: public ScoutState
// {
// public:
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
// };

// class ResetOdom: public ScoutState
// {
// public:
//   bool entryPoint();
//   bool exec();
//   bool exitPoint();
// };