#pragma once

#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <operations/NavigationVisionAction.h>
#include <operations/Spiral.h>
#include <operations/ResourceLocaliserAction.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <operations/obstacle_avoidance.h>
#include <state_machines/common_robot_state_machine.h>

using namespace COMMON_NAMES;

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

  bool near_volatile_ = false;
  bool new_volatile_msg_ = false;

  std::mutex objects_mutex_;
  perception::ObjectArray::ConstPtr vision_objects_;
  bool objects_msg_received_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

};

class Undock : public ScoutState {
public:   
   Undock() : ScoutState(SCOUT_UNDOCK, 3) {}

  //  void entryPoint() override;
  // define transition check conditions for the state (transition() is overriden by each individual state)
   State& transition() override {
      if(m_unCount < m_unMaxCount) {
         // if no transition, stay in current state
         return *this;
      }
      return getState(SCOUT_SEARCH_VOLATILE);
   }
   
};

class Search : public ScoutState {
public:   
   Search() : ScoutState(SCOUT_SEARCH_VOLATILE, 3) {}

  //  void entryPoint() override;
   State& transition() override {
      // @todo:   check if (!near_volatile) 
      if(m_unCount < m_unMaxCount) {
         return *this;
      }
      return getState(LOCATE_VOLATILE);
   }
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
