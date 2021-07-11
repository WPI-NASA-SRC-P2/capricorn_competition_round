/**
 * @file scout_state_machine.h
 * @author Team Bebop(mmuqeetjibran@wpi.edu)
 * @brief Hauler state machine which controls all the operations done by hauler
 * @version 0.1
 * @date 2021-06-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
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
#include <state_machines/robot_state_status.h>
#include <state_machines/robot_desired_state.h>
#include <maploc/ResetOdom.h>
#include <operations/ParkRobotAction.h>
#include <maploc/ResetOdom.h>

using namespace COMMON_NAMES;


/****************************************/
/****************************************/

// class ScoutScheduler : public RobotScheduler {
   
// public:

//    ScoutScheduler(){}

//    void step() override {
//       RobotScheduler::step();
//    }

//    bool done() override {
//       return false;
//    }

//    // void setState(STATE_MACHINE_TASK new_state) override{
//    void setState(STATE_MACHINE_TASK new_state) {
//       new_state_ = new_state;
//       new_state_request = true;
//    }
// };

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
   
   ScoutState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name);
   
   ~ScoutState();

   //UNDERSTANDING: Trigerred in the setInitialState()
   void entryPoint() override {
      ROS_INFO_STREAM(" [ STATE_MACHINES | scout_state_machine | " << getName() << "] - entry point ]");
   }
   //UNDERSTANDING: Every state might HAVE its own exitpoint. 
   void exitPoint() override {
      ROS_INFO_STREAM(" [ STATE_MACHINES | scout_state_machine | " << getName() << "] - exit point ]");
   }

   void step() override {}

   State& transition() override {
      return getState(SCOUT_RESET_ODOM);   //Just put to fix build errors, remove all traces of transition from the state machines. 
   };


protected:
  // robot_state_status variables
  int robot_desired_state_;
  state_machines::robot_state_status status_;

  ros::ServiceClient spiralClient_;

  bool near_volatile_ = false;       //
  bool new_volatile_msg_ = false; 
 
  std::mutex objects_mutex_;
  perception::ObjectArray::ConstPtr vision_objects_;
  bool objects_msg_received_ = false;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionResult navigation_vision_result_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;

  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_ *resource_localiser_client_;

  typedef actionlib::SimpleActionClient<operations::ParkRobotAction> ParkRobotClient;
  ParkRobotClient *park_robot_client_;
  operations::ParkRobotGoal park_robot_goal_;
};

/**
 * @brief Undock move off volatile
 * 
 * @param isDone() navigation vision is done
 * @param hasSucceeded() navigation vision has succeeded
 * 
 */
class Undock : public ScoutState {
public:   
   Undock(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_UNDOCK, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private: 
   bool first_;
   operations::NavigationGoal navigation_action_goal_;
   operations::NavigationVisionGoal navigation_vision_goal_;
};

/**
 * @brief Seach attemps to seach for a volatile
 * 
 * @param isDone() when near a volatile
 * @param hasSucceeded() when near a volatile
 */
class Search : public ScoutState {
public:   
   Search(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_SEARCH_VOLATILE, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   operations::Spiral srv;

};

/**
 * @brief Locate move closer to the center of the volatile
 * 
 * @param isDone volatile is within range 
 * @param hasSucceeded scout is parked on top of volatile
 */
class Locate : public ScoutState {
public:   
   Locate(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_LOCATE_VOLATILE, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   operations::ResourceLocaliserGoal goal;
   bool first_;
};

/**
 * @brief ResetOdomAtHopper navigates to hopper, and then resets odom
 * 
 * @param isDone() when naviagtion vision is complete
 * @param hasSucceeded(); when navigation vision succeeds, and resetOdom msg was sent 
 * 
 */
class ResetOdomAtHopper : public ScoutState {
public:   
   ResetOdomAtHopper(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_RESET_ODOM, nh, robot_name) {}

   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   void goToProcPlant();
   void parkAtHopper();
   void undockFromHopper();
   void resetOdom();
   void goToRepair();
   void idleScout(){}

   bool first_GTPP, first_PAH, first_UFH, first_GTR, macro_state_succeeded, macro_state_done;
   
   enum RESET_ODOM_MICRO_STATES{
      GO_TO_PROC_PLANT,
      PARK_AT_HOPPER,
      UNDOCK_FROM_HOPPER,
      RESET_ODOM_AT_HOPPER,
      GO_TO_REPAIR_STATION,
      SCOUT_IDLE
   };

   bool state_done = false;
   RESET_ODOM_MICRO_STATES micro_state;
};

/**
 * @brief IdleState Robot should stop and do nothing
 * 
 * @param isDone() goals have been send to stop the robot
 * @param hasSucceded() goals sent have been successfull
 */
class IdleState : public ScoutState {
public:   
   IdleState(ros::NodeHandle nh, std::string robot_name) : ScoutState(ROBOT_IDLE_STATE, nh, robot_name) {}

   bool isDone() override{ 
      current_state_done_ = true;
      return true; }
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override{ 
      last_state_succeeded_ = true;
      return true; }

   void entryPoint() override;
   void step() override{}
   void exitPoint() override{}
};

/**
 * @brief GoToRepairStation naviagte to repair station
 * 
 * @param isDone() navigation vision is done getting to the repair station
 * @param hasSucceeded() navigation vision has succeeded at getting to the repair station
 */
class GoToRepairStation : public ScoutState {
public:   
   GoToRepairStation(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_GOTO_REPAIR_STATION, nh, robot_name) {}

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
 * @brief Sends the scout to the given goal
 * 
 * @param isDone() when near a volatile
 * @param hasSucceeded() when near a volatile
 */
class ScoutGoToLoc : public ScoutState {
public:   
   ScoutGoToLoc(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_SEARCH_VOLATILE, nh, robot_name) {}

   // define transition check conditions for the state (isDone() is overriden by each individual state)
   bool isDone() override;
   // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
   bool hasSucceeded() override;

   void entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   bool first_;
   geometry_msgs::PoseStamped target_loc_;
   operations::NavigationGoal navigation_action_goal_;
};

// class ParkAtRepairStation : public ScoutState {
// public:   
//    ParkAtRepairStation(ros::NodeHandle nh, std::string robot_name) : ScoutState(SCOUT_PARK_REPAIR_STATION, nh, robot_name) {}

//    // define transition check conditions for the state (isDone() is overriden by each individual state)
//    bool isDone() override;
//    // define if state succeeded in completing its action for the state (hasSucceeded is overriden by each individual state)
//    bool hasSucceeded() override;

//    void entryPoint() override;
//    void step() override;
//    void exitPoint() override;

// private:
//    bool first_;
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