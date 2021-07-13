#pragma once
#include <utils/common_names.h>
#include "ros/ros.h"
#include <state_machines/set_robot_state.h>
#include <team_level/robot_state_register.h>
#include <team_level/robot_pose_register.h>

// #include <team_level/team_scheduler.h>

// /**
//  * The macrostate is a state machine for team-level coordination.
//  *
//  * A state in the macrostate is a collection of individual robot states.
//  */
// class MacroState {
//   public:
// };

enum TEAM_MACRO_STATE{
   STANDBY, // If No robot in the team;
   IDLE,    // If Robots in the team are idle
   SEARCH,  // Team has a busy scout
   SCOUT_WAITING, // Team has a scout waiting for Excavator to take over
   EXCAVATING,    // No or idle scout in the team
                  // Excavator and Hauler busy digging and collecting
   DUMPING, // Hauler going to dump the volatile
            // No or idle Excavator
   GO_TO_REPAIR_STATION,   // Will go to Repair Station 
   WAIT_FOR_HOPPER_APPOINTMENT,  // Waits for an appointment at hopper. 
   RESET_AT_HOPPER,     // Only one robot can be in this state at once. 
};

enum TEAM_MICRO_STATE{
   // SEARCH
   SEARCH_FOR_VOLATILE,
   RESET_ODOMETRY_AT_HOPPER,
   GO_TO_MAP_FRAME,

   // SCOUT_WAITING
   ROBOTS_TO_GOAL,
   UNDOCK_SCOUT,
   PARK_EXCAVATOR_AT_SCOUT,
   MAKE_EXCAV_HAULER_IDLE,

   // EXCAVATING
   WAIT_FOR_HAULER,
   PRE_PARK_MANEUVER_EXCAVATOR,
   PARK_AT_EXCAVATOR_HAULER,
   DIG_AND_DUMP,
   UNDOCK_HAULER,

   // DUMPING
   DUMP_COLLECTION,

   // IDLE
   IDLE_MICRO_STATE
};

using namespace COMMON_NAMES;

class TeamScheduler;

class TeamState {
   
public:

   TeamState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh);

   ~TeamState() 
   {
      delete robot_state_register;
      delete robot_pose_register;
   }

   uint64_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual bool entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual bool isDone() = 0;

   virtual TeamState& transition() = 0;

   TeamState& getState(uint64_t un_state);

   void setTeam(TeamScheduler& c_robot_scheduler);
   
   virtual TEAM_MICRO_STATE getMicroState() = 0;

   void updateRobots(ROBOTS_ENUM scout = NONE, ROBOTS_ENUM excavator = NONE, ROBOTS_ENUM hauler = NONE)
   {
      scout_in_team = scout;
      excavator_in_team = excavator;
      hauler_in_team = hauler;
   }


   void setResetRobot(bool reset_needed)
   {
      // ROS_WARN_STREAM(reset_needed);
      reset_robot_odometry = reset_needed;
      // ROS_WARN_STREAM(reset_robot_odometry);
   }

protected:

   TeamScheduler* m_pcTeam;
   uint64_t m_unId;
   std::string m_strName;
   ROBOTS_ENUM scout_in_team, excavator_in_team, hauler_in_team;
   RobotStateRegister *robot_state_register;
   RobotPoseRegister *robot_pose_register;
   geometry_msgs::PoseStamped volatile_site_location;
   bool reset_robot_odometry = false;
};

// // All the states as per the diagram
class Standby: public TeamState{
public:
   Standby(ros::NodeHandle &nh):TeamState(STANDBY, "Standby", nh){}
   bool isDone() override ;

   TeamState& transition() override {return *this;}
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class Idle: public TeamState{
public:
   Idle(ros::NodeHandle &nh):TeamState(IDLE, "Idle", nh){}
   bool isDone() override ;
   
   TeamState& transition() override {return *this;}
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class Search: public TeamState{
public:
   Search(ros::NodeHandle &nh):TeamState(SEARCH, "Search", nh){}
   bool isDone() override ;

   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState();
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   TEAM_MICRO_STATE micro_state;
   bool reset_once;
};

class ScoutWaiting: public TeamState{
public:
   ScoutWaiting(ros::NodeHandle &nh):TeamState(SCOUT_WAITING, "ScoutWaiting", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState();
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   TEAM_MICRO_STATE micro_state;
   void stepRobotsToGoal();
   void stepUndockScout();
   void stepParkExcavatorAtScout();
   void stepMakeExcavHaulerIdle();
};

class Excavating: public TeamState{
public:
   Excavating(ros::NodeHandle &nh):TeamState(EXCAVATING, "Excavating", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState();
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;

private:
   TEAM_MICRO_STATE micro_state;
   void stepWaitForHauler();
   void stepPreParkManeuverExcavator();
   void stepParkHauler();
   void stepDigAndDump();
   void stepUndockHauler();
};

class Dumping: public TeamState{
public:
   Dumping(ros::NodeHandle &nh):TeamState(DUMPING, "Dumping", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class GoToRepairStation: public TeamState{
public:
   GoToRepairStation(ros::NodeHandle &nh):TeamState(GO_TO_REPAIR_STATION, "GoToRepairStation", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class WaitForHopperAppointment: public TeamState{
public:
   WaitForHopperAppointment(ros::NodeHandle &nh):TeamState(WAIT_FOR_HOPPER_APPOINTMENT, "WaitForHopperAppointment", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class ResetAtHopper: public TeamState{
public:
   ResetAtHopper(ros::NodeHandle &nh):TeamState(RESET_AT_HOPPER, "ResetAtHopper", nh){}
   bool isDone() override ;
   
   TeamState& transition() override;
   TEAM_MICRO_STATE getMicroState(){return IDLE_MICRO_STATE;}
   
   bool entryPoint() override;
   void step() override;
   void exitPoint() override;
};
