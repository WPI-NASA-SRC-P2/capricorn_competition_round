#pragma once
#include <utils/common_names.h>
#include "ros/ros.h"
#include <state_machines/set_robot_state.h>
#include <team_level/robot_status.h>

// /**
//  * The macrostate is a state machine for team-level coordination.
//  *
//  * A state in the macrostate is a collection of individual robot states.
//  */
// class MacroState {
//   public:
// };

using namespace COMMON_NAMES;

class TeamState {
   
public:

   TeamState(uint32_t un_id, const std::string& str_name, ros::NodeHandle nh);

   virtual ~TeamState() {}

   uint32_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual void entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual bool isDone() = 0;

   TeamState& getState(uint32_t un_state);

   // void setRobotScheduler(RobotScheduler& c_robot_scheduler);
   
protected:

   // // RobotScheduler* m_pcRobotScheduler;
   uint32_t m_unId;
   std::string m_strName;
   ROBOTS_ENUM scout, excavator, hauler;
   RobotStatus *robot_status;

   ros::ServiceClient scout_1_service_client;
   ros::ServiceClient scout_2_service_client;
   ros::ServiceClient scout_3_service_client;

   ros::ServiceClient excavator_1_service_client;
   ros::ServiceClient excavator_2_service_client;
   ros::ServiceClient excavator_3_service_client;

   ros::ServiceClient hauler_1_service_client;
   ros::ServiceClient hauler_2_service_client;
   ros::ServiceClient hauler_3_service_client;

   std::map<ROBOTS_ENUM, ros::ServiceClient> ROBOT_ENUM_SERVICE_MAP{
                                 {SCOUT_1, scout_1_service_client},
                                 {SCOUT_2, scout_2_service_client},
                                 {SCOUT_3, scout_3_service_client},
                                 {EXCAVATOR_1, excavator_1_service_client},
                                 {EXCAVATOR_2, excavator_2_service_client},
                                 {EXCAVATOR_3, excavator_3_service_client},
                                 {HAULER_1, hauler_1_service_client},
                                 {HAULER_2, hauler_2_service_client},
                                 {HAULER_3, hauler_3_service_client}};

};

// // All the states as per the diagram
class STANDBY: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class IDLE: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class SEARCH: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class SCOUT_WAITING: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class EXCAVATING: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class DUMPING: public TeamState{
   bool isDone() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};