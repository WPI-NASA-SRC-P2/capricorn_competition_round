#pragma once
#include <utils/common_names.h>
#include "ros/ros.h"
#include <state_machines/set_robot_state.h>

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
   
   virtual TeamState& transition() = 0;

   TeamState& getState(uint32_t un_state);

   // void setRobotScheduler(RobotScheduler& c_robot_scheduler);
   
private:

   // // RobotScheduler* m_pcRobotScheduler;
   uint32_t m_unId;
   std::string m_strName;

   ros::ServiceClient scout_1_service_client;
   ros::ServiceClient scout_2_service_client;
   ros::ServiceClient scout_3_service_client;

   ros::ServiceClient excavator_1_service_client;
   ros::ServiceClient excavator_2_service_client;
   ros::ServiceClient excavator_3_service_client;

   ros::ServiceClient hauler_1_service_client;
   ros::ServiceClient hauler_2_service_client;
   ros::ServiceClient hauler_3_service_client;
};

// // All the states as per the diagram
class STANDBY: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class IDLE: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class SEARCH: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class SCOUT_WAITING: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class EXCAVATING: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};

class DUMPING: public TeamState{
   TeamState& transition() override ;
   
   void entryPoint() override;
   void step() override;
   void exitPoint() override;
};