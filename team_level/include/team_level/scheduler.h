#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/common_names.h>
#include "ros/ros.h"

using namespace COMMON_NAMES;

// class MacroState;
class State;
class RobotScheduler;

/****************************************/
/****************************************/

class StateMachineException {
   
public:
   
   StateMachineException(const std::string& str_msg);
   std::string getMessage() const;

private:

   std::string m_strMsg;
};

/****************************************/
/****************************************/

// /**
//  * The macrostate is a state machine for team-level coordination.
//  *
//  * A state in the macrostate is a collection of individual robot states.
//  */
// class MacroState {
//   public:
// };

/****************************************/
/****************************************/
template<int SCOUTS, int EXCAVATORS, int HAULERS>
class TeamScheduler {
   
public:

   virtual ~TeamScheduler();

   virtual void addState(State* pc_state);

   State& getState(uint32_t un_id);

   void setInitialState(uint32_t un_state);
   
   virtual void step();

   virtual void exec();

   virtual bool done() = 0;
   
   virtual void setInterrupt(STATE_MACHINE_TASK interrupt_state) = 0;
   
private:

   State* m_pcCurrent;
   std::unordered_map<uint32_t, State*> m_mapStates;

   std::vector<*ExcavationTeam>

   // std::array instead of this

   RobotStatus scouts[SCOUTS];
   RobotStatus excavators[EXCAVATORS];
   RobotStatus haulers[HAULERS];

   std::vector<std::pair<bool,bool>> scout_vecPair_volAvl_recruitedTeam;
   std::vector<bool> excavator_idle;
   std::vector<bool> hauler_idle;

protected:
   bool m_bInterrupt = false;
   STATE_MACHINE_TASK interrupt_state_;
};


/****************************************/
/****************************************/

class ExcavationTeam{
   void setScout(RobotStatus& scout);
   void setExcavator(RobotStatus& excavator);
   void setHauler(RobotStatus& hauler);
   RobotStatus getExcavator();
   RobotStatus getHauler();
   RobotStatus getScout();
   bool shouldScoutDisband();
   void disbandScout();
   bool shouldTeamDisband();
   void disbandTeam();
   bool isExcavatorRecruited();
   bool isHaulerRecruited();

   void updateMacroState();
   void setMacroState();
   TeamState getMacroState();
}

/****************************************/
/****************************************/

class RobotStatus{
   public:
      Robot(std::string robot_name)
      int getId();
      // StateStatusEnum getStateStatus();
      bool hasFailed();
      bool isDone();
   private:
      ros::Subscriber robot_state_subscriber;
}

/****************************************/
/****************************************/

class TeamState {
   
public:

   TeamState(uint32_t un_id,
         const std::string& str_name) :
      m_pcRobotScheduler(nullptr),
      m_unId(un_id),
      m_strName(str_name) {}

   virtual ~TeamState() {}

   uint32_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual void entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual State& transition() = 0;

   State& getState(uint32_t un_state);

   void setRobotScheduler(RobotScheduler& c_robot_scheduler);
   
private:

   RobotScheduler* m_pcRobotScheduler;
   uint32_t m_unId;
   std::string m_strName;
};

// All the states as per the diagram
class TeamSearch: public TeamState{

}

class Recruite: public TeamState{

}

class ReachSite: public TeamState{
   
}

#endif // STATE_MACHINE_H
