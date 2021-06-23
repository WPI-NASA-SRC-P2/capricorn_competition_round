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

class RobotScheduler {
   
public:

   virtual ~RobotScheduler();

   virtual void addState(State* pc_state);

   State& getState(uint32_t un_id);

   void setInitialState(uint32_t un_state);
   
   virtual void step();

   virtual void exec();

   virtual bool done() = 0;
   
   virtual void setState(STATE_MACHINE_TASK new_state) = 0;
   
private:

   State* m_pcCurrent;
   std::unordered_map<uint32_t, State*> m_mapStates;

protected:
   bool new_state_request = false;
   STATE_MACHINE_TASK new_state_;
};

/****************************************/
/****************************************/

class State {
   
public:

   State(uint32_t un_id,
         const std::string& str_name) :
      m_pcRobotScheduler(nullptr),
      m_unId(un_id),
      m_strName(str_name) {}

   virtual ~State() {}

   uint32_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual void entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual bool isDone() = 0;

   State& getState(uint32_t un_state);

   void setRobotScheduler(RobotScheduler& c_robot_scheduler);
   
private:

   RobotScheduler* m_pcRobotScheduler;
   uint32_t m_unId;
   std::string m_strName;
};


#endif // STATE_MACHINE_H
