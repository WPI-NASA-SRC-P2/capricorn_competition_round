#ifndef TEAM_SCHEDULER_H
#define TEAM_SCHEDULER_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/common_names.h>
#include "ros/ros.h"

using namespace COMMON_NAMES;

#define TOTAL_SCOUTS 2
#define TOTAL_EXAVATORS 2
#define TOTAL_HAULERS 2

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

   RobotStatus scouts[TOTAL_SCOUTS];
   RobotStatus excavators[TOTAL_EXAVATORS];
   RobotStatus haulers[TOTAL_HAULERS];

   std::vector<std::pair<bool,bool>> scout_vecPair_volAvl_recruitedTeam;
   std::vector<bool> excavator_idle;
   std::vector<bool> hauler_idle;

protected:
   bool m_bInterrupt = false;
   STATE_MACHINE_TASK new_state_;
};

#endif // TEAM_SCHEDULER_H
