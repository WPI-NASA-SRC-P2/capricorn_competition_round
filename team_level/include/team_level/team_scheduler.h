#pragma once

#include <utils/common_names.h>
#include <team_level/robot_state_register.h>
#include <team_level/team_state.h>
#include <unordered_map>

class TeamScheduler{
public:
   TeamScheduler(ros::NodeHandle &nh);
   ~TeamScheduler();

   void setScout(ROBOTS_ENUM scout){ hired_scout = scout; }
   void setExcavator(ROBOTS_ENUM excavator){ hired_excavator = excavator; }
   void setHauler(ROBOTS_ENUM hauler){ hired_hauler = hauler; }
   
   void disbandScout(){ hired_scout = NONE; }
   void disbandExcavator(){ hired_excavator = NONE; }
   void disbandHauler(){ hired_hauler = NONE; }
   
   ROBOTS_ENUM getScout() const { return hired_scout; }
   ROBOTS_ENUM getExcavator() const { return hired_excavator; }
   ROBOTS_ENUM getHauler() const { return hired_hauler; }
   
   bool isScoutHired(){ return !(hired_scout == NONE); }
   bool isExcavatorHired(){ return !(hired_excavator == NONE); }
   bool isHaulerHired(){ return !(hired_hauler == NONE); }
   
   TEAM_MACRO_STATE getTeamMacroState(){ return macro_state; }
   void setTeamMacroState(TEAM_MACRO_STATE team_state){ 
         macro_state = team_state; 
         new_state_request = true;}

   void exec();
   void step();

   TeamState& getState(uint32_t un_id);

private:
   void updateTeamMacroState();
   ROBOTS_ENUM hired_scout = NONE, hired_excavator = NONE, hired_hauler = NONE;
   TEAM_MACRO_STATE macro_state;

   void addStates(ros::NodeHandle &nh);
   void addState(TeamState* macro_state);

   void setInitialState(uint32_t un_state);

   bool new_state_request;

   TeamState* current_state_ptr;
   std::unordered_map<uint32_t, TeamState*> MACRO_STATE_PTR_MAP;
};
