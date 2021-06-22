#include <utils/common_names.h>
#include <team_level/robot_status.h>

enum TEAM_MACRO_STATE{
   STANDBY, // If No robot in the team;
   IDLE,    // If Robots in the team are idle
   SEARCH,  // Team has a busy scout
   SCOUT_WAITING, // Team has a scout waiting for Excavator to take over
   EXCAVATING,    // No or idle scout in the team
                  // Excavator and Hauler busy digging and collecting
   DUMPING, // Hauler going to dump the volatile
            // No or idle Excavator
};

class Team{
public:
   Team(ros::NodeHandle nh, TEAM_MACRO_STATE team_state, ROBOTS_ENUM scout, 
         ROBOTS_ENUM excavator, ROBOTS_ENUM hauler);
   ~Team();

   RobotStatus *robot_state;

   void setScout(ROBOTS_ENUM scout){ hired_scout = scout; }
   void setExcavator(ROBOTS_ENUM excavator){ hired_excavator = excavator; }
   void setHauler(ROBOTS_ENUM hauler){ hired_hauler = hauler; }
   
   void disbandScout(ROBOTS_ENUM scout){ hired_scout = NONE; }
   void disbandExcavator(ROBOTS_ENUM excavator){ hired_excavator = NONE; }
   void disbandHauler(ROBOTS_ENUM hauler){ hired_hauler = NONE; }
   
   ROBOTS_ENUM getScout() const { return hired_scout; }
   ROBOTS_ENUM getExcavator() const { return hired_excavator; }
   ROBOTS_ENUM getHauler() const { return hired_hauler; }
   
   bool isScoutHired(){ return !(hired_scout == NONE); }
   bool isExcavatorHired(){ return !(hired_excavator == NONE); }
   bool isHaulerHired(){ return !(hired_hauler == NONE); }
   
   TEAM_MACRO_STATE getTeamMacroState(){ return macro_state; }
   void setTeamMacroState(TEAM_MACRO_STATE team_state){ macro_state = team_state; }

private:
   void updateTeamMacroState();
   ROBOTS_ENUM hired_scout, hired_excavator, hired_hauler;
   TEAM_MACRO_STATE macro_state;
};