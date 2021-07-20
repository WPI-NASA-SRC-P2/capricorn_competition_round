#ifndef TEAM_SCHEDULER_H
#define TEAM_SCHEDULER_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/common_names.h>
#include "ros/ros.h"
#include <team_level/team_scheduler.h>

using namespace COMMON_NAMES;

#define MAX_SCOUTS 2
#define MAX_EXCAVATORS 2
#define MAX_HAULERS 2
#define MAX_TEAMS 8
#define MAX_ROBOTS 9

class TeamManager {
   
public:

   TeamManager(ros::NodeHandle nh);
   ~TeamManager();
   
   void step();

   void exec();

private:
   std::array<TeamScheduler*, MAX_TEAMS> all_teams;

   std::array<bool, MAX_TEAMS> teams_need_scout;
   std::array<bool, MAX_TEAMS> scout_for_sale;
   std::array<bool, MAX_TEAMS> teams_need_excavator;
   std::array<bool, MAX_TEAMS> excavator_for_sale;
   std::array<bool, MAX_TEAMS> teams_need_hauler;
   std::array<bool, MAX_TEAMS> hauler_for_sale;

   static const int resetting_hauler_index = 1;
   // LAST WEEK FIX!
   // Assumes that there are 2 robots of each kind
   bool both_excavators_working = true;
   bool both_scouts_working = true;
   bool both_haulers_working = true;

   std::array<bool, 10> robots_waiting_to_reset;
   bool hopper_busy;

   int getStandbyTeam();

   void initTeams(ros::NodeHandle nh);

   void deleteTeams();

   void initTeamArray(ros::NodeHandle nh);

   void addRobots();
   
   void setSearchStates();

   void setHaulerForReset();

   void setEmptyTeamsStandby();

   void recruitment();

   bool hasScout(int team_index);

   bool hasExcavator(int team_index);

   bool hasHauler(int team_index);

   void fireScout(int team_index);

   void fireExcavator(int team_index);

   void fireHauler(int team_index);

   void recruitScout(int team_index);

   void recruitExcavator(int team_index);

   void recruitHauler(int team_index);

   void transferRobotToStandbyTeam(ROBOTS_ENUM transfered_robot, TEAM_MACRO_STATE desired_state_of_team, int current_team_index);

   void checkAndRecruitForSearch(int team_index);

   void checkAndRecruitForScoutWaiting(int team_index);

   void checkAndRecruitForExcavating(int team_index);

   void checkAndRecruitForDumping(int team_index);

   void checkAndRecruitForIdle(int team_index);

   void checkAndRecruitForStandby(int team_index);

   void checkAndRecruitForGoToRepairStation(int team_index);
   
   void checkAndRecruitForWaitForHopperAppointment(int team_index);
   
   void checkAndRecruitForResetAtHopper(int team_index);

   void checkAndRecruitForGoToInitLoc(int team_index);
};

#endif // TEAM_SCHEDULER_H
