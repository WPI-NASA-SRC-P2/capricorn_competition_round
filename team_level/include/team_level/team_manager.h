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

#define MAX_SCOUTS 1
#define MAX_EXCAVATORS 1
#define MAX_HAULERS 1
#define MAX_TEAMS 4

class TeamManager {
   
public:

   TeamManager(ros::NodeHandle nh);
   ~TeamManager(){}
   
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

   void initTeams(ros::NodeHandle nh);

   void initTeamArray(ros::NodeHandle nh);

   void addRobots();
   
   void setSearchStates();

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

   void checkAndRecruitForSearch(int team_index);

   void checkAndRecruitForScoutWaiting(int team_index);

   void checkAndRecruitForExcavating(int team_index);

   void checkAndRecruitForDumping(int team_index);

   void checkAndRecruitForIdle(int team_index);

   void checkAndRecruitForStandby(int team_index);
};

#endif // TEAM_SCHEDULER_H
