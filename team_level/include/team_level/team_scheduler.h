#ifndef TEAM_SCHEDULER_H
#define TEAM_SCHEDULER_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/common_names.h>
#include "ros/ros.h"
#include <team_level/team.h>

using namespace COMMON_NAMES;

#define MAX_SCOUTS 2
#define MAX_EXCAVATORS 2
#define MAX_HAULERS 2
#define MAX_TEAMS 4

class TeamScheduler {
   
public:

   TeamScheduler(ros::NodeHandle nh);
   ~TeamScheduler();
   
   void step();

   void exec();

   bool done();
   
   void setInterrupt(STATE_MACHINE_TASK interrupt_state);
   
private:
   std::array<RobotStatus*, MAX_SCOUTS> scouts;
   std::array<RobotStatus*, MAX_EXCAVATORS> excavators;
   std::array<RobotStatus*, MAX_HAULERS> haulers;

   std::array<Team*, MAX_TEAMS> all_teams;

   void initTeams(ros::NodeHandle nh);

   void initTeamArray(ros::NodeHandle nh);
   void addRobots();
   void setSearchStates();
};

#endif // TEAM_SCHEDULER_H
