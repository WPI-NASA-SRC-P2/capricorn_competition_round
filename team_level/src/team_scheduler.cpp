#include <team_level/team_scheduler.h>

TeamScheduler::TeamScheduler(ros::NodeHandle nh)
{
   initTeams(nh);
}

void TeamScheduler::initTeams(ros::NodeHandle nh)
{
   initTeamArray(nh);
   addRobots();
   setSearchStates();
}

void TeamScheduler::initTeamArray(ros::NodeHandle nh)
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      all_teams.at(i) = new Team(nh);
   }
}

void TeamScheduler::addRobots()
{
   for(int i = 0; i < MAX_SCOUTS; i++)
   {
      int robot_index = (int) SCOUT_1 + i;
      ROBOTS_ENUM robot = (ROBOTS_ENUM) robot_index;
      all_teams.at(i)->setScout(robot);
      all_teams.at(i)->setTeamMacroState(IDLE);
   }
   
   for(int i = 0; i < MAX_EXCAVATORS; i++)
   {
      int robot_index = (int) EXCAVATOR_1 + i;
      ROBOTS_ENUM robot = (ROBOTS_ENUM) robot_index;
      all_teams.at(i)->setExcavator(robot);
      all_teams.at(i)->setTeamMacroState(IDLE);
   }
   
   for(int i = 0; i < MAX_HAULERS; i++)
   {
      int robot_index = (int) HAULER_1 + i;
      ROBOTS_ENUM robot = (ROBOTS_ENUM) robot_index;
      all_teams.at(i)->setHauler(robot);
      all_teams.at(i)->setTeamMacroState(IDLE);
   }      
}

void TeamScheduler::setSearchStates()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if(all_teams.at(i)->isScoutHired())
         all_teams.at(i)->setTeamMacroState(SEARCH);
   }
}
