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

void TeamScheduler::recruitment()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      
      switch (all_teams.at(i)->getTeamMacroState())
      {
      case STANDBY:
         checkAndRecruitForStandby(i);
         break;
      case IDLE:
         checkAndRecruitForIdle(i);
         break;
      case SEARCH:
         checkAndRecruitForSearch(i);
         break;
      case SCOUT_WAITING:
         checkAndRecruitForScoutWaiting(i);
         break;
      case EXCAVATING:
         checkAndRecruitForExcavating(i);
         break;
      case DUMPING:
         checkAndRecruitForDumping(i);
         break;
      default:
         break;
      }
   }
}

bool TeamScheduler::hasScout(int team_index)
{
   if(all_teams.at(team_index)->isScoutHired())
   {
      teams_need_scout.at(team_index) = false;
      return true;
   }
   else
   {
      ROS_INFO_THROTTLE(1, "Team %i needs Scout", team_index);
      teams_need_scout.at(team_index) = true;
      return false;
   }
}

bool TeamScheduler::hasExcavator(int team_index)
{
   if(all_teams.at(team_index)->isExcavatorHired())
   {
      teams_need_excavator.at(team_index) = false;
      return true;
   }
   else
   {
      ROS_INFO_THROTTLE(1, "Team %i needs Excavator", team_index);
      teams_need_excavator.at(team_index) = true;
      return false;
   }
}

bool TeamScheduler::hasHauler(int team_index)
{
   if(all_teams.at(team_index)->isHaulerHired())
   {
      teams_need_hauler.at(team_index) = false;
      return true;
   }
   else
   {
      ROS_INFO_THROTTLE(1, "Team %i needs Hauler", team_index);
      teams_need_hauler.at(team_index) = true;
      return false;
   }
}

void TeamScheduler::fireScout(int team_index)
{
   if(all_teams.at(team_index)->isScoutHired())
   {
      ROS_INFO_THROTTLE(1, "Scout for sale in team %i", team_index);
      scout_for_sale.at(team_index) = true;
   }
   else
   {
      scout_for_sale.at(team_index) = false;
   }
}

void TeamScheduler::fireExcavator(int team_index)
{
   if(all_teams.at(team_index)->isExcavatorHired())
   {
      ROS_INFO_THROTTLE(1, "Excavator for sale in team %i", team_index);
      excavator_for_sale.at(team_index) = true;
   }
   else
   {
      excavator_for_sale.at(team_index) = false;
   }
}

void TeamScheduler::fireHauler(int team_index)
{
   if(all_teams.at(team_index)->isHaulerHired())
   {
      ROS_INFO_THROTTLE(1, "Hauler for sale in team %i", team_index);
      hauler_for_sale.at(team_index) = true;
   }
   else
   {
      hauler_for_sale.at(team_index) = true;
   }
}

void TeamScheduler::recruitScout(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(scout_for_sale.at(i))
      {
         ROBOTS_ENUM scout = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setScout(scout);

         scout_for_sale.at(i) = false;
         teams_need_scout.at(team_index) = false;
      }
   }
}

void TeamScheduler::recruitExcavator(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(excavator_for_sale.at(i))
      {
         ROBOTS_ENUM excavator = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setExcavator(excavator);

         excavator_for_sale.at(i) = false;
         teams_need_excavator.at(team_index) = false;
      }
   }
}

void TeamScheduler::recruitHauler(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(hauler_for_sale.at(i))
      {
         ROBOTS_ENUM hauler = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setHauler(hauler);

         hauler_for_sale.at(i) = false;
         teams_need_hauler.at(team_index) = false;
      }
   }
}


void TeamScheduler::checkAndRecruitForSearch(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
   
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamScheduler::checkAndRecruitForScoutWaiting(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
   if(!hasExcavator(team_index))
      recruitExcavator(team_index);
   if(!hasHauler(team_index))
      recruitHauler(team_index);
}

void TeamScheduler::checkAndRecruitForExcavating(int team_index)
{
   if(!hasExcavator(team_index))
      recruitExcavator(team_index);
   if(!hasHauler(team_index))
      recruitHauler(team_index);

   fireScout(team_index);
   // Recruit from array as well
}

void TeamScheduler::checkAndRecruitForDumping(int team_index)
{
   if(!hasHauler(team_index))
      recruitHauler(team_index);
   
   fireScout(team_index);
   fireExcavator(team_index);
   // Recruit from array as well
}

void TeamScheduler::checkAndRecruitForIdle(int team_index)
{
   fireScout(team_index);
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamScheduler::checkAndRecruitForStandby(int team_index)
{
   fireScout(team_index);
   fireExcavator(team_index);
   fireHauler(team_index);
}
