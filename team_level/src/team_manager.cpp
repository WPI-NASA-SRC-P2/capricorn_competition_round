#include <team_level/team_manager.h>

TeamManager::TeamManager(ros::NodeHandle nh)
{
   initTeams(nh);
}

void TeamManager::initTeams(ros::NodeHandle nh)
{
   initTeamArray(nh);
   addRobots();
   setSearchStates();
}

void TeamManager::initTeamArray(ros::NodeHandle nh)
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      all_teams.at(i) = new TeamScheduler(nh);
   }
}

void TeamManager::addRobots()
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

void TeamManager::setSearchStates()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if(all_teams.at(i)->isScoutHired())
         all_teams.at(i)->setTeamMacroState(SEARCH);
   }
}

void TeamManager::recruitment()
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

bool TeamManager::hasScout(int team_index)
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

bool TeamManager::hasExcavator(int team_index)
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

bool TeamManager::hasHauler(int team_index)
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

void TeamManager::fireScout(int team_index)
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

void TeamManager::fireExcavator(int team_index)
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

void TeamManager::fireHauler(int team_index)
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

void TeamManager::recruitScout(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(scout_for_sale.at(i))
      {
         ROBOTS_ENUM scout = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setScout(scout);
         all_teams.at(i)->disbandScout();

         scout_for_sale.at(i) = false;
         teams_need_scout.at(team_index) = false;
      }
   }
}

void TeamManager::recruitExcavator(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(excavator_for_sale.at(i))
      {
         ROBOTS_ENUM excavator = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setExcavator(excavator);
         all_teams.at(i)->disbandExcavator();

         excavator_for_sale.at(i) = false;
         teams_need_excavator.at(team_index) = false;
      }
   }
}

void TeamManager::recruitHauler(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(hauler_for_sale.at(i))
      {
         ROBOTS_ENUM hauler = all_teams.at(i)->getScout();
         all_teams.at(team_index)->setHauler(hauler);
         all_teams.at(i)->disbandHauler();

         hauler_for_sale.at(i) = false;
         teams_need_hauler.at(team_index) = false;
      }
   }
}


void TeamManager::checkAndRecruitForSearch(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
   
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamManager::checkAndRecruitForScoutWaiting(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
   if(!hasExcavator(team_index))
      recruitExcavator(team_index);
   if(!hasHauler(team_index))
      recruitHauler(team_index);
}

void TeamManager::checkAndRecruitForExcavating(int team_index)
{
   if(!hasExcavator(team_index))
      recruitExcavator(team_index);
   if(!hasHauler(team_index))
      recruitHauler(team_index);

   fireScout(team_index);
   // Recruit from array as well
}

void TeamManager::checkAndRecruitForDumping(int team_index)
{
   if(!hasHauler(team_index))
      recruitHauler(team_index);
   
   fireScout(team_index);
   fireExcavator(team_index);
   // Recruit from array as well
}

void TeamManager::checkAndRecruitForIdle(int team_index)
{
   fireScout(team_index);
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamManager::checkAndRecruitForStandby(int team_index)
{
   fireScout(team_index);
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamManager::step()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      all_teams.at(i)->step();
   }
}

void TeamManager::exec()
{
   while(ros::ok())
   {
      step();
      ros::spinOnce();
      ros::Duration(0.5).sleep();
   }
}