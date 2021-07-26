#include <team_level/team_manager.h>

TeamManager::TeamManager(ros::NodeHandle nh)
{
   initTeams(nh);
   robot_state_register = new RobotStateRegister(nh);
}

TeamManager::~TeamManager()
{
   deleteTeams();
   delete robot_state_register;
}

void TeamManager::deleteTeams()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      delete all_teams.at(i);
   }
}

void TeamManager::initTeams(ros::NodeHandle nh)
{
   initTeamArray(nh);
   addRobots();
   setSearchStates();
   setEmptyTeamsStandby();
   setHaulerForReset();
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
      int standby_team = getStandbyTeam();
      all_teams.at(standby_team)->setExcavator(robot);
      all_teams.at(standby_team)->setTeamMacroState(GO_TO_INIT_LOC);
   }
   
   for(int i = 0; i < MAX_HAULERS; i++)
   {
      if (i == resetting_hauler_index)
         continue;
      int robot_index = (int) HAULER_1 + i;
      ROBOTS_ENUM robot = (ROBOTS_ENUM) robot_index;
      int standby_team = getStandbyTeam();
      all_teams.at(standby_team)->setHauler(robot);
      all_teams.at(standby_team)->setTeamMacroState(GO_TO_INIT_LOC);
   }      
}

void TeamManager::setSearchStates()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if(all_teams.at(i)->isScoutHired())
      {
         all_teams.at(i)->setTeamMacroState(SEARCH);
         all_teams.at(i)->setResetRobot(false);
      }
   }
}

void TeamManager::setHaulerForReset()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if(all_teams.at(i)->getTeamMacroState() == STANDBY)
      {      
         int robot_index = (int) HAULER_1 + resetting_hauler_index;
         ROBOTS_ENUM robot = (ROBOTS_ENUM) robot_index;
         all_teams.at(i)->setHauler(robot);
         all_teams.at(i)->setTeamMacroState(DUMPING);
         return;
      }
   }
}

void TeamManager::setEmptyTeamsStandby()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      bool scout_in_team = all_teams.at(i)->isScoutHired();
      bool excavator_in_team = all_teams.at(i)->isExcavatorHired();
      bool hauler_in_team = all_teams.at(i)->isHaulerHired();

      if(!scout_in_team && !excavator_in_team && !hauler_in_team)
      {      
         all_teams.at(i)->setTeamMacroState(STANDBY);
      }
   }
}

void TeamManager::recruitment()
{
   hopper_busy = false;
   setEmptyTeamsStandby();
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if (DEBUG)
         ROS_INFO_STREAM("[TEAM_LEVEL | team_manager.cpp ]: " << all_teams.at(i)->getScout()<<"  "<<all_teams.at(i)->getExcavator()<<"  "<<all_teams.at(i)->getHauler());

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
      case GO_TO_REPAIR_STATION:
         checkAndRecruitForGoToRepairStation(i);
         break;
      case WAIT_FOR_HOPPER_APPOINTMENT:
         checkAndRecruitForWaitForHopperAppointment(i);
         break;
      case RESET_AT_HOPPER:
         checkAndRecruitForResetAtHopper(i);
         break;
      case GO_TO_INIT_LOC:
         checkAndRecruitForGoToInitLoc(i);
         break;
      default:
         break;
      }
   }

   if(!hopper_busy)
      for(int i = 0; i < MAX_TEAMS; i++)
         if(all_teams.at(i)->getTeamMacroState() == WAIT_FOR_HOPPER_APPOINTMENT)
         {
            all_teams.at(i)->setTeamMacroState(RESET_AT_HOPPER);
            break;
         }
}

bool TeamManager::hasScout(int team_index)
{
   scout_for_sale.at(team_index) = false;
   if(all_teams.at(team_index)->isScoutHired())
   {
      teams_need_scout.at(team_index) = false;
      return true;
   }
   else
   {
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Team %i needs Scout", team_index);
      teams_need_scout.at(team_index) = true;
      return false;
   }
}

bool TeamManager::hasExcavator(int team_index)
{
   excavator_for_sale.at(team_index) = false;
   if(all_teams.at(team_index)->isExcavatorHired())
   {
      teams_need_excavator.at(team_index) = false;
      return true;
   }
   else
   {
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Team %i needs Excavator", team_index);
      teams_need_excavator.at(team_index) = true;
      return false;
   }
}

bool TeamManager::hasHauler(int team_index)
{
   hauler_for_sale.at(team_index) = false;
   if(all_teams.at(team_index)->isHaulerHired())
   {
      teams_need_hauler.at(team_index) = false;
      return true;
   }
   else
   {
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Team %i needs Hauler", team_index);
      teams_need_hauler.at(team_index) = true;
      return false;
   }
}

void TeamManager::fireScout(int team_index)
{
   if(all_teams.at(team_index)->isScoutHired())
   {
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Scout for sale in team %i", team_index);
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
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Excavator for sale in team %i", team_index);
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
      if(DEBUG)
         ROS_INFO_THROTTLE(3, "[TEAM_LEVEL | team_manager.cpp ]: Hauler for sale in team %i", team_index);
      hauler_for_sale.at(team_index) = true;
   }
   else
   {
      hauler_for_sale.at(team_index) = false;
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

         all_teams.at(team_index)->setTeamMacroState(SEARCH);
         all_teams.at(team_index)->setResetRobot(true);

         scout_for_sale.at(i) = false;
         teams_need_scout.at(team_index) = false;
         break;
      }
   }
}

void TeamManager::recruitExcavator(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(excavator_for_sale.at(i))
      {
         ROBOTS_ENUM excavator = all_teams.at(i)->getExcavator();

         // LAST WEEK FIX:
         // Assumptions: Two robots of each kind
         //              This state is only effectively called in SCOUT_WAITING

         if(both_excavators_working && both_scouts_working)
         {
            if(all_teams.at(team_index)->getScout() == SCOUT_1 && excavator == EXCAVATOR_1)
            {
               if (DEBUG)
                  ROS_INFO_STREAM("SCOUT_1 HIRED "<<excavator);
               all_teams.at(team_index)->setExcavator(excavator);
               all_teams.at(i)->disbandExcavator();

               excavator_for_sale.at(i) = false;
               teams_need_excavator.at(team_index) = false;
               break;
            }
            else if(all_teams.at(team_index)->getScout() == SCOUT_2 && excavator == EXCAVATOR_2)
            {
               if (DEBUG)
                  ROS_INFO_STREAM("SCOUT_1 HIRED "<<excavator);
               all_teams.at(team_index)->setExcavator(excavator);
               all_teams.at(i)->disbandExcavator();

               excavator_for_sale.at(i) = false;
               teams_need_excavator.at(team_index) = false;
               break;
            }
         }
      }
   }   
}

void TeamManager::recruitHauler(int team_index)
{
   for (int i = 0; i < MAX_TEAMS; i++)
   {
      if(hauler_for_sale.at(i))
      {
         ROBOTS_ENUM hauler = all_teams.at(i)->getHauler();

         // LAST WEEK FIX:
         // Assumptios: Two robots of each kind
         //              This state is only effectively called in EXCAVATING
         if(both_excavators_working && both_haulers_working)
         {
            if(all_teams.at(team_index)->getExcavator() == EXCAVATOR_1 && hauler == HAULER_1)
            {
               if (DEBUG)
                  ROS_INFO_STREAM("EXCAVATOR_1 HIRED "<<hauler);
               all_teams.at(team_index)->setHauler(hauler);
               all_teams.at(i)->disbandHauler();

               hauler_for_sale.at(i) = false;
               teams_need_hauler.at(team_index) = false;
               break;
            }
            else if(all_teams.at(team_index)->getExcavator() == EXCAVATOR_2 && hauler == HAULER_2)
            {
               if (DEBUG)
                  ROS_INFO_STREAM("EXCAVATOR_2 HIRED "<<hauler);
               all_teams.at(team_index)->setHauler(hauler);
               all_teams.at(i)->disbandHauler();

               hauler_for_sale.at(i) = false;
               teams_need_hauler.at(team_index) = false;
               break;
            }
         }
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
   // LAST WEEK FIX!
   // Sending hauler only after the excavator has found the scout

   // if(!hasHauler(team_index))
   //    recruitHauler(team_index);
}

void TeamManager::checkAndRecruitForExcavating(int team_index)
{
   if(!hasExcavator(team_index))
      recruitExcavator(team_index);
   if(!hasHauler(team_index))
      recruitHauler(team_index);

   fireScout(team_index);
}

void TeamManager::checkAndRecruitForDumping(int team_index)
{
   if(!hasHauler(team_index))
      recruitHauler(team_index);
   
   fireScout(team_index);
   fireExcavator(team_index);
}

void TeamManager::transferRobotToStandbyTeam(ROBOTS_ENUM transfered_robot, TEAM_MACRO_STATE desired_state_of_team, int current_team_index)
{
   int standby_team = getStandbyTeam();
   all_teams.at(standby_team)->setAnyRobot(transfered_robot);
   all_teams.at(current_team_index)->disbandAnyRobot(transfered_robot);

   all_teams.at(standby_team)->setTeamMacroState(desired_state_of_team);
}

void TeamManager::checkAndRecruitForGoToRepairStation(int team_index)
{
   if(all_teams.at(team_index)->isScoutHired())
   {
      if(all_teams.at(team_index)->isExcavatorHired())
      {
         ROBOTS_ENUM excavator = all_teams.at(team_index)->getExcavator();
         transferRobotToStandbyTeam(excavator, IDLE, team_index);
      }
      if(all_teams.at(team_index)->isHaulerHired())
      {
         ROBOTS_ENUM hauler = all_teams.at(team_index)->getHauler();
         transferRobotToStandbyTeam(hauler, IDLE, team_index);
      }
   }
   else if(all_teams.at(team_index)->isExcavatorHired() && all_teams.at(team_index)->isHaulerHired())
   {
      ROBOTS_ENUM excavator = all_teams.at(team_index)->getExcavator();
      transferRobotToStandbyTeam(excavator, GO_TO_REPAIR_STATION, team_index);
   }

   fireExcavator(team_index);
}

void TeamManager::checkAndRecruitForWaitForHopperAppointment(int team_index)
{
   fireExcavator(team_index);
}

void TeamManager::checkAndRecruitForResetAtHopper(int team_index)
{
   if(all_teams.at(team_index)->isScoutHired())
   {
      bool scout_dead = robot_state_register->isRobotOutOfCommission(all_teams.at(team_index)->getScout());
      if(scout_dead)
      {
         // all_teams.at(team_index)->disbandScout();
         hopper_busy = false;
         return;
      }
   }
   else if(all_teams.at(team_index)->isExcavatorHired())
   {
      bool excavator_dead = robot_state_register->isRobotOutOfCommission(all_teams.at(team_index)->getExcavator());
      if(excavator_dead)
      {
         // all_teams.at(team_index)->disbandExcavator();
         hopper_busy = false;
         return;
      }
   }
   else if(all_teams.at(team_index)->isHaulerHired())
   {
      bool hauler_dead = robot_state_register->isRobotOutOfCommission(all_teams.at(team_index)->getHauler());
      if(hauler_dead)
      {
         // all_teams.at(team_index)->disbandHauler();
         hopper_busy = false;
         return;
      }
   }
   else
   {
      ROS_WARN_STREAM("[TEAM_LEVEL | team_manager.cpp ]: NONE robot was set for the hopper reset");
      hopper_busy = false;
   }
   hopper_busy = true;
}

void TeamManager::checkAndRecruitForGoToInitLoc(int team_index)
{
   // Should not have the scout in the first place
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamManager::checkAndRecruitForIdle(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
   else
      all_teams.at(team_index)->setTeamMacroState(SEARCH);

  
   fireExcavator(team_index);
   fireHauler(team_index);
}

void TeamManager::checkAndRecruitForStandby(int team_index)
{
   if(!hasScout(team_index))
      recruitScout(team_index);
      
   fireExcavator(team_index);
   fireHauler(team_index);
}

int TeamManager::getStandbyTeam()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if(all_teams.at(i)->getTeamMacroState() == STANDBY)
         return i;
   }
}

void TeamManager::step()
{
   for(int i = 0; i < MAX_TEAMS; i++)
   {
      if (DEBUG)
         ROS_WARN_STREAM("Team "<<i<<" Task:"<<all_teams.at(i)->getTeamMacroState());
      all_teams.at(i)->step();
   }
}

void TeamManager::exec()
{
   while(ros::ok())
   {
      recruitment();
      step();
      ros::spinOnce();
      if (DEBUG)
         ros::Duration(0.5).sleep();
   }
}