#include <team_level/team_state.h>


TeamState::TeamState(uint64_t un_id, const std::string& str_name, ros::NodeHandle &nh) :
      m_pcTeam(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
    robot_state_register = new RobotStateRegister(nh);
    robot_pose_register = new RobotPoseRegister(nh);
    volatile_register = new DetectedVolatileRegister(nh);
}

void TeamState::setTeam(TeamScheduler& c_robot_scheduler) {
   m_pcTeam = &c_robot_scheduler;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S T A N D B Y   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Standby::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of Standby");
   if(!(scout_in_team == NONE && excavator_in_team == NONE && hauler_in_team == NONE))
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | STANDBY STATE CALLED, BUT AT LEAST ONE ROBOT IS NOT UNSET");
      return false;
   }
   return true;
}

bool Standby::isDone()
{
   return true;
}

void Standby::step()
{
   // Do Nothing
}

void Standby::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of STANDBY, cancelling STANDBY goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Idle::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of Idle");
   if(scout_in_team == NONE && excavator_in_team == NONE && hauler_in_team == NONE)
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | IDLE STATE CALLED, BUT NO ROBOT IS SET");
      return false;
   }
   return true;
}

bool Idle::isDone()
{
   return true;
}

void Idle::step()
{
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

void Idle::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of IDLE, cancelling IDLE goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Search::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of Search");
   if(scout_in_team == NONE)
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | SCOUT IS UNSET, BUT STILL ENTRY POINT HAS BEEN CALLED!");
      return false;
   }

   micro_state = reset_robot_odometry ? RESET_ODOMETRY_AT_HOPPER : SEARCH_FOR_VOLATILE;
   reset_once = true;
   // ROS_INFO_STREAM("Reset odom "<<reset_robot_odometry<<" micro_state:"<<micro_state);
   return true;
}

bool Search::isDone()
{
   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team) && robot_state_register->hasSucceeded(scout_in_team);
   bool is_new_volatile = volatile_register->isNewVolatile(scout_in_team, robot_pose_register->getRobotPose(scout_in_team));
   return ((scout_task == SCOUT_SEARCH_VOLATILE && scout_done_and_succeeded) && is_new_volatile);
}

TEAM_MICRO_STATE Search::getMicroState()
{
   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team) && robot_state_register->hasSucceeded(scout_in_team);
   bool done_reset = scout_task == SCOUT_RESET_ODOM && scout_done_and_succeeded;
   if(!done_reset && reset_robot_odometry && reset_once)
      return  RESET_ODOMETRY_AT_HOPPER;
   else 
   {
      reset_once = false;
      return SEARCH_FOR_VOLATILE;
   }
}

TeamState& Search::transition()
{
   micro_state = getMicroState();

   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   bool scout_done_and_failed = robot_state_register->isDone(scout_in_team) && !robot_state_register->hasSucceeded(scout_in_team);

   bool need_reset = scout_task == SCOUT_SEARCH_VOLATILE && scout_done_and_failed;

   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | volatile detected, transitioning to scout_undock state");
      return getState(SCOUT_WAITING);
   }
   else if(need_reset)
   {
      ROS_INFO("TEAM_LEVEL | team_state | Volatile not found for long, going back to reset odom");
      return getState(GO_TO_REPAIR_STATION);
   }
   else
      return *this;
}
   
void Search::step()
{
   // robot_state_register->setRobotState(scout_in_team, SCOUT_SEARCH_VOLATILE);      
   // ROS_INFO_STREAM("micro_state:"<<micro_state);
   switch (micro_state)
   {
   case RESET_ODOMETRY_AT_HOPPER:
      robot_state_register->setRobotState(scout_in_team, SCOUT_RESET_ODOM);      
      break;
   case SEARCH_FOR_VOLATILE:
      robot_state_register->setRobotState(scout_in_team, SCOUT_SEARCH_VOLATILE);      
      break;
   default:
      break;
   }
}

void Search::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of SEARCH, cancelling SEARCH goal");
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T _ W A I T I N G   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ScoutWaiting::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of ScoutWaiting");

   if(scout_in_team == NONE || excavator_in_team == NONE)
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | AT LEAST ONE ROBOT IS UNSET FOR SCOUT WAITING!");
      return false;
   }

   micro_state = ROBOTS_TO_GOAL;

   volatile_site_location = robot_pose_register->getRobotPose(scout_in_team);
   volatile_register->registerNewVolatile(scout_in_team, volatile_site_location);
   return true;
}

bool ScoutWaiting::isDone()
{
   STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
   bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team) && robot_state_register->hasSucceeded(excavator_in_team);

   return excavator_task == EXCAVATOR_PARK_AND_PUB && excavator_done_and_succeeded;
}

TEAM_MICRO_STATE ScoutWaiting::getMicroState()
{
   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);

   bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team) && robot_state_register->hasSucceeded(scout_in_team);
   bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team) && robot_state_register->hasSucceeded(excavator_in_team);
   bool excavator_done_and_failed = robot_state_register->isDone(excavator_in_team) && !robot_state_register->hasSucceeded(excavator_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team) && robot_state_register->hasSucceeded(hauler_in_team);

   if (scout_task == ROBOT_IDLE_STATE || scout_task == SCOUT_SEARCH_VOLATILE )
   {
      if(scout_task != SCOUT_UNDOCK)
         return ROBOTS_TO_GOAL;
   }
   if (excavator_task == EXCAVATOR_GO_TO_SCOUT)
   {
      if(excavator_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ] Excavator Succeeded to reach Scout");
         if (scout_task == SCOUT_VISUAL_RESET_ODOM && robot_state_register->isDone(scout_in_team))
            return UNDOCK_SCOUT;
         if (scout_task == SCOUT_LOCATE_VOLATILE && robot_state_register->isDone(scout_in_team))
            return UNDOCK_SCOUT;
      }
      else if (excavator_done_and_failed)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ] Excavator Failed to reach Scout");
         return RESET_SCOUT_EXCAV_ODOM;
      }
   }
   if (excavator_task == EXCAVATOR_GO_TO_SCOUT_RECOVERY)
   {
      if(excavator_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ] Excavator Succeeded to reach Scout");
         if (scout_task == SCOUT_VISUAL_RESET_ODOM && robot_state_register->isDone(scout_in_team))
            return UNDOCK_SCOUT;
         if (scout_task == SCOUT_LOCATE_VOLATILE && robot_state_register->isDone(scout_in_team))
            return UNDOCK_SCOUT;
      }
      else if (excavator_done_and_failed)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ] Excavator Failed to reach Scout");
         return MAKE_EXCAV_HAULER_IDLE;
      }
   }
   if (scout_task == SCOUT_UNDOCK && scout_done_and_succeeded)
      return PARK_EXCAVATOR_AT_SCOUT;
   if (scout_task == SCOUT_VISUAL_RESET_ODOM && robot_state_register->isDone(scout_in_team))
   {
      if(excavator_task == EXCAVATOR_VISUAL_RESET_ODOM && robot_state_register->isDone(excavator_in_team))
      {
         return RECOVERY_SCOUT_FINDING;
      }
   }
   if((scout_task == SCOUT_VISUAL_RESET_ODOM && !scout_done_and_succeeded) || 
      (excavator_task == EXCAVATOR_VISUAL_RESET_ODOM && !excavator_done_and_succeeded))
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  6");
         return RESET_SCOUT_EXCAV_ODOM;
      }
   if (scout_task == SCOUT_LOCATE_VOLATILE)
   {
      bool excav_in_process = excavator_task == EXCAVATOR_GO_TO_SCOUT;
      bool excav_absent = excavator_task == ROBOT_IDLE_STATE;

      bool hauler_in_process = hauler_task == HAULER_GO_BACK_TO_EXCAVATOR;
      bool hauler_absent = hauler_task == ROBOT_IDLE_STATE;
      
      if(!(excav_in_process || excav_absent))
         ROS_WARN_STREAM("TEAM_LEVEL | team_state | Excavator state anomaly!"<<excavator_task);
      if(!(hauler_in_process || hauler_absent))
         ROS_WARN_STREAM("TEAM_LEVEL | team_state | Hauler state anomaly!"<<hauler_task);
      return ROBOTS_TO_GOAL;
   }
   if (scout_task == SCOUT_UNDOCK)
   {
      return UNDOCK_SCOUT;
   }

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Scout enum "<<scout_in_team<<" state:"<<scout_task);
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Excavator enum "<<excavator_in_team<<" state:"<<excavator_task);
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Hauler enum "<<hauler_in_team<<" state:"<<hauler_task);
}

TeamState& ScoutWaiting::transition()
{
   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | Excavator reached, Shifting to EXCAVATING");
      return getState(EXCAVATING);
   }
   else if (micro_state == MAKE_EXCAV_HAULER_IDLE)
   {
      return getState(GO_TO_REPAIR_STATION);
   }
   else
   {
      micro_state = getMicroState();
      return *this;
   }
}
   
void ScoutWaiting::step()
{
   switch (micro_state)
   {
   case ROBOTS_TO_GOAL:
      stepRobotsToGoal();
      break;
   case RECOVERY_SCOUT_FINDING:
      stepRecoveryScoutFinding();
      break;
   case MAKE_EXCAV_HAULER_IDLE:
      stepMakeExcavHaulerIdle();
      break;
   case RESET_SCOUT_EXCAV_ODOM:
      stepResetScoutExcavOdom();
      break;
   case UNDOCK_SCOUT:
      stepUndockScout();
      break;
   case PARK_EXCAVATOR_AT_SCOUT:
      stepParkExcavatorAtScout();
      break;
   default:
      ROS_WARN_STREAM("TEAM_LEVEL | team_state | Incorrect micro state found:"<<micro_state);
      break;
   }
}

void ScoutWaiting::stepRobotsToGoal()
{
   // Scout Goal
   robot_state_register->setRobotState(scout_in_team, SCOUT_LOCATE_VOLATILE);

   // Excavator Goal
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_GO_TO_SCOUT, volatile_site_location);

   // Hauler Goal
   // robot_state_register->setRobotState(hauler_in_team, HAULER_GO_BACK_TO_EXCAVATOR, volatile_site_location);
}

void ScoutWaiting::stepResetScoutExcavOdom()
{
   robot_state_register->setRobotState(scout_in_team, SCOUT_VISUAL_RESET_ODOM);

   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_VISUAL_RESET_ODOM);
}

void ScoutWaiting::stepMakeExcavHaulerIdle()
{
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
      
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

void ScoutWaiting::stepRecoveryScoutFinding()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_GO_TO_SCOUT_RECOVERY);
}

void ScoutWaiting::stepUndockScout()
{
   robot_state_register->setRobotState(scout_in_team, SCOUT_UNDOCK);
}

void ScoutWaiting::stepParkExcavatorAtScout()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_PARK_AND_PUB);
}

void ScoutWaiting::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of ScoutWaiting, cancelling ScoutWaiting goal");
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X C A V A T I N G   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Excavating::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of Excavating");

   if(excavator_in_team == NONE || hauler_in_team == NONE )
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | AT LEAST ONE ROBOT IS UNSET FOR Excavating!");
      return false;
   }

   ballet_once = true;
   digging_re_attempted = false;
   v_ppm_once = true;
   park_at_excav_re_attempted = false;

   volatile_site_location = robot_pose_register->getRobotPose(excavator_in_team);
   return true;
}

bool Excavating::isDone()
{
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team) && robot_state_register->hasSucceeded(hauler_in_team);

   return hauler_task == HAULER_UNDOCK_EXCAVATOR && hauler_done_and_succeeded;
}

TEAM_MICRO_STATE Excavating::getMicroState()
{
   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);

   bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team) && robot_state_register->hasSucceeded(scout_in_team);
   bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team) && robot_state_register->hasSucceeded(excavator_in_team);
   bool excavator_done_and_failed = robot_state_register->isDone(excavator_in_team) && !robot_state_register->hasSucceeded(excavator_in_team);
   bool hauler_done_and_failed = robot_state_register->isDone(hauler_in_team) && !robot_state_register->hasSucceeded(hauler_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team) && robot_state_register->hasSucceeded(hauler_in_team);

   if(excavator_task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: UNDOCK_HAULER  2");
         return UNDOCK_HAULER;
      }
   if (excavator_task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_done_and_failed && digging_re_attempted)
      {
         return UNDOCK_HAULER;
      }
   if(excavator_task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_done_and_failed && ballet_once)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: UNDOCK_HAULER  2");
         return BALLET_ONCE;
      }
   if(excavator_task == EXCAVATOR_PRE_HAULER_PARK_MANEUVER && excavator_done_and_failed)
      {
         return PRE_PARK_MANEUVER_RECOVERY;
      }
   if(excavator_task == EXCAVATOR_PRE_PARK_MANEUVER_RECOVERY && excavator_done_and_succeeded)
      {
         if(!v_ppm_once)
            park_at_excav_re_attempted = true;
         return PARK_AT_EXCAVATOR_HAULER;
      }   
   if(hauler_task == HAULER_PARK_AT_EXCAVATOR && hauler_done_and_failed)
      {
         v_ppm_once = false;
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: DIG_AND_DUMP  3");
         return PRE_PARK_MANEUVER_RECOVERY;
      }
   if(excavator_task == EXCAVATOR_DIG_AND_DUMP_VOLATILE)
   {
      if(!ballet_once)
      {
         digging_re_attempted = true;
         return DIG_AND_DUMP;
      }
   }
   if(((excavator_task == EXCAVATOR_BALLET_DANCING) || (hauler_task == HAULER_BALLET_DANCING)) && !digging_re_attempted)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: UNDOCK_HAULER  2");
         if((excavator_done_and_succeeded && hauler_done_and_succeeded)
            && (excavator_task == EXCAVATOR_BALLET_DANCING) && (hauler_task == HAULER_BALLET_DANCING))
            {
               ballet_once = false;
               return DIG_AND_DUMP;
            }
         else
            return BALLET_ONCE;
      }
   if(hauler_task == HAULER_PARK_AT_EXCAVATOR && hauler_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: DIG_AND_DUMP  3");
         return DIG_AND_DUMP;
      }
   if(excavator_task == EXCAVATOR_PRE_HAULER_PARK_MANEUVER && excavator_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: PARK_AT_EXCAVATOR_HAULER  4");
         return PARK_AT_EXCAVATOR_HAULER;
      }
   if((excavator_task == EXCAVATOR_VOLATILE_RECOVERY && excavator_done_and_succeeded)
      && (hauler_task == HAULER_GO_BACK_TO_EXCAVATOR && hauler_done_and_succeeded) )
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: PRE_PARK_MANEUVER_EXCAVATOR  5");
         return PRE_PARK_MANEUVER_EXCAVATOR;
      }
   if (hauler_task == HAULER_VISUAL_RESET_ODOM && robot_state_register->isDone(hauler_in_team))
   {
      if(excavator_task == EXCAVATOR_VISUAL_RESET_ODOM && robot_state_register->isDone(excavator_in_team))
      {
         return RECOVERY_EXCAVATOR_FINDING;
      }
   }
   if(hauler_task == HAULER_GO_BACK_TO_EXCAVATOR && hauler_done_and_failed)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  6");
         return RESET_EXCAV_HAULER_ODOM;
      }
   if((excavator_task == EXCAVATOR_VOLATILE_RECOVERY && excavator_done_and_succeeded)
      && hauler_task == HAULER_GO_TO_EXCAVATOR_RECOVERY && hauler_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  6");
         return PRE_PARK_MANEUVER_EXCAVATOR;
      }
   if((hauler_task == HAULER_VISUAL_RESET_ODOM && !hauler_done_and_succeeded) || 
      (excavator_task == EXCAVATOR_VISUAL_RESET_ODOM && !excavator_done_and_succeeded))
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  6");
         return RESET_EXCAV_HAULER_ODOM;
      }
   if(hauler_task == HAULER_GO_BACK_TO_EXCAVATOR && !hauler_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  6");
         return WAIT_FOR_HAULER;
      }
      
   if((hauler_task == ROBOT_IDLE_STATE /*&& excavator_task == ROBOT_IDLE_STATE*/) || excavator_task == EXCAVATOR_PARK_AND_PUB)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  7");
         return WAIT_FOR_HAULER;
      }
   if(hauler_task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_done_and_succeeded)
      {
         // ROS_INFO_STREAM("[ TEAM_LEVEL | team_state ]: WAIT_FOR_HAULER  1");
         return WAIT_FOR_HAULER;
      }

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Scout enum "<<scout_in_team<<" state:"<<scout_task);
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Excavator enum "<<excavator_in_team<<" state:"<<excavator_task);
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Hauler enum "<<hauler_in_team<<" state:"<<hauler_task);
}

TeamState& Excavating::transition()
{
   STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
   bool excavator_done_and_failed = robot_state_register->isDone(excavator_in_team) && !(robot_state_register->hasSucceeded(excavator_in_team));
   bool hauler_done_and_failed = robot_state_register->isDone(hauler_in_team) && !robot_state_register->hasSucceeded(hauler_in_team);

   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | Excavator reached, Shifting to DUMPING");
      return getState(GO_TO_REPAIR_STATION);
   }
   else if(excavator_task == EXCAVATOR_PRE_PARK_MANEUVER_RECOVERY && excavator_done_and_failed && park_at_excav_re_attempted)
   {
      ROS_INFO("TEAM_LEVEL | team_state | Excavation FAILED! Shifting to GO_TO_REPAIR_STATION");
      return getState(GO_TO_REPAIR_STATION);
   }
   else if(excavator_task == EXCAVATOR_VOLATILE_RECOVERY && excavator_done_and_failed)
   {
      ROS_INFO("TEAM_LEVEL | team_state | Excavation FAILED! Shifting to GO_TO_REPAIR_STATION");
      return getState(GO_TO_REPAIR_STATION);
   }
   else if(hauler_task == HAULER_GO_TO_EXCAVATOR_RECOVERY && hauler_done_and_failed)
   {
      ROS_INFO("TEAM_LEVEL | team_state | Excavation FAILED! Shifting to GO_TO_REPAIR_STATION");
      return getState(GO_TO_REPAIR_STATION);
   }
   else
   {
      micro_state = getMicroState();
      return *this;
   }
}
   
void Excavating::step()
{
   switch (micro_state)
   {
   case WAIT_FOR_HAULER:
      stepWaitForHauler();
      break;
   case RESET_EXCAV_HAULER_ODOM:
      stepResetExcavHaulerOdom();
      break;
   case RECOVERY_EXCAVATOR_FINDING:
      stepRecoveryExcavatorFinding();
      break;
   case PRE_PARK_MANEUVER_EXCAVATOR:
      stepPreParkManeuverExcavator();
      break;
   case PARK_AT_EXCAVATOR_HAULER:
      stepParkHauler();
      break;
   case DIG_AND_DUMP:
      stepDigAndDump();
      break;
   case BALLET_ONCE:
      stepBalletOnce();
      break;
   case UNDOCK_HAULER:
      stepUndockHauler();
   default:
      break;
   }
}

void Excavating::stepWaitForHauler()
{
   robot_state_register->setRobotState(hauler_in_team, HAULER_GO_BACK_TO_EXCAVATOR, volatile_site_location);

   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_VOLATILE_RECOVERY); 
}

void Excavating::stepResetExcavHaulerOdom()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_VISUAL_RESET_ODOM);

   robot_state_register->setRobotState(hauler_in_team, HAULER_VISUAL_RESET_ODOM);
}

void Excavating::stepRecoveryExcavatorFinding()
{
   robot_state_register->setRobotState(hauler_in_team, HAULER_GO_TO_EXCAVATOR_RECOVERY);

   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_VOLATILE_RECOVERY); 
}

void Excavating::stepPreParkManeuverExcavator()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_PRE_HAULER_PARK_MANEUVER); 
}

void Excavating::stepParkHauler()
{
   robot_state_register->setRobotState(hauler_in_team, excavator_in_team, HAULER_PARK_AT_EXCAVATOR);
}

void Excavating::stepDigAndDump()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_DIG_AND_DUMP_VOLATILE);
}

void Excavating::stepBalletOnce()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_BALLET_DANCING);

   robot_state_register->setRobotState(hauler_in_team, HAULER_BALLET_DANCING);
}

void Excavating::stepUndockHauler()
{
   robot_state_register->setRobotState(hauler_in_team, excavator_in_team, HAULER_UNDOCK_EXCAVATOR);
}

void Excavating::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of Excavating, cancelling Excavating goal");
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P I N G   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Dumping::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of Dumping");

   if(hauler_in_team == NONE )
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | HAULER IS UNSET FOR DUMPING!");
      return false;
   }
   return true;
}

bool Dumping::isDone()
{
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team) && robot_state_register->hasSucceeded(hauler_in_team);

   return hauler_task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_done_and_succeeded;
}

TeamState& Dumping::transition()
{
   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | HAULER reached, Shifting to IDLE");
      return getState(IDLE);
   }
   else
   {
      return *this;
   }
}
   
void Dumping::step()
{
   robot_state_register->setRobotState(hauler_in_team, HAULER_DUMP_VOLATILE_TO_PROC_PLANT);
}

void Dumping::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of Dumping, cancelling Dumping goal");
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O   T O   R E P A I R   S T A T I O N   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToRepairStation::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of GoToRepairStation");

   if(checkForRespectiveRobots())
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | Multiple robots set for GoToRepairStation!");
      return false;
   }

   if(scout_in_team != NONE)
      scout_robot_staring_state = robot_state_register->currentState(scout_in_team);
   if(excavator_in_team != NONE)
      excavator_robot_staring_state = robot_state_register->currentState(excavator_in_team);
   if(hauler_in_team != NONE)
      hauler_robot_staring_state = robot_state_register->currentState(hauler_in_team);

   return true;
}

bool GoToRepairStation::checkForRespectiveRobots()
{
   bool only_scout = scout_in_team != NONE && excavator_in_team == NONE && hauler_in_team == NONE;
   bool only_excavator = scout_in_team == NONE && excavator_in_team != NONE && hauler_in_team == NONE;
   bool only_hauler = scout_in_team == NONE && excavator_in_team == NONE && hauler_in_team != NONE;
   return !(only_scout || only_excavator || only_hauler);
} 


bool GoToRepairStation::isDone()
{
   if(scout_in_team != NONE)
   {
      STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
      bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team);// && robot_state_register->hasSucceeded(scout_in_team);

      return scout_task == SCOUT_GOTO_REPAIR_STATION && scout_done_and_succeeded;
   }
   if(excavator_in_team != NONE)
   {
      STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
      bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team);// && robot_state_register->hasSucceeded(excavator_in_team);

      return excavator_task == EXCAVATOR_GO_TO_REPAIR && excavator_done_and_succeeded;
   }
   if(hauler_in_team != NONE)
   {
      STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
      bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team);// && robot_state_register->hasSucceeded(hauler_in_team);

      return hauler_task == HAULER_VISUAL_RESET_ODOM && hauler_done_and_succeeded;
   }
}

TeamState& GoToRepairStation::transition()
{
   if(isDone())
   {
      if(hauler_in_team != NONE)
      {
         ROS_INFO("TEAM_LEVEL | team_state | Robots reached GoToRepairStation, Shifting to WAIT_FOR_HOPPER_APPOINTMENT");
         return getState(WAIT_FOR_HOPPER_APPOINTMENT);
      }
      else
      {
         ROS_INFO("TEAM_LEVEL | team_state | Robots reached GoToRepairStation, Shifting to IDLE");
         return getState(IDLE);
      }
   }
   else
   {
      return *this;
   }
}

TEAM_MICRO_STATE GoToRepairStation::getMicroState()
{
   if(scout_in_team != NONE)
      return getScoutMicroState();
   if(excavator_in_team != NONE)
      return getExcavatorMicroState();
   if(hauler_in_team != NONE)
      return getHaulerMicroState();
}

TEAM_MICRO_STATE GoToRepairStation::getScoutMicroState()
{
   STATE_MACHINE_TASK scout_task = robot_state_register->currentState(scout_in_team);
   bool scout_done_and_succeeded = robot_state_register->isDone(scout_in_team) && robot_state_register->hasSucceeded(scout_in_team);
   
   if (scout_task == scout_robot_staring_state)
      return VISUAL_RESET_ODOM;
   if (scout_task == SCOUT_VISUAL_RESET_ODOM && !robot_state_register->isDone(scout_in_team))
      return VISUAL_RESET_ODOM;
   if (scout_task == SCOUT_VISUAL_RESET_ODOM && robot_state_register->isDone(scout_in_team))
      return GO_TO_REPAIR_STATION_MICRO;
   if (scout_task == SCOUT_GOTO_REPAIR_STATION && !robot_state_register->isDone(scout_in_team))
      return GO_TO_REPAIR_STATION_MICRO;
   if (scout_task == SCOUT_GOTO_REPAIR_STATION && robot_state_register->isDone(scout_in_team))
      return IDLE_MICRO_STATE;

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Scout enum "<<scout_in_team<<" state:"<<scout_task);   
}

TEAM_MICRO_STATE GoToRepairStation::getExcavatorMicroState()
{
   STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
   bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team);// && robot_state_register->hasSucceeded(excavator_in_team);

   if (excavator_task == excavator_robot_staring_state)
      return GO_TO_LOOKOUT_LOCATION;
   if (excavator_task == EXCAVATOR_GO_TO_LOOKOUT_LOCATION && !excavator_done_and_succeeded)
      return GO_TO_LOOKOUT_LOCATION;
   if (excavator_task == EXCAVATOR_GO_TO_LOOKOUT_LOCATION && excavator_done_and_succeeded)
      return VISUAL_RESET_ODOM;
   if (excavator_task == EXCAVATOR_VISUAL_RESET_ODOM &&  !excavator_done_and_succeeded)
      return VISUAL_RESET_ODOM;
   if (excavator_task == EXCAVATOR_VISUAL_RESET_ODOM && excavator_done_and_succeeded)
      return GO_TO_REPAIR_STATION_MICRO;
   if (excavator_task == EXCAVATOR_GO_TO_REPAIR &&  !excavator_done_and_succeeded)
      return GO_TO_REPAIR_STATION_MICRO;
   if (excavator_task == EXCAVATOR_GO_TO_REPAIR && excavator_done_and_succeeded)
      return IDLE_MICRO_STATE;

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Excavator enum "<<excavator_in_team<<" state:"<<excavator_task);   
}

TEAM_MICRO_STATE GoToRepairStation::getHaulerMicroState()
{
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team);// && robot_state_register->hasSucceeded(hauler_in_team);

   if(hauler_task == hauler_robot_staring_state)
      return GO_TO_LOOKOUT_LOCATION;
   if(hauler_task == HAULER_GO_TO_LOOKOUT_LOCATION && !hauler_done_and_succeeded)
      return GO_TO_LOOKOUT_LOCATION;
   if(hauler_task == HAULER_GO_TO_LOOKOUT_LOCATION && hauler_done_and_succeeded)
      return VISUAL_RESET_ODOM;
   if(hauler_task == HAULER_VISUAL_RESET_ODOM && !hauler_done_and_succeeded)
      return VISUAL_RESET_ODOM;
   if(hauler_task == HAULER_VISUAL_RESET_ODOM && hauler_done_and_succeeded)
      return IDLE_MICRO_STATE;

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Hauler enum "<<hauler_in_team<<" state:"<<hauler_task);   
}

void GoToRepairStation::step()
{
   switch(getMicroState())
   {
      case GO_TO_LOOKOUT_LOCATION:
         stepGoToLookoutLocation();
         break;
      case VISUAL_RESET_ODOM:
         stepVisualResetOdom();
         break;
      case GO_TO_REPAIR_STATION_MICRO:
         stepGoToRepairStationMicro();
         break;
      case IDLE_MICRO_STATE:
         stepIdle();
         break;
   }
}

void GoToRepairStation::stepGoToLookoutLocation()
{
   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_GO_TO_LOOKOUT_LOCATION);

   robot_state_register->setRobotState(hauler_in_team, HAULER_GO_TO_LOOKOUT_LOCATION);
}

void GoToRepairStation::stepVisualResetOdom()
{
   robot_state_register->setRobotState(scout_in_team, SCOUT_VISUAL_RESET_ODOM);

   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_VISUAL_RESET_ODOM);

   robot_state_register->setRobotState(hauler_in_team, HAULER_VISUAL_RESET_ODOM);
}

void GoToRepairStation::stepGoToRepairStationMicro()
{
   robot_state_register->setRobotState(scout_in_team, SCOUT_GOTO_REPAIR_STATION);

   robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_GO_TO_REPAIR);
}

void GoToRepairStation::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of GoToRepairStation, cancelling GoToRepairStation goal");
   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);

   robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// W A I T  F O R   H O P P E R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool WaitForHopperAppointment::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of WaitForHopperAppointment");
   return true;
}

bool WaitForHopperAppointment::isDone()
{
   return true;
}

TeamState& WaitForHopperAppointment::transition()
{
   return *this;
}
   
void WaitForHopperAppointment::step()
{
   if(scout_in_team != NONE)
      robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   if(excavator_in_team != NONE)
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   if(hauler_in_team != NONE)
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
}

void WaitForHopperAppointment::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of WaitForHopperAppointment, cancelling WaitForHopperAppointment goal");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ I N I T _ L O C   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool GoToInitLoc::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of GoToInitLoc");

   if(excavator_in_team == NONE && hauler_in_team == NONE)
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | No robot set for GoToInitLoc!");
      return false;
   }

   return true;
}

bool GoToInitLoc::isDone()
{
   if(excavator_in_team != NONE)
   {
      STATE_MACHINE_TASK excavator_task = robot_state_register->currentState(excavator_in_team);
      bool excavator_done_and_succeeded = robot_state_register->isDone(excavator_in_team);// && robot_state_register->hasSucceeded(excavator_in_team);

      return excavator_task == EXCAVATOR_GO_TO_LOOKOUT_LOCATION && excavator_done_and_succeeded;
   }
   if(hauler_in_team != NONE)
   {
      STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
      bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team);// && robot_state_register->hasSucceeded(hauler_in_team);

      return hauler_task == HAULER_GO_TO_LOOKOUT_LOCATION && hauler_done_and_succeeded;
   }
}

TeamState& GoToInitLoc::transition()
{
   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | Robots reached, Shifting to IDLE");
      return getState(IDLE);
   }
   else
   {
      return *this;
   }
}
   
void GoToInitLoc::step()
{
   if(excavator_in_team != NONE)
      robot_state_register->setRobotState(excavator_in_team, EXCAVATOR_GO_TO_LOOKOUT_LOCATION);
   if(hauler_in_team != NONE)
   {
      if(hauler_in_team == HAULER_2)
         robot_state_register->setRobotState(hauler_in_team, HAULER_INITIAL_RESET);
      else
         robot_state_register->setRobotState(hauler_in_team, HAULER_GO_TO_LOOKOUT_LOCATION);
   }
}

void GoToInitLoc::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of GoToInitLoc, cancelling GoToRepairStation goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// R E S E T   A T   H O P P E R   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ResetAtHopper::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   if(!(scout_in_team == NONE && excavator_in_team == NONE && hauler_in_team != NONE))
   {
      ROS_ERROR_STREAM("TEAM_LEVEL | team_state | Robot other than Hauler found!");
      return false;
   }
   
   hauler_robot_staring_state = robot_state_register->currentState(hauler_in_team);
   ROS_INFO("TEAM_LEVEL | team_state | entrypoint of ResetAtHopper");
   return true;
}

bool ResetAtHopper::isDone()
{
   if(hauler_in_team != NONE)
   {
      STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
      bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team);// && robot_state_register->hasSucceeded(hauler_in_team);

      return hauler_task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_done_and_succeeded;
   }
}

TeamState& ResetAtHopper::transition()
{
   if(isDone())
   {
      ROS_INFO("TEAM_LEVEL | team_state | Robots reached, Shifting to IDLE");
         return getState(IDLE);
   }
   else
   {
      return *this;
   }
}

TEAM_MICRO_STATE ResetAtHopper::getMicroState()
{
   STATE_MACHINE_TASK hauler_task = robot_state_register->currentState(hauler_in_team);
   bool hauler_done_and_succeeded = robot_state_register->isDone(hauler_in_team);// && robot_state_register->hasSucceeded(hauler_in_team);

   if(hauler_task == hauler_robot_staring_state)
      return GO_TO_REPAIR_STATION_MICRO;
   if(hauler_task == HAULER_GOTO_REPAIR_STATION && !hauler_done_and_succeeded)
      return GO_TO_REPAIR_STATION_MICRO;
   if(hauler_task == HAULER_GOTO_REPAIR_STATION && hauler_done_and_succeeded)
      return DUMP_VOLATILE_AT_HOPPER;
   if(hauler_task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && !hauler_done_and_succeeded)
      return DUMP_VOLATILE_AT_HOPPER;
   if(hauler_task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_done_and_succeeded)
      return IDLE_MICRO_STATE;

   ROS_WARN("TEAM_LEVEL | team_state | Unknown Combination of the robot states found!");
   ROS_WARN_STREAM("TEAM_LEVEL | team_state | Hauler enum "<<hauler_in_team<<" state:"<<hauler_task);   
}

void ResetAtHopper::step()
{
   switch (getMicroState())
   {
   case GO_TO_REPAIR_STATION_MICRO:
      robot_state_register->setRobotState(hauler_in_team, HAULER_GOTO_REPAIR_STATION);
      break;
   
   case DUMP_VOLATILE_AT_HOPPER:
      robot_state_register->setRobotState(hauler_in_team, HAULER_DUMP_VOLATILE_TO_PROC_PLANT);
      break;
   
   case IDLE_MICRO_STATE:
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
      break;
   
   default:
      break;
   }
}

void ResetAtHopper::exitPoint() 
{
   ROS_INFO("TEAM_LEVEL | team_state | exitpoint of ResetAtHopper, cancelling ResetAtHopper goal");
   if(scout_in_team != NONE)
   {
      robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(scout_in_team, ROBOT_IDLE_STATE);
   }
   if(excavator_in_team != NONE)
   {
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(excavator_in_team, ROBOT_IDLE_STATE);
   }
   if(hauler_in_team != NONE)
   {
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
      robot_state_register->setRobotState(hauler_in_team, ROBOT_IDLE_STATE);
   }
}