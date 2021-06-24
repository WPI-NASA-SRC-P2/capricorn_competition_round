#include <team_level/team_state.h>


TeamState::TeamState(uint32_t un_id, const std::string& str_name, ros::NodeHandle &nh) :
      m_pcTeam(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
    robot_status = new RobotStatus(nh);
}

void TeamState::setTeam(Team& c_robot_scheduler) {
   m_pcTeam = &c_robot_scheduler;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S T A N D B Y   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Standby::entryPoint(ROBOTS_ENUM scout, ROBOTS_ENUM excavator, ROBOTS_ENUM hauler)
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("entrypoint of Standby");
   if(!(scout == NONE && excavator == NONE && hauler == NONE))
   {
      ROS_ERROR_STREAM("STANDBY STATE CALLED, BUT AT LEAST ONE ROBOT IS NOT UNSET");
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
   ROS_INFO("exitpoint of STANDBY, cancelling STANDBY goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Idle::entryPoint(ROBOTS_ENUM scout, ROBOTS_ENUM excavator, ROBOTS_ENUM hauler)
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("entrypoint of Idle");
   if(scout == NONE && excavator == NONE && hauler == NONE)
   {
      ROS_ERROR_STREAM("IDLE STATE CALLED, BUT NO ROBOT IS SET");
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
   // Do Nothing
}

void Idle::exitPoint() 
{
   ROS_INFO("exitpoint of IDLE, cancelling IDLE goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Search::entryPoint(ROBOTS_ENUM scout, ROBOTS_ENUM excavator, ROBOTS_ENUM hauler)
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("entrypoint of Search");
   scout_in_team = scout;
   excavator_in_team = excavator;
   hauler_in_team = hauler;

   if(scout_in_team == NONE)
   {
      ROS_ERROR_STREAM("SCOUT IS UNSET, BUT STILL ENTRY POINT HAS BEEN CALLED!");
      return false;
   }

   return true;
}

bool Search::isDone()
{
   return robot_status->isDone(scout_in_team);
}

TeamState& Search::transition()
{
   if(isDone())
   {
      ROS_INFO("volatile detected, transitioning to scout_undock state");
      return getState(SCOUT_WAITING);
   }
   else
      return *this;
}
   
void Search::step()
{
   robot_status->setRobotState(scout_in_team, SCOUT_SEARCH_VOLATILE);
}

void Search::exitPoint() 
{
   ROS_INFO("exitpoint of SEARCH, cancelling SEARCH goal");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T _ W A I T I N G   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool ScoutWaiting::entryPoint(ROBOTS_ENUM scout, ROBOTS_ENUM excavator, ROBOTS_ENUM hauler)
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("entrypoint of ScoutWaiting");
   scout_in_team = scout;
   excavator_in_team = excavator;
   hauler_in_team = hauler;

   if(scout_in_team == NONE || excavator_in_team == NONE || hauler_in_team == NONE )
   {
      ROS_ERROR_STREAM("AT LEAST ONE ROBOT IS UNSET FOR SCOUT WAITING!");
      return false;
   }
   return true;
}

bool ScoutWaiting::isDone()
{
   STATE_MACHINE_TASK excavator_task = robot_status->currentState(excavator_in_team);
   bool excavator_done_and_succeeded = robot_status->isDone(excavator_in_team) && robot_status->hasSucceeded(excavator_in_team);

   return excavator_task == PARK_EXCAVATOR_AT_SCOUT && excavator_done_and_succeeded;
}

TEAM_MICRO_STATE ScoutWaiting::getMicroState()
{
   STATE_MACHINE_TASK scout_task = robot_status->currentState(scout_in_team);
   STATE_MACHINE_TASK excavator_task = robot_status->currentState(excavator_in_team);
   STATE_MACHINE_TASK hauler_task = robot_status->currentState(hauler_in_team);

   bool scout_done_and_succeeded = robot_status->isDone(scout_in_team) && robot_status->hasSucceeded(scout_in_team);
   bool excavator_done_and_succeeded = robot_status->isDone(excavator_in_team) && robot_status->hasSucceeded(excavator_in_team);
   bool hauler_done_and_succeeded = robot_status->isDone(hauler_in_team) && robot_status->hasSucceeded(hauler_in_team);

   if (scout_task == SCOUT_SEARCH_VOLATILE)
      return ROBOTS_TO_GOAL;
   if (excavator_task == EXCAVATOR_MACRO_GO_TO_SCOUT && excavator_done_and_succeeded)
   {
      if (scout_task == SCOUT_LOCATE_VOLATILE && scout_done_and_succeeded)
         return UNDOCK_SCOUT;
   }
   if (scout_task == SCOUT_MACRO_UNDOCK && scout_done_and_succeeded)
   {
      return PARK_EXCAVATOR_AT_SCOUT;
   }
   // Now for the current state
   if (scout_task == SCOUT_LOCATE_VOLATILE)
   {
      bool excav_in_process = excavator_task == EXCAVATOR_MACRO_GO_TO_SCOUT;
      bool excav_absent = excavator_task == ROBOT_IDLE_STATE;

      bool hauler_in_process = hauler_task == HAULER_GO_TO_LOC;
      bool hauler_absent = hauler_task == ROBOT_IDLE_STATE;
      
      if(!(excav_in_process || excav_absent))
         ROS_WARN_STREAM("Excavator state anomaly!"<<excavator_task);
      if(!(hauler_in_process || hauler_absent))
         ROS_WARN_STREAM("Hauler state anomaly!"<<hauler_task);
      return ROBOTS_TO_GOAL;
   }
   else
   {
      ROS_WARN("Unknown Combination of the robot states found!");
      ROS_WARN_STREAM("Scout enum "<<scout_in_team<<" state:"<<scout_task);
      ROS_WARN_STREAM("Excavator enum "<<excavator_in_team<<" state:"<<excavator_task);
      ROS_WARN_STREAM("Hauler enum "<<hauler_in_team<<" state:"<<hauler_task);
   }
}

TeamState& ScoutWaiting::transition()
{
   if(isDone())
   {
      ROS_INFO("Excavator reached, Shifting to EXCAVATING");
      return getState(EXCAVATING);
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
   case UNDOCK_SCOUT:
      stepUndockScout();
      break;
   case PARK_EXCAVATOR_AT_SCOUT:
      stepParkExcavatorAtScout();
   default:
      break;
   }
}

void ScoutWaiting::stepRobotsToGoal()
{
   // Scout Goal
   robot_status->setRobotState(scout_in_team, SCOUT_LOCATE_VOLATILE);

   // Excavator Goal
   robot_status->setRobotState(excavator_in_team, EXCAVATOR_MACRO_GO_TO_SCOUT);

   // Hauler Goal
   robot_status->setRobotState(hauler_in_team, HAULER_GO_TO_LOC);
}

void ScoutWaiting::stepUndockScout()
{
   robot_status->setRobotState(scout_in_team, SCOUT_MACRO_UNDOCK);
}

void ScoutWaiting::stepParkExcavatorAtScout()
{
   robot_status->setRobotState(excavator_in_team, EXCAVATOR_PARK_AND_PUB);
}

void ScoutWaiting::exitPoint() 
{
   ROS_INFO("exitpoint of SEARCH, cancelling SEARCH goal");
}

// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "scout_state_machine");
//    ros::NodeHandle &nh;

//    try {
//       ScoutScheduler cSchd(700);
//       cSchd.addState(new ScoutWaiting());
//       cSchd.addState(new STANDBY());
//       cSchd.addState(new Locate());
//       cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
//       // cSchd.setInitialState(SCOUT_UNDOCK);
//       cSchd.exec();
//       return 0;
//    }
//    catch(StateMachineException& ex) {
//       std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
//    }
// }
