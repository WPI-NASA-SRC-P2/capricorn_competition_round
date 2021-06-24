#include <team_level/team_state.h>


TeamState::TeamState(uint32_t un_id, const std::string& str_name, ros::NodeHandle &nh) :
      m_pcTeam(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
    robot_status = new RobotStatus(nh);

    robot_state_publisher_  = nh.advertise<state_machines::robot_desired_state>(COMMON_NAMES::CAPRICORN_TOPIC + ROBOTS_DESIRED_STATE_TOPIC, 1, true);
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

   if(ROBOT_ENUM_NAME_MAP.find(scout_in_team) == ROBOT_ENUM_NAME_MAP.end()) {
      ROS_ERROR_STREAM("Set Scout "<<scout_in_team<<" doesn't exist on the ROBOT_ENUM_NAME_MAP");
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
      return *this;
   else
   {
      ROS_INFO("volatile detected, transitioning to scout_undock state");
      return getState(SCOUT_WAITING);
   }
}
   
void Search::step()
{
   if(ROBOT_ENUM_NAME_MAP.find(scout_in_team) != ROBOT_ENUM_NAME_MAP.end()) {
    state_machines::robot_desired_state desired_state_msg;
    desired_state_msg.robot_name = ROBOT_ENUM_NAME_MAP[scout_in_team];
    desired_state_msg.robot_desired_state = SCOUT_SEARCH_VOLATILE;
    robot_state_publisher_.publish(desired_state_msg);
   }
   else{
      ROS_ERROR_STREAM("Scout enum "<<scout_in_team<<" not found in map ROBOT_ENUM_NAME_MAP");
   }
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

   bool scout_map_check = ROBOT_ENUM_NAME_MAP.find(scout_in_team) == ROBOT_ENUM_NAME_MAP.end();
   bool excavator_map_check = ROBOT_ENUM_NAME_MAP.find(excavator_in_team) == ROBOT_ENUM_NAME_MAP.end();
   bool hauler_map_check = ROBOT_ENUM_NAME_MAP.find(hauler_in_team) == ROBOT_ENUM_NAME_MAP.end();

   if(scout_map_check || excavator_map_check || hauler_map_check) {
      ROS_ERROR_STREAM("One of the robots doesn't exist on the ROBOT_ENUM_NAME_MAP");
      ROS_ERROR_STREAM("Robots: Scout:"<<scout_in_team<<"\t Excavator:"<<excavator_in_team<<"\t Hauler:"<<hauler_in_team);
      return false;
   }
   return true;
}

bool ScoutWaiting::isDone()
{
   return robot_status->isDone(excavator_in_team);
}

TeamState& ScoutWaiting::transition()
{
   if(isDone())
      return *this;
   else
   {
      ROS_INFO("Excavator reached, Shifting to EXCAVATING");
      return getState(EXCAVATING);
   }
}
   
void ScoutWaiting::step()
{
   if(ROBOT_ENUM_NAME_MAP.find(excavator_in_team) != ROBOT_ENUM_NAME_MAP.end()) {
    state_machines::robot_desired_state desired_state_msg;
    desired_state_msg.robot_name = ROBOT_ENUM_NAME_MAP[excavator_in_team];
    desired_state_msg.robot_desired_state = EXCAVATOR_GO_TO_LOC;
    robot_state_publisher_.publish(desired_state_msg);
   }
   else{
      ROS_ERROR_STREAM("Scout enum "<<excavator_in_team<<" not found in map ROBOT_ENUM_NAME_MAP");
   }
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
