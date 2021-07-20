#include <team_level/team_scheduler.h>

TeamScheduler::TeamScheduler(ros::NodeHandle &nh)
{
    addStates(nh);
    setInitialState(STANDBY);
}

TeamScheduler::~TeamScheduler()
{   
   std::for_each(
      MACRO_STATE_PTR_MAP.begin(),
      MACRO_STATE_PTR_MAP.end(),
      [](std::pair<uint64_t, TeamState*> c_item){
         delete c_item.second;
      });
}

void TeamScheduler::addStates(ros::NodeHandle &nh)
{
    addState(new Search(nh));
    addState(new ScoutWaiting(nh));
    addState(new Excavating(nh));
    addState(new Dumping(nh));
    addState(new Standby(nh));
    addState(new Idle(nh));
    addState(new GoToRepairStation(nh));
    addState(new WaitForHopperAppointment(nh));
    addState(new ResetAtHopper(nh));
    addState(new GoToInitLoc(nh));
}

void TeamScheduler::addState(TeamState* pc_state) {
   if(MACRO_STATE_PTR_MAP.find(pc_state->getId()) == MACRO_STATE_PTR_MAP.end()) {
      MACRO_STATE_PTR_MAP[pc_state->getId()] = pc_state;
      pc_state->setTeam(*this);
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]: Duplicated state id :" << pc_state->getId());
   }
}

TeamState& TeamState::getState(uint64_t un_state) {
   return m_pcTeam->getState(un_state);
}

TeamState& TeamScheduler::getState(uint64_t un_id) 
{
   auto pcState = MACRO_STATE_PTR_MAP.find(un_id);
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      return *(pcState->second);
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]:Can't get state id " << un_id);
   }
}

void TeamScheduler::setInitialState(uint64_t un_state) {
   auto pcState = MACRO_STATE_PTR_MAP.find(un_state);
   // if state exists in map, then set it to the initial state of the scheduler
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      // acquire value of the state (every map has a key(first) and a value(second))
      current_state_ptr = pcState->second;
      current_state_ptr->updateRobots(hired_scout, hired_excavator, hired_hauler);

      // completes entry point of the initial state
      current_state_ptr->entryPoint();
      setTeamMacroState((TEAM_MACRO_STATE) un_state);
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]: Can't set initial state to " << un_state);
   }
}

void TeamScheduler::step() {
   /* Only execute if 'current' was initialized */
   if(current_state_ptr) {
      //  ROS_INFO_STREAM(hired_scout << "  " << hired_excavator << "  " << hired_hauler);
    //    ROS_INFO_STREAM("Robot enum:" << SCOUT_1);
      /* Attempt a transition, every state of every rover has its own transition() */
      TeamState* cNewState = &current_state_ptr->transition();
      // TeamState* cNewState = current_state_ptr;
      if (new_state_request)
      {
         cNewState = &getState(macro_state);
         new_state_request = false;
      }

      current_state_ptr->updateRobots(hired_scout, hired_excavator, hired_hauler);
      if(cNewState != current_state_ptr) {
         /* Perform transition */
         current_state_ptr->exitPoint();
         cNewState->updateRobots(hired_scout, hired_excavator, hired_hauler);
         // cNewState->setResetRobot(reset_robot_odometry);
         bool entry = cNewState->entryPoint();
         
         setTeamMacroState((TEAM_MACRO_STATE) cNewState->getId());
         if(!entry)
            return;
         current_state_ptr = cNewState;
      }
      /* Execute current state */
      current_state_ptr->step();
   }
   else {
      ROS_ERROR_STREAM("[TEAM_LEVEL | team_scheduler ]: The Team has not been initialized, you must call SetInitialState()");
   }
}

/****************************************/
/****************************************/

//UNDERSTANDING: Each robot has its own done() and this is what is checked to perform step()
void TeamScheduler::exec() {
   ros::Rate r(50); // 50Hz loop rate
   while(ros::ok())
   {
      step();
      ros::spinOnce();
      r.sleep();
   }
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "[TEAM_LEVEL | team_scheduler ]: scout_state_machine");
   ros::NodeHandle nh;

   // TeamScheduler team(nh, TEAM_MACRO_STATE::STANDBY, ROBOTS_ENUM::SCOUT_1, ROBOTS_ENUM::NONE, ROBOTS_ENUM::NONE);
   // team.exec();

    // ros::Duration(1).sleep();
//     team.setTeamMacroState(SEARCH);
//     ros::Duration(1).sleep();
//     team.setTeamMacroState(SEARCH);
//     ros::Duration(1).sleep();
//     team.setTeamMacroState(SEARCH);
//     ros::spinOnce();
    return 0;
}
