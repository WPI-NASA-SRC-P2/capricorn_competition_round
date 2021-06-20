#include <team_level/scheduler.h>
#include <algorithm>

/****************************************/
/****************************************/

TeamState& TeamState::getState(uint32_t un_state) {
   return m_pcRobotScheduler->getState(un_state);
}

/****************************************/
/****************************************/

//UNDERSTANDING: When we add states in addState(state), the state is assigned to a scheduler. 
void TeamState::setRobotScheduler(TeamScheduler& c_robot_scheduler) {
   m_pcRobotScheduler = &c_robot_scheduler;
}

/****************************************/
/****************************************/

TeamScheduler::~TeamScheduler() {
   std::for_each(
      m_mapStates.begin(),
      m_mapStates.end(),
      [](std::pair<uint32_t, TeamState*> c_item){
         delete c_item.second;
      });
}

/****************************************/
/****************************************/

//UNDERSTANDING: Adding states to an unordered map (map of states declared in the header).

void TeamScheduler::addState(TeamState* pc_state) {
   if(m_mapStates.find(pc_state->getId()) == m_mapStates.end()) {
      m_mapStates[pc_state->getId()] = pc_state;
      pc_state->setRobotScheduler(*this);
   }
   else {
      throw StateMachineException(ToString("Duplicated state id ", pc_state->getId()));
   }
}

/****************************************/
/****************************************/

TeamState& TeamScheduler::getState(uint32_t un_id) {
   auto pcState = m_mapStates.find(un_id);
   if(pcState != m_mapStates.end()) {
      return *(pcState->second);
   }
   else {
      throw StateMachineException(ToString("Can't get state id ", un_id));
   }
}

/****************************************/
/****************************************/

//UNDERSTANDING: First finds the state in the map and then sets the entry point of the current state.

void TeamScheduler::setInitialState(uint32_t un_state) {
   auto pcState = m_mapStates.find(un_state);
   // if state exists in map, then set it to the initial state of the scheduler
   if(pcState != m_mapStates.end()) {
      // acquire value of the state (every map has a key(first) and a value(second))
      m_pcCurrent = pcState->second;
      // completes entry point of the initial state
      m_pcCurrent->entryPoint();
   }
   else {
      throw StateMachineException(ToString("Can't set initial state to ", un_state));
   }
}

/****************************************/
/****************************************/

void disbandingDoneTeam()
{
  std::vector<*ExcavationTeam>::iterator currTeam = activeTeams.begin();
  while (currTeam != activeTeams.end())
  {
    if(currTeam->shouldDisband())
    { 
      activeTeams.erase(currTeam++)
    }
  }
}

void checkForNewVolatile()
{
  for(int i = 0; i<scouts.size(); i++)
  {
    if(scouts.at(i)->getCurrentState() == SCOUT_LOCATE)
    {
      scout_vecPair_volAvl_recruitedTeam.at(i).first = true;
    }
  }
}

void TeamScheduler::step() {
  // Check if can disband an active team
  disbandingDoneTeam();

  // Recruit missing hauler

  // Check if scout has found anything
  checkForNewVolatile();

  // Check if that scout has been assigned a ticket
  // *ticket: an excavator has been assigned for the volatile spot
  // If not, 
  for(int i = 0; i<scouts.size(); i++)
  {
  if(scout_vecPair_volAvl_recruitedTeam.at(i).first)
  if(!scout_vecPair_volAvl_recruitedTeam.at(i).second)
  {
  // Recruit Excavator
    // Check if excavator has been rectruited, assign a ticket to scout
    for(int j = 0; j<excavators.size(); j++)
    {
      if(excavators.at(j)->getCurrentState == IDLE)
      {
        // Add the new team to vector of active teams
        activeTeams.push_back(new ExcavationTeam());
        activeTeams.back.setExcavator((EXCAVATOR_ENUM)j)
        activeTeams.back.setScout((SCOUT_ENUM)i)
        // Check if hauler has been recruited, if not, recruit one
        for(int k = 0; k<haulers.size(); k++)
        {
          if(haulers.at(k)->getCurrentState == IDLE)
          {
            activeTeams.back.setHauler((EXCAVATOR_ENUM)k)
            break;
          }
        }
        scout_vecPair_volAvl_recruitedTeam.at(i).second = true;
        break;
      }
    }
  }
  }
  

  // iterate over the active teams

  // update the macro state for the current team

  // check if the robots are in the states that they need to be. 
  // If not, update the macro state

  // Get the robot's states according to the macro state (redundant)

  // Execute the robot states
}











   /* Only execute if 'current' was initialized */
  //  if(m_pcCurrent) {
  //     /* Attempt a transition, every state of every rover has its own transition() */
  //     TeamState* cNewState = &m_pcCurrent->transition();
  //     if (m_bInterrupt)
  //     {
  //        cNewState = &getState(interrupt_state_);
  //        m_bInterrupt = false;
  //     }

  //     if(cNewState != m_pcCurrent) {
  //        /* Perform transition */
  //        m_pcCurrent->exitPoint();
  //        cNewState->entryPoint();
  //        m_pcCurrent = cNewState;
  //     }
  //     /* Execute current state */
  //     m_pcCurrent->step();
  //  }
  //  else {
  //     throw StateMachineException("The Robotscheduler has not been initialized, you must call SetInitialState()");
  //  }
}

/****************************************/
/****************************************/

//UNDERSTANDING: Each robot has its own done() and this is what is checked to perform step()
void TeamScheduler::exec() {
   while(!done() && ros::ok()) 
   {
      step();
      ros::spinOnce();
   }
}

/****************************************/
/****************************************/
