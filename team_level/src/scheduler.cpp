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
    if(currTeam->shouldTeamDisband())
    { 
      currTeam->disbandTeam();
      activeTeams.erase(currTeam++);
    }
    if(shouldScoutDisband())
    {
      currTeam->disbandScout();
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

void updateMacroState()
{
  // iterate over the active teams

  // update the macro state for the current team

  // check if the robots are in the states that they need to be. 
  // If not, update the macro state

  // Get the robot's states according to the macro state (redundant)

}

void executeTeamStates()
{
  // Execute the robot states

}

void recruitHauler(ExcavationTeam &team)
{
  for(int k = 0; k<haulers.size(); k++)
  {
    if(haulers.at(k)->getCurrentState == IDLE)
    {
      team->setHauler((EXCAVATOR_ENUM)k)
      break;
    }
  }
}

void recruiTeams()
{
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
        recruitHauler(activeTeams.back);

        scout_vecPair_volAvl_recruitedTeam.at(i).second = true;
        break;
      }
    }
  }
  }
}

void recruitHaulerIfNeeded()
{
  for (auto team : activeTeams)
  {
    if(!team->isHaulerRecruited())
    {
      recruitHauler(team);
    }
  }
}

void TeamScheduler::step() {
  // Check if can disband an active team
  disbandingDoneTeam();

  // Recruit missing hauler
  recruitHaulerIfNeeded();

  // Check if scout has found anything
  checkForNewVolatile();

  recruiTeams();

  updateMacroState();
  executeTeamStates();  
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
