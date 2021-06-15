#include <state_machines/common_robot_state_machine.h>
#include <algorithm>

/****************************************/
/****************************************/

StateMachineException::StateMachineException(const std::string& str_msg) :
   m_strMsg(str_msg) {}

/****************************************/
/****************************************/

std::string StateMachineException::getMessage() const {
   return m_strMsg;
}

/****************************************/
/****************************************/

State& State::getState(uint32_t un_state) {
   return m_pcRobotScheduler->getState(un_state);
}

/****************************************/
/****************************************/

//UNDERSTANDING: When we add states in addState(state), the state is assigned to a scheduler. 
void State::setRobotScheduler(RobotScheduler& c_robot_scheduler) {
   m_pcRobotScheduler = &c_robot_scheduler;
}

/****************************************/
/****************************************/

RobotScheduler::~RobotScheduler() {
   std::for_each(
      m_mapStates.begin(),
      m_mapStates.end(),
      [](std::pair<uint32_t, State*> c_item){
         delete c_item.second;
      });
}

/****************************************/
/****************************************/

//UNDERSTANDING: Adding states to an unordered map (map of states declared in the header).

void RobotScheduler::addState(State* pc_state) {
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

State& RobotScheduler::getState(uint32_t un_id) {
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

void RobotScheduler::setInitialState(uint32_t un_state) {
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

void RobotScheduler::step() {
   /* Only execute if 'current' was initialized */
   if(m_pcCurrent) {
      /* Attempt a transition, every state of every rover has its own transition() */
      State* cNewState = &m_pcCurrent->transition();
      if (interrupt)
      {
         cNewState = &getState(SCOUT_UNDOCK);
      }

      if(cNewState != m_pcCurrent) {
         /* Perform transition */
         m_pcCurrent->exitPoint();
         cNewState->entryPoint();
         m_pcCurrent = cNewState;
      }
      /* Execute current state */
      m_pcCurrent->step();
   }
   else {
      throw StateMachineException("The Robotscheduler has not been initialized, you must call SetInitialState()");
   }
}

/****************************************/
/****************************************/

//UNDERSTANDING: Each robot has its own done() and this is what is checked to perform step()
void RobotScheduler::exec() {
   while(!done()) step();
}

/****************************************/
/****************************************/
