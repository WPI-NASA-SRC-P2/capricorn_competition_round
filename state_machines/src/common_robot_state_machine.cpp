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

void RobotScheduler::setInitialState(uint32_t un_state) {
   auto pcState = m_mapStates.find(un_state);
   if(pcState != m_mapStates.end()) {
      m_pcCurrent = pcState->second;
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
      /* Attempt a transition */
      State& cNewState = m_pcCurrent->transition();
      if(&cNewState != m_pcCurrent) {
         /* Perform transition */
         m_pcCurrent->exitPoint();
         cNewState.entryPoint();
         m_pcCurrent = &cNewState;
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

void RobotScheduler::exec() {
   while(!done()) step();
}

/****************************************/
/****************************************/
