#include <team_level/team.h>

Team::Team(ros::NodeHandle &nh, TEAM_MACRO_STATE team_state, ROBOTS_ENUM scout, 
         ROBOTS_ENUM excavator, ROBOTS_ENUM hauler)
{
    robot_state = new RobotStatus(nh);

    if(scout != NONE)
        setScout(scout);
    if(excavator != NONE)
        setScout(excavator);
    if(hauler != NONE)
        setScout(hauler);

    hired_scout = scout;
    hired_excavator = excavator;
    hired_hauler = hauler;

    macro_state = team_state;

    addStates(nh);
}

void Team::addStates(ros::NodeHandle &nh)
{
    addState(new Search(nh));
    addState(new Standby(nh));
}

void Team::addState(TeamState* pc_state) {
   if(MACRO_STATE_PTR_MAP.find(pc_state->getId()) == MACRO_STATE_PTR_MAP.end()) {
      MACRO_STATE_PTR_MAP[pc_state->getId()] = pc_state;
    //   pc_state->setRobotScheduler(*this);
   }
   else {
      ROS_ERROR_STREAM("Duplicated state id " << pc_state->getId());
   }
}

void Team::setInitialState(uint32_t un_state) {
   auto pcState = MACRO_STATE_PTR_MAP.find(un_state);
   // if state exists in map, then set it to the initial state of the scheduler
   if(pcState != MACRO_STATE_PTR_MAP.end()) {
      // acquire value of the state (every map has a key(first) and a value(second))
      current_state_ptr = pcState->second;
      // completes entry point of the initial state
      current_state_ptr->entryPoint();
   }
   else {
      ROS_ERROR_STREAM("Can't set initial state to " << un_state);
   }
}
