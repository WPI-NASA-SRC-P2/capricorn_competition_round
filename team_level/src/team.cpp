#include <team_level/team.h>

Team::Team(ros::NodeHandle nh, TEAM_MACRO_STATE team_state, ROBOTS_ENUM scout, 
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
}
