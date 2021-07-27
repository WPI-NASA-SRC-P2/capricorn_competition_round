#include <team_level/team_manager.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sceduler");
    ros::NodeHandle nh;

    TeamManager team_scheduler(nh);

    if(argc == 1)
        team_scheduler.exec();
    
    int team_index = std::stoi(argv[1]);
    ROBOTS_ENUM scout = (ROBOTS_ENUM) std::stoi(argv[2]);
    ROBOTS_ENUM excavator = (ROBOTS_ENUM) std::stoi(argv[3]);
    ROBOTS_ENUM hauler = (ROBOTS_ENUM) std::stoi(argv[4]);
    TEAM_MACRO_STATE desired_state_of_team = (TEAM_MACRO_STATE) std::stoi(argv[5]);

    team_scheduler.DEBUG_createTeam(team_index, scout, excavator, hauler, desired_state_of_team);
    team_scheduler.exec();

    ROS_INFO("[SCHEDULER | start_scheduler.cpp | scheduler]: Hauler state machine died!\n");
}