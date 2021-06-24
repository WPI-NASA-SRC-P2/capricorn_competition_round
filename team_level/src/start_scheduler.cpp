#include <team_level/team_scheduler.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sceduler");
    ros::NodeHandle nh;

    TeamScheduler team_scheduler(nh);

    team_scheduler.exec();

    ROS_INFO("[SCHEDULER | start_scheduler.cpp | scheduler]: Hauler state machine died!\n");
}