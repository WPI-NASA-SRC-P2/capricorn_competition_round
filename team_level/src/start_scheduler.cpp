#include <team_level/scheduler.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sceduler");
    ros::NodeHandle nh;

    Scheduler* sm = new Scheduler(nh, 1);

    sm->startScheduler();

    ROS_INFO("[SCHEDULER | start_scheduler.cpp | scheduler]: Hauler state machine died!\n");
}