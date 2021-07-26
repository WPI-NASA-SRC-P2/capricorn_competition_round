#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

// defining client for ParkRobot ActionLib
typedef actionlib::SimpleActionClient<operations::SolarModeAction> g_client;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and the target location (should be hopper or excavator) passed as first and second command line arguments!");
        return -1;
    }

    std::string solar_status(argv[2]);


    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::SOLAR_CHARGING_CLIENT_NODE_NAME);

    // initializing the client
    g_client client(robot_name + COMMON_NAMES::SOLAR_RECHARGE_ACTIONLIB, true);

    // wait for the server to run
    client.waitForServer();

    // defining goal
    operations::SolarModeGoal goal;

    // the second argument given would be the target object for parking, options: hopper or excavator
    goal.solar_charge_status = solar_status;

    client.sendGoal(goal);

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Reached goal");

    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
