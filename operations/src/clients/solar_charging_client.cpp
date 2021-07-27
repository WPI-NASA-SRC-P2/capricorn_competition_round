#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <operations/SolarModeAction.h>
#include <boost/lexical_cast.hpp>


// defining client for ParkRobot ActionLib
typedef actionlib::SimpleActionClient<operations::SolarModeAction> g_client;
g_client *client;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and the target location (should be hopper or excavator) passed as first and second command line arguments!");
        return -1;
    }

    bool solar_status(argv[2]);
    bool solar_status_;
    solar_status_ = solar_status;

    std::string robot_name(argv[1]);
    std::string robot_name_ = robot_name;
    ros::init(argc, argv, robot_name + COMMON_NAMES::SOLAR_CHARGING_CLIENT_NODE_NAME);

    // initializing the client
    client = new g_client("/capricorn/" + robot_name_ + "/" + robot_name_ + "_solar_mode_server", true);


    ROS_INFO("Waiting for server");
    // wait for the server to run
    client -> waitForServer();
    ROS_INFO("found server");

    // defining goal
    operations::SolarModeGoal goal;

    // the second argument given would be the target object for parking, options: hopper or excavator
    goal.solar_charge_status = solar_status_;

    client->sendGoal(goal);
    ROS_INFO("Goal Sent");

    if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Reached goal");

    ROS_INFO("Current State: %s\n", client->getState().toString().c_str());
    return 0;
}
