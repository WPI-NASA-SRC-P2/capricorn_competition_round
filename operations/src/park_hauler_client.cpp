#include <operations/ParkHaulerAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionClient<operations::ParkHaulerAction> Client;

int main(int argc, char** argv)
{
    std::string robot_name = argv[1];
    ros::init(argc, argv, robot_name + COMMON_NAMES::PARK_HAULER_HOPPER_CLIENT_NODE_NAME);
    Client client(robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB_NAME, true); // true -> don't need ros::spin()
    client.waitForServer();

    operations::ParkHaulerGoal goal; 
    goal.hopper_or_excavator = argv[2]; 

    client.sendGoal(goal);
    //client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Reached goal");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
