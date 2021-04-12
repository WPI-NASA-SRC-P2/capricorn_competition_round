#include <operations/NavigationVisionAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> Client;

int main(int argc, char** argv)
{
    std::string robot_name = argv[1];
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_CLIENT_NODE_NAME);
    Client client(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB_NAME, true); // true -> don't need ros::spin()
    client.waitForServer();

    operations::NavigationVisionGoal goal; 
    goal.desired_object_label = argv[2]; 

    client.sendGoal(goal);
    //client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Reached goal");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
