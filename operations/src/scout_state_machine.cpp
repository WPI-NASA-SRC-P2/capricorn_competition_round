#include<operations/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    navigation_client = new NavigationClient(NAVIGATION_ACTIONLIB, true);
}

ScoutStateMachine::~ScoutStateMachine()
{
    delete navigation_client;
}

void ScoutStateMachine::startStateMachine()
{
    navigation_client->waitForServer();
    while (ros::ok() && state_machine_continue_)
    {
        switch (robot_state_)
        {
        case LOCATOR_STATES::INIT:
            // For the demo purpose, call goToGoal(20,0)
            break;
        case LOCATOR_STATES::SEARCH:
            // For the demo, spiral motion
            break;
        case LOCATOR_STATES::LOCATE:
            // ResourceLocator
            break;
        case LOCATOR_STATES::FOUND:
            // Wait for a flag from excavator that it is ready to take over
            break;
        case LOCATOR_STATES::MOVE_OUT:
            // Once flag is received, move some distance aside. 
            break;
        case LOCATOR_STATES::RECHARGE:
            // Find and get to the recharge station
            break;
        
        default:
            break;
        }
    }
}