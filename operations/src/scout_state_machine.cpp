#include<operations/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    navigation_client_ = new NavigationClient_(NAVIGATION_ACTIONLIB, true);
    resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
}

ScoutStateMachine::~ScoutStateMachine()
{
    delete navigation_client_;
    delete resource_localiser_client_;
}

void ScoutStateMachine::startStateMachine()
{
    navigation_client_->waitForServer();
    while (ros::ok() && state_machine_continue_)
    {
        switch (robot_state_)
        {
        case LOCATOR_STATES::INIT:
            // Call funtion to go to 'unexplored' region
            // For the demo purpose, call goToGoal(20,0) #random
            break;
        case LOCATOR_STATES::SEARCH:
            // Call a function to execute search algorithm
            // For the demo, spiral motion
            break;
        case LOCATOR_STATES::LOCATE:
            // Call actionlib for resource_locate
            // ResourceLocator
            break;
        case LOCATOR_STATES::FOUND:
            // Stop the robot, brake.
            // Wait for a flag from excavator that it is ready to take over
            break;
        case LOCATOR_STATES::MOVE_OUT:
            // Once flag is received, move some distance aside. 
            break;
        case LOCATOR_STATES::RECHARGE:
            // Mahimana's method to go back to the recharge station
            // Find and get to the recharge station
            break;
        case LOCATOR_STATES::WAIT_FOR_STATE_UPDATE:
            // This means there is nothing to do. 
            // Just hang around for any update
            break;

        default:
            ROS_ERROR_STREAM(robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        // sleep for some time

        // call function to update the states (Because most of these things are actionlibs
        // So they will be running in a seperate node)
        // This function should keep track of the executed actionlibs, and their current states
        // If succeeded
    }
}