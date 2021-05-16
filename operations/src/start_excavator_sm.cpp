#include <ros/ros.h>
#include <operations/excavator_state_machine.h>

std::string g_robot_name;

bool checkTask(STATE_MACHINE_TASK task)
{
    return EXCAVATOR_TASKS.find(task) != EXCAVATOR_TASKS.end();
}


/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm ExcavatorStateMachine object
 */
void execute(const operations::ExcavatorStateMachineTaskGoalConstPtr& goal, SM_SERVER* as, ExcavatorStateMachine* sm)
{
    ROS_INFO_STREAM("Received "<<g_robot_name<<"  State Machine Goal: "<<goal->task);

    operations::ExcavatorStateMachineTaskResult result;
    // Waiting for the servers to start
    ROS_INFO("Waiting for navigation client");
    sm->navigation_client_->waitForServer();
    ROS_INFO("Waiting for excavator arm client");
    sm->excavator_arm_client_->waitForServer();
    ROS_INFO("Waiting for navigation vision client");
    sm->navigation_vision_client_->waitForServer(); //Not being used currently
    ROS_WARN("Servers started");

    STATE_MACHINE_TASK robot_state = (STATE_MACHINE_TASK)goal->task;

    if(!checkTask(robot_state))
    {
        // the class is not valid, send the appropriate result
        result.result = COMMON_NAMES::COMMON_RESULT::INVALID_GOAL;
        as->setAborted(result, "Invalid Task");
        ROS_ERROR_STREAM("Invalid Task - "<<g_robot_name<<" State Machine");
        return;
    }

    bool output = false;

    switch (robot_state)
    {
        case STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC:
            output = sm->goToLoc(goal->goal_loc);
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT:
            output = sm->goToScout();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB:
            output = sm->parkExcavator();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_DIG_AND_DUMP_VOLATILE:
            ROS_INFO("Digging Volatile");
            output = sm->digAndDumpVolatile();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_GOTO_DEFAULT_ARM_POSE:
            ROS_INFO("Digging Volatile");
            output = sm->goToDefaultArmPosition();
            break;
        default:
            ROS_ERROR_STREAM(sm->robot_name_ + " state machine encountered unhandled state!");
            break;
    }

    result.result = output ? COMMON_NAMES::COMMON_RESULT::SUCCESS : COMMON_NAMES::COMMON_RESULT::FAILED;
    as->setAborted(result, "Interrupted");
    ROS_INFO("EXITING LOOP");
    return;
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal(ExcavatorStateMachine* sm)
{
    ROS_INFO_STREAM("Cancelling "<<g_robot_name<<"  State Machine Goal");

    sm->navigation_vision_client_->cancelGoal();
    sm->navigation_client_->cancelGoal();
    sm->excavator_arm_client_->cancelGoal();
}

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    g_robot_name = argv[1];

    ros::init(argc, argv, g_robot_name + "_sm");
    ros::NodeHandle nh;

    ExcavatorStateMachine* sm = new ExcavatorStateMachine(nh, g_robot_name);

    SM_SERVER server(nh, g_robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server, sm), false);
	server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
    server.start();

    ROS_INFO("Started Excavator State Machine Actionlib Server");
    ros::spin();

    ROS_WARN("Excavator state machine died!\n");

    return 0;
}