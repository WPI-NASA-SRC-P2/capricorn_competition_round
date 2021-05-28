#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <templates_actionlib/template_actionMSG.h>

int main(int argc, char **argv)
{
    //Initializing the Action Client node
    ros::init(argc, argv, "template_actionClient");

    //Create the Action Client
    actionlib::SimpleActionClient<templates_actionlib::templateAction> action_client_("template_actionClient", true);

    //Wait for the action server to start
    ROS_INFO("Waiting for the action server to start.");

    //will wait for infinite time
    action_client_.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    //Goal Message is created
    template_actionlib::template_actionMSGGoal goal;

    //goal message set and sent to the action server
    goal.x = 20;
    goal.y = 10;
    goal.z = 0;
    action_client_.sendGoal(goal)

    //wait for the action to return
    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = action_client_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}
}