#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


int main(int argc, char **argv)
{
    //Initializing the Action Client node
    ros::init(argc, argv, "template_actionClient");

    //Create the Action Client
    actionlib::SimpleActionClient<templates_actionlib::templateAction> action_client_("template_actionClient", true);

}