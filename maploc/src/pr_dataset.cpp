/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <maploc/ground_truth_pr.h>
#include <perception/Find_PP_RSAction.h>
#include <actionlib/client/simple_action_client.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>

typedef actionlib::SimpleActionClient<perception::Find_PP_RSAction> Client;

void bb_and_objects_callback(const perception::ObjectArray& objects) 
{
    maploc::PosePR pr_msg;
    pr_msg = ground_truth_2d(robot_name, ground_truth_3d(robot_name));

    float size_x_rs = INT_MAX, size_y_rs = INT_MAX, size_x_pp = INT_MAX, size_y_pp = INT_MAX;
    long area_pp = -1, area_rs = -1;

    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        if(objects.obj[i].label == "processingPlant")
        {
            size_x_pp = objects.obj[i].size_x;
            size_y_pp = objects.obj[i].size_y;
            area_pp = size_x_pp * size_y_pp;
        }
        else if(objects.obj[i].label == "repairStation") {
            size_x_rs = objects.obj[i].size_x;
            size_y_rs = objects.obj[i].size_y;
            area_rs = size_x_rs * size_y_rs;
        }
    }

    std::cout<<"Radius: "<<pr_msg.radius<<", Angle: "<<pr_msg.angle<<"\n";
    std::cout<<"Dimensions of RS X: "<<size_x_rs<<", Y: "<<size_y_rs<<"\n";
    std::cout<<"Dimensions of PP X: "<<size_x_pp<<", Y: "<<size_y_pp<<"\n";

    ros::shutdown();
}

int main(int argc, char** argv)
{
    std::string name(argv[1]);
    robot_name = name;
    ros::init(argc, argv, robot_name + COMMON_NAMES::PR_DATASET_NODE_NAME);

    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    
    client = nh.serviceClient<gazebo_msgs::GetLinkState>(COMMON_NAMES::LINK_STATE_QUERY);
    Client action_client(robot_name + COMMON_NAMES::FIND_PP_RS_ACTIONLIB_NAME, true); 
    action_client.waitForServer();

    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &bb_and_objects_callback);

    perception::Find_PP_RSGoal goal;
    goal.robot_name = robot_name;
    action_client.sendGoal(goal);
    action_client.waitForResult(ros::Duration(15.0));
    
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Succeeded to find processing plant and repair station");
        ros::spin();
    }
    else if(action_client.getState() == actionlib::SimpleClientGoalState::ABORTED || action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
        ROS_INFO("Cannot find the Processing Plant or Station");
    } 
    
    return 0;
}