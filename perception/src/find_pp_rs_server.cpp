/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <perception/Find_PP_RSAction.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>

ros::Publisher yaw_sensor_pub;
float height = 480;
float width = 640;

std::string robot_name;

#define UPDATE_HZ 20
std_msgs::Float64 yaw_msg;
bool pp_rs_detected = false;
bool execute_service = false;
int times = 0;

void objects_callback(const perception::ObjectArray& objects) 
{   
    if(execute_service)
    {
        float center_rs = -1, center_pp = -1;

        for(int i = 0; i < objects.number_of_objects; i++) 
        {   
            if(objects.obj[i].label == "processingPlant")
            {
                center_pp = objects.obj[i].center.x;
            }
            else if(objects.obj[i].label == "repairStation") {
                center_rs = objects.obj[i].center.x;
            }
        }
        
        int proportional = 15;
        float error;

        if(center_rs == -1 && center_pp == -1)
        {
            pp_rs_detected = false;
            return;
        }
        else if(center_rs > -1 && center_pp > -1) 
        {
            pp_rs_detected = true;
            error = ((width / 2) -(center_rs + (center_pp - center_rs) / 2));
        }
        else 
        {
            pp_rs_detected = true;
            int c = (center_pp > -1) ? center_pp : center_rs;
            error = ((width / 2) - c);
        }

        if(error < 7 && error > -7) 
        {
            times++;
        }
        else
        {
            times = 0;
        }

        yaw_msg.data = yaw_msg.data +  error / width * 3.14 / proportional;
        

        if(yaw_msg.data > 3.14) 
        {
            yaw_msg.data = 3.14;
        }
        else if(yaw_msg.data < -3.14)
        {
            yaw_msg.data = -3.14;
        }

        yaw_sensor_pub.publish(yaw_msg);
    }
}

typedef actionlib::SimpleActionServer<perception::Find_PP_RSAction> Server;

void execute(const perception::Find_PP_RSGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
    execute_service = true;
    yaw_sensor_pub.publish(yaw_msg);
    int multi = 1;
    times = 0;
    
    ros::Rate update_rate(UPDATE_HZ);
    
    while (ros::ok())
    {
        if(times > 10)
        {
            ROS_INFO("SUCCEEDED");
            as->setSucceeded();
            execute_service = false;
            return;
        }
        ros::spinOnce();       
        if(!pp_rs_detected) 
        {
            yaw_msg.data = yaw_msg.data + multi * 0.04;
            yaw_sensor_pub.publish(yaw_msg);

            if(yaw_msg.data < -4) 
            {
                break;
            }

            else if(yaw_msg.data > 4) 
            {
                multi = -1;
                ros::Duration(1).sleep();
            }
        }     
        update_rate.sleep();   
    }
    ROS_INFO("No processing plant or repair station found - SERVER");

    as->setAborted();
    execute_service = false;
}

int main(int argc, char** argv)
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, robot_name + COMMON_NAMES::FIND_PP_RS_SERVER_NODE_NAME);
    ros::NodeHandle nh;
    
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);
    
    yaw_sensor_pub = nh.advertise<std_msgs::Float64>("/" + robot_name + COMMON_NAMES::SET_SENSOR_YAW_TOPIC, 10);
    yaw_msg.data = 0;

    Server server(nh, robot_name + COMMON_NAMES::FIND_PP_RS_ACTIONLIB_NAME, boost::bind(&execute, _1, &server), false);
    server.start();
    std::cout<<"Started Server\n";
    ros::spin();
    return 0;
}