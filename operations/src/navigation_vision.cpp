#include <ros/ros.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

float height = 480;
float width = 640;

std::string robot_name;

#define UPDATE_HZ 20

void objects_callback(const perception::ObjectArray& objects) 
{  
    operations::NavigationGoal goal;
    goal.manual_driving = true;
    
    if(execute_service)
    {
        float center_rs = -1;
        float size_rs = -1;
        
        for(int i = 0; i < objects.number_of_objects; i++) 
        {   
            if(objects.obj[i].label == "repairStation") {
                center_rs = objects.obj[i].center.x;
                size_rs = objects.obj[i].size_y;
            }
        }
        
        int proportional_angle = 15;
        float error_angle;
        int error_angle_threshold = 7;
        
        int proportional_size = 15;
        float error_size;
        int size_threshold = 400;
        int error_size_threshold = 7;
        
        if(center_rs == -1)
        {
            rs_detected = false;
            return;
        }
        else
        {
            rs_detected = true;
            error_angle = width - center_rs;
            error_size = size_threshold - size_rs;
        }
        
        if(error_angle < error_angle_threshold)
        {
            goal.angular_velocity = 0;
            
            if(error_size < error_size_threshold)
            {
                goal.forward_velocity = error_size * proportional_size;
            }
            else
            {
                goal.forward_velocity = 0;
            }
            
        }
        else
        {
            goal.angular_velocity = error_angle / width * 3.14 / proportional;
            goal.forward_velocity = 0;
        }
    }
    
    client->sendGoal(goal);
    
}

int main(int argc, char** argv)
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_NODE_NAME);
    ros::NodeHandle nh;
    
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);
  
    client = new Client(NAVIGATION_ACTIONLIB, true);
    client->waitForServer();

    ros::spin();
    return 0;
}
