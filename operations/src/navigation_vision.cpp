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

void objects_callback(const perception::ObjectArray& objects, std::string desired_object_label) 
{  
    
    bool obj_detected;
    
    // Create a goal object and turn on manual driving
    operations::NavigationGoal goal;
    goal.manual_driving = true;

    // Initialize location and size variables
    float center_obj = -1;
    float height_obj = -1;
        
    // Find the desired object
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        if(objects.obj[i].label == desired_object_label) {
            // Store the object's center and height
            center_obj = objects.obj[i].center.x;
            height_obj = objects.obj[i].size_y;
        }
    }
        
    // Initialize error, P Control, and necessary thresholds 
    int proportional_angle = 15;
    float error_angle;
    int error_angle_threshold = 7;
        
    int proportional_height = 15;
    float error_height;
    int height_threshold = 400;
    int error_height_threshold = 7;

    if(center_obj == -1)
    {
        obj_detected = false;
        return;
    }
    else
    {
        obj_detected = true;
        error_angle = width - center_obj;
        error_height = height_threshold - height_obj;
    }

    if(error_angle < error_angle_threshold)
    {
        // If object is centered, stop turning.
        goal.angular_velocity = 0;

        if(error_height < error_height_threshold)
        {
            // If the object is big enough, stop the robot
            goal.forward_velocity = 0;
        }
        else
        {
            // Keep driving forward
            goal.forward_velocity = error_height * proportional_height;
        }

    }
    else
    {
        // If object is not centered, turn.
        goal.angular_velocity = error_angle / width * 3.14 / proportional;
        goal.forward_velocity = 0;
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
