#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>

#include <operations/NavigationVisionAction.h>

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

typedef actionlib::SimpleActionServer<operations::NavigationVisionAction> Server;

operations::NavigationGoal nav_goal;
perception::ObjectArray objects;

std::string desired_label;
bool centered = false;
bool execute_called = false;
int height_threshold = 400;
const int angle_threshold_narrow = 10;
const int angle_threshold_wide = 30;
const float width = 640.0;

void set_desired_label_height_threshold()
{
    if(desired_label == "hopper")
    {
        height_threshold = 250;
    }
    if(desired_label == "excavator")
    {
        height_threshold = 240;
    }
    else
    {
        height_threshold = 400;
    }    
}

void objects_callback(const perception::ObjectArray& objs) 
{
    if(!execute_called)
    {
        return;
    }
    
    objects = objs;
}

void vision_navigation()
{

    bool obj_detected;
    nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;

    // Initialize location and size variables
    float center_obj = -1;
    float height_obj = -1;

    // Initialize error, P Control, and necessary thresholds 
    static float proportional_angle = 0.0010;
    //float integral_angle = 0.0000001;
    //float derivative_angle = 0.001;
    static float error_angle;

    float error_height;

    static float prev_angular_velocity;
    static bool prev_centered;

    // Find the desired object
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        if(objects.obj[i].label == desired_label) {
            // Store the object's center and height
            center_obj = objects.obj[i].center.x;
            height_obj = objects.obj[i].size_y;
        }
    }
    
    if(center_obj == -1)
    {
        obj_detected = false;
        nav_goal.angular_velocity = 0.35;
        nav_goal.forward_velocity = 0;
    }
    else
    {
        obj_detected = true;
        error_angle = (width / 2.0) - center_obj;
        error_height = height_threshold - height_obj;
    
      if (abs(error_angle) > angle_threshold_wide)
      {
         centered = false;
      }
      
      if(centered || abs(error_angle) < angle_threshold_narrow)
      {
          centered = true;
          nav_goal.angular_velocity = 0;
          if(error_height <= 0)
          {
                // If the object is big enough, stop the robot
            nav_goal.forward_velocity = 0.0000001;
            execute_called = false;
            client->sendGoal(nav_goal);
            ROS_INFO("Reached Goal");
            return;
          }
          else
          {
              // Keep driving forward
              nav_goal.forward_velocity = 1.1;
          }
      }
      else
      {
          //sum_error_angle += error_angle;
          nav_goal.angular_velocity = error_angle * proportional_angle/* + sum_error_angle * integral_angle + (error_angle - prev_error_angle) * derivative_angle*/;
          nav_goal.forward_velocity = 0;

          if(nav_goal.angular_velocity < prev_angular_velocity - 0.05)
          {
              nav_goal.angular_velocity = prev_angular_velocity - 0.05;
          }
          if(nav_goal.angular_velocity > prev_angular_velocity + 0.05)
          {
              nav_goal.angular_velocity = prev_angular_velocity + 0.05;
          }
        }
    }

    if(prev_centered && !centered)
    {
        nav_goal.angular_velocity = 0;
        nav_goal.forward_velocity = 0;
    }

      //prev_error_angle = error_angle;
      prev_angular_velocity = nav_goal.angular_velocity;
      prev_centered = centered;
      ROS_INFO_STREAM("-----------------------------------------------------------");
      ROS_INFO_STREAM("Height: "<<height_obj);
      ROS_INFO_STREAM("ERROR Height: "<<error_height<<", Angle: "<<error_angle);
      ROS_INFO_STREAM("Angular velocity: "<<nav_goal.angular_velocity);
      ROS_INFO_STREAM("Forward velocity: "<<nav_goal.forward_velocity);
      ROS_INFO_STREAM("Centered? "<<centered);
}

void execute(const operations::NavigationVisionGoalConstPtr& goal, Server* as)
{
    execute_called = true;
    ROS_INFO("Goal Received");
    desired_label = goal->desired_object_label;
    set_desired_label_height_threshold();
    
    while (ros::ok())
    {
        vision_navigation();
        ros::Duration(0.1).sleep();
        client->sendGoal(nav_goal);
        if(!execute_called)
        {
            break;
        }
    }

    as->setSucceeded();
}

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
        return -1;
    }

    std::string robot_name = argv[1];
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_SERVER_NODE_NAME);
    ros::NodeHandle nh;

    client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);

    Server server(nh, robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ROS_INFO("Starting Navigation Vision Server");
    ros::spin();
    return 0;
}
