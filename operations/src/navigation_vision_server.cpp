#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>

#include <operations/NavigationVisionAction.h>
#include <actionlib/server/simple_action_server.h>

// typedef for the Action Server
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

typedef actionlib::SimpleActionServer<operations::NavigationVisionAction> Server;

using namespace COMMON_NAMES;

std::string robot_name;
std::string desired_label;
//float sum_error_angle = 0.0;
//float prev_error_angle = 0.0;
bool centered = false;
operations::NavigationGoal nav_goal;

void objects_callback(const perception::ObjectArray& objects) 
{
  ROS_INFO("Working callback");

  bool obj_detected;

  // Create a nav_goal object and turn on manual driving
  nav_goal.drive_mode = NAV_TYPE::MANUAL;

  // Initialize location and size variables
  float center_obj = -1;
  float height_obj = -1;
  float width = 640.0;

  // Initialize error, P Control, and necessary thresholds 
  static float proportional_angle = 0.0005;
  //float integral_angle = 0.0000001;
  //float derivative_angle = 0.001;
  static float error_angle;
  static int angle_threshold_narrow = 15;
  static int angle_threshold_wide = 40;

  float error_height;
  int height_threshold = 300;

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
    nav_goal.angular_velocity = 0.2;
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
        nav_goal.forward_velocity = 0;
        client->sendGoal(nav_goal);
        ros::shutdown();
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

  ROS_INFO_STREAM("ERROR Height: "<<error_height<<", Angle: "<<error_angle);
  ROS_INFO_STREAM("Angular velocity: "<<nav_goal.angular_velocity);
  ROS_INFO_STREAM("Forward velocity: "<<nav_goal.forward_velocity);
  ROS_INFO_STREAM("Centered? "<<centered);

}

void execute(const operations::NavigationVisionGoalConstPtr& goal, Server* as)
{
  desired_label = goal->desired_object_label;
  //DO STUFF
  while (ros::ok())
  {
    ros::spinOnce();
    client->sendGoal(nav_goal);
    // update_rate.sleep();
  }
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_vision_server");
  ros::NodeHandle n;

  std::string name(argv[1]);
  robot_name = name;

  ros::Subscriber objects_sub = n.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);

  Server server(n, "navigation_vision", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}