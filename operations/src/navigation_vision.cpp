/* TODO:
- Tune the PID controller for turning
- Investigate rightward jerking at beginning of turn
- Convert into action library
  - Input: Label
  - Output: Progress, Done? 
*/

#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// typedef for the Action Server
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;

using namespace COMMON_NAMES;

std::string robot_name;
//float sum_error_angle = 0.0;
//float prev_error_angle = 0.0;
bool centered = false;
operations::NavigationGoal goal;


void objects_callback(const perception::ObjectArray& objects) 
{
  ROS_INFO("Working callback");

  bool obj_detected;

  // Create a goal object and turn on manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

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
    if(objects.obj[i].label == "repairStation") {
      // Store the object's center and height
      center_obj = objects.obj[i].center.x;
      height_obj = objects.obj[i].size_y;
    }
  }

  if(center_obj == -1)
  {
    obj_detected = false;
    goal.angular_velocity = 0.2;
    goal.forward_velocity = 0;
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
      goal.angular_velocity = 0;
      if(error_height <= 0)
      {
        // If the object is big enough, stop the robot
        goal.forward_velocity = 0;
        client->sendGoal(goal);
        ros::shutdown();
      }
      else
      {
        // Keep driving forward
        goal.forward_velocity = 1.1;
      }
    }
    else
    {
      //sum_error_angle += error_angle;
      goal.angular_velocity = error_angle * proportional_angle/* + sum_error_angle * integral_angle + (error_angle - prev_error_angle) * derivative_angle*/;
      goal.forward_velocity = 0;

      if(goal.angular_velocity < prev_angular_velocity - 0.05)
      {
        goal.angular_velocity = prev_angular_velocity - 0.05;
      }
      if(goal.angular_velocity > prev_angular_velocity + 0.05)
      {
        goal.angular_velocity = prev_angular_velocity + 0.05;
      }
    }
  }

  if(prev_centered && !centered)
  {
    goal.angular_velocity = 0;
    goal.forward_velocity = 0;
  }

  //prev_error_angle = error_angle;
  prev_angular_velocity = goal.angular_velocity;
  prev_centered = centered;

  ROS_INFO_STREAM("ERROR Height: "<<error_height<<", Angle: "<<error_angle);
  ROS_INFO_STREAM("Angular velocity: "<<goal.angular_velocity);
  ROS_INFO_STREAM("Centered? "<<centered);

}

int main(int argc, char** argv)
{
  // Ensure the robot name is passed in
  if (argc != 2 && argc != 4)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
      return -1;
  }
  else
  {
    // Robot Name from argument
    robot_name = argv[1];
    std::string node_name = robot_name + "_navigation_action_client";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Subscribing to teleop topic
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);
    // initialize client
    client = new Client(NAVIGATION_ACTIONLIB, true);
    printf("Waiting for server...\n");
    client->waitForServer();
    printf("Done waiting. Spinning\n");

    ros::Rate update_rate(1);

    while (ros::ok())
    {
      ros::spinOnce();
      client->sendGoal(goal);
      update_rate.sleep();
    }

    ros::spin();
    return 0;
  }
}