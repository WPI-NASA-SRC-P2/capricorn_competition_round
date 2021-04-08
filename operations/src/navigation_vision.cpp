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
float sum_error_angle = 0.0;
float prev_error_angle = 0.0;
float sum_error_height = 0.0;
float prev_error_height = 0.0;

void objects_callback(const perception::ObjectArray& objects) 
{
  ROS_INFO("Working callback");

  bool obj_detected;

  // Create a goal object and turn on manual driving
  operations::NavigationGoal goal;
  goal.drive_mode = NAV_TYPE::MANUAL;

  // Initialize location and size variables
  float center_obj = -1;
  float height_obj = -1;
  float width = 640.0;

  // Find the desired object
  for(int i = 0; i < objects.number_of_objects; i++) 
  {   
    if(objects.obj[i].label == "repairStation") {
      // Store the object's center and height
      center_obj = objects.obj[i].center.x;
      height_obj = objects.obj[i].size_y;
    }
  }

  // Initialize error, P Control, and necessary thresholds 
  float proportional_angle = 0.001;
  float integral_angle = 0.0000001;
  float derivative_angle = 0.001;
  float error_angle;
  int error_angle_threshold = 40;

  float proportional_height = 0.02;
  float integral_height = 0.0;
  float derivative_height = 0.0;
  float error_height;
  int height_threshold = 300;
  int error_height_threshold = 10;

  if(center_obj == -1)
  {
    obj_detected = false;
    goal.angular_velocity = 0.3;
    goal.forward_velocity = 0;
  }
  else
  {
    obj_detected = true;
    error_angle = (width / 2.0) - center_obj;
    error_height = height_threshold - height_obj;
  

    if(abs(error_angle) < error_angle_threshold)
    {
      // If object is centered, stop turning.
      goal.angular_velocity = 0;

      if(error_height < error_height_threshold)
      {
          // If the object is big enough, stop the robot
          goal.forward_velocity = 0;
          client->sendGoal(goal);
          ros::shutdown();
      }
      else
      {
          // Keep driving forward
          sum_error_height += error_height;
          goal.forward_velocity = error_height * proportional_height + sum_error_height * integral_height + (error_height - prev_error_height) * derivative_height;
      }
    }
    else
    {
      // If object is not centered, turn.
      sum_error_angle += error_angle;
      goal.angular_velocity = error_angle * proportional_angle + sum_error_angle * integral_angle + (error_angle - prev_error_angle) * derivative_angle;
      goal.forward_velocity = 0;
    }
  }

  prev_error_angle = error_angle;
  prev_error_height = error_height;

  ROS_INFO_STREAM("ERROR Height: "<<error_height<<", Angle: "<<error_angle);
  ROS_INFO_STREAM("Angular velocity: "<<goal.angular_velocity);
  client->sendGoal(goal);

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

    ros::spin();
    return 0;
  }
}