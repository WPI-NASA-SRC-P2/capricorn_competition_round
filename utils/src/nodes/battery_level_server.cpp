#include "utils/battery_level_server.h"
#include "utils/battery_level.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Float64MultiArray.h>

ros::Publisher debug_deadlinesPublisher;

//Setting the node's update rate
#define UPDATE_HZ 10

void BatteryLevelServer::poseCallback(nav_msgs::Odometry odom){

    geometry_msgs::Pose current_location_ = odom.pose.pose;
    
    geometry_msgs::PoseStamped target_location_;
    target_location_.pose.position.x = -6.0;
    target_location_.pose.position.y = -6.0;  
    target_location_.pose.position.z = 0.65;
    target_location_.pose.orientation.x = 0;
    target_location_.pose.orientation.y = 0;
    target_location_.pose.orientation.z = 0;
    target_location_.pose.orientation.w = 1;

    // calculate the distance between the robot's current location and charging station
    distance_ = battery_level::calc_distance(target_location_.pose.position.x, target_location_.pose.position.y, current_location_.position.x, current_location_.position.y);       //needs to be calculated using a distance formula
}

void BatteryLevelServer::deadlinesCallback(){
    std_msgs::Float64MultiArray deadlines;

    // publish the soft and hard deadlines to a topic
    float percentage_needed = battery_level::base_battery_level(battery_level::discharge_rate, distance_, battery_level::speed);
    soft_deadline_ = battery_level::calc_soft_deadline(percentage_needed);
    hard_deadline_ = battery_level::calc_hard_deadline(percentage_needed);
    

   deadlines.layout.dim[0].label  = "height";
   deadlines.layout.dim[0].size   = 2;
   deadlines.layout.dim[1].label  = "width";
   deadlines.layout.dim[1].size   = 1;
   deadlines.data[0] = soft_deadline_;
   deadlines.data[1] = hard_deadline_;
    //publish the dealines
  debug_deadlinesPublisher.publish(deadlines); 
}


int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "battery_level_server");

  std::string robot_name(argv[1]); // takes the firt arg and sets it equal to  robot_name

  std::string robot_name_ = robot_name;

  //ROS Topic for getting the curren location of the robot
  std::string pose_topic_ = "/" + robot_name_ + "/camera/odom";

  //create a nodehandle
  ros::NodeHandle nh;

  BatteryLevelServer server;

  server.pose_subscriber = nh.subscribe(pose_topic_, 1000, &BatteryLevelServer::poseCallback, &server);

  debug_deadlinesPublisher = nh.advertise<std_msgs::Float64MultiArray>("/galaga/batteryLevelDeadlines", 1000);

  //Instantiating ROS server for calculating deadlines
  ros::ServiceServer service = nh.advertiseService("deadlines_server", &BatteryLevelServer::deadlinesCalculator, &server);
  ros::spin();

  return 0;
}
