#include "utils/battery_level_server.h"
#include "utils/battery_level.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>


//Setting the node's update rate
#define UPDATE_HZ 10

void BatteryLevelServer::poseCallback(nav_msgs::Odometry odom){

    current_location_ = odom.pose.pose;

    // calculate the distance between the robot's current location and charging station
    distance_ = battery_level::calc_distance(target_location_.pose.position.x, target_location_.pose.position.y, current_location_.pose.position.x, current_location_.pose.position.y)        //needs to be calculated using a distance formula
}

void BatteryLevelServer::deadlinesCalculator(){
    std_msgs::Float64MultiArray deadlines;

    // publish the soft and hard deadlines to a topic
    float percentage_needed = battery_level::base_battery_level(battery_level::discharge_rate, BatteryLevelServer::distance_, battery_level::speed);
    soft_deadline_ = battery_level::calc_soft_deadline(percentage_needed);
    hard_deadline_ = battery_level::calc_hard_deadline(percentage_needed);
    

   deadlines.dim[0].label  = "soft deadline";
   deadlines.dim[0].size   = 1;
   deadlines.dim[1].label  = "hard deadline";
   deadlines.dim[1].size   = 1;
   deadlines.data[0] = soft_deadline_;
   deadlines.data[1] = hard_deadline_;
    //publish the dealines
    debug_deadlinesPublisher.pub(deadlines); 
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
