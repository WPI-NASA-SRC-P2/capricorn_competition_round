#include "utils/battery_level_server.h"
#include "utils/battery_level.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "ros/ros.h"

ros::Publisher debug_softDeadlinePublisher;
ros::Publisher debug_hardDeadlinePublisher;

//Setting the node's update rate
#define UPDATE_HZ 10

void BatteryLevelServer::deadlinesCallback(utils::battery_deadlines::Request &req, utils::battery_deadlines::Response &res)
{

  //assign current location based on request from the client 
  current_location_ = req.current_location;

  //Hard Coded Repair Station Location from the simulation
  target_location_.pose.position.x = -6.0;
  target_location_.pose.position.y = -6.0;
  target_location_.pose.position.z = 0.65;
  target_location_.pose.orientation.x = 0;
  target_location_.pose.orientation.y = 0;
  target_location_.pose.orientation.z = 0;
  target_location_.pose.orientation.w = 1;

  //calculate the distance between the robot's current location and charging station
  distance_ = battery_level::calc_distance(target_location_.pose.position.x, target_location_.pose.position.y, current_location_.pose.position.x, current_location_.pose.position.y);

  std_msgs::Float64 softDeadline;
  std_msgs::Float64 hardDeadline;

  //Calculate the deadlines and assign them to their respective publishers
  float percentage_needed = battery_level::base_battery_level(battery_level::discharge_rate, distance_, battery_level::speed);
  hard_deadline_ = battery_level::calc_hard_deadline(percentage_needed);
  soft_deadline_ = battery_level::calc_soft_deadline(percentage_needed);

  softDeadline.data = soft_deadline_;
  hardDeadline.data = hard_deadline_;


  // publish the soft and hard deadlines to a topic
  debug_softDeadlinePublisher.publish(softDeadline);
  debug_hardDeadlinePublisher.publish(hardDeadline);

  //set the service response
  res.soft_deadline_ = soft_deadline_;
  res.hard_deadline_ = hard_deadline_;

  //return true;
}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "battery_level_server");

  //takes the firt arg and sets it equal to  robot_name
  std::string robot_name_(argv[1]); 

  //Server name
  std::string server_name_ = "/capricorn/" + robot_name_ + "/deadlines_server";

  //create a nodehandle
  ros::NodeHandle nh;

  BatteryLevelServer server;

  //declare the publishers for soft and hard deadlines
  debug_softDeadlinePublisher = nh.advertise<std_msgs::Float64>("/galaga/softDeadline", 1000);
  debug_hardDeadlinePublisher = nh.advertise<std_msgs::Float64>("/galaga/hardDeadline", 1000);

  //Instantiating ROS server for calculating deadlines
  ros::ServiceServer service = nh.advertiseService(server_name_, &BatteryLevelServer::deadlinesCallback, &server);
  ros::spin();

  return 0;
}
