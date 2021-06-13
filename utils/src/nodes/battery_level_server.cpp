#include "battery_level_server.h"

#include <cspace.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>


float distance = 0;
float current_location = 0;

//Setting the node's update rate
#define UPDATE_HZ 10

void BatteryLevelServer::poseCallback(geometry_msgs::PoseStamped pose){

    // calculate the distance between the robot's current location and charging station
    distance = calc_distance(pose, current_location);         //needs to be calculated using a distance formula
}

void BatteryLevelServer::deadlinesCalculator(){
    // publish the soft and hard deadlines to a topic
    
    debug_deadlinesPublisher.pub(); //publish the dealines
}


int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "battery_level_server");

  std::string robot_name(argv[1]); // takes the firt arg and sets it equal to  robot_name

  std::string robot_name_ = robot_name;

  //ROS Topic names
  std::string pose_topic_ = "/capricorn/" + robot_name_ + "/object_detection_map";

  //create a nodehandle
  ros::NodeHandle nh;

  BatteryLevelServer server;

  server.pose_subscriber = nh.subscribe(pose_topic_, 1000, &BatteryLevelServer::poseCallback, &server);

  debug_deadlinesPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/galaga/batteryLevelDeadlines", 1000);


  //Instantiating ROS server for calculating deadlines
  ros::ServiceServer service = nh.advertiseService("deadlines_server", &BatteryLevelServer::deadlinesCalculator, &server);

  ros::spin();

  return 0;
}
