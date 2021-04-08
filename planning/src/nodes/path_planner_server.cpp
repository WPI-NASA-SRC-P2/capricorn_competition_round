#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "path_planner.h"
#include "planning/trajectory.h"
#include "planning/TrajectoryWithVelocities.h"
#include "trajectory_methods.h"


//Setting the node's update rate
#define UPDATE_HZ 10


bool TrajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
    //res.trajectory = trajectoryGenerator(req.targetPose);
    ROS_INFO("Entered trajectoryGeneration");
    planning::TrajectoryWithVelocities trajectory;
    res.trajectory  = trajectory;
    return true;
}



int main(int argc, char *argv[])
{   
    //initialize node 
    ros::init(argc, argv, "path_planner_server");

    //create a nodehandle
    ros::NodeHandle nh;

    //Instantiating TrajectoryMethods object
    TrajectoryMethods methods;

    //Instantiating ROS server for generating trajectory
    ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &trajectoryMethods::TrajectoryGeneration, &methods);
    
    ros::spin();
    ros::Duration(10).sleep();

    return 0;
}
