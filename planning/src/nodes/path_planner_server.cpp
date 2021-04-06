#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "path_planner.h"
#include "planning/trajectory.h"
#include "planning/TrajectoryWithVelocities.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <astar.h>
#include <cspace.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

class trajectoryMethods {

public:
    trajectoryMethods() {}

    bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
    {
        //res.trajectory = trajectoryGenerator(req.targetPose);
        ROS_INFO("Entered trajectoryGeneration");
        planning::TrajectoryWithVelocities trajectory;
        res.trajectory  = trajectory;
        return true;
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_planner_server");
    ros::NodeHandle nh;

    trajectoryMethods methods;

    ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &trajectoryMethods::trajectoryGeneration, &methods);
    ros::spin();

    ros::Duration(10).sleep();

    return 0;
}
