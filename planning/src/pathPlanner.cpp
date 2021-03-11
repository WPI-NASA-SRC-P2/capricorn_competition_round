
#include <ros/ros.h>
#include <templates/template_class.h>
#include <nav_msgs/OccupancyGrid.h>

int grid_width = 10;
int grid_height = 10;

PathPlanner::PathPlanner())
{

}

PathPlanner::~PathPlanner()
{

}

std::string PathPlanner::getTeamName()
{
    // returns the private variable 
    return team_name;
}

void PathPlanner::setTeamName(const std::string& input_string)
{
    // team_name is assigned the value of Input string
    // Doing this, you keep the private variable safe 
    team_name = input_string;
}

/**
 * @brief converts index of a grid into x and y coordinates
 * 
 */
void PathPlanner::indexToGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int index_x = msg.info.width % grid_width + 1;
    int index_y = msg.info.height / grid_width;

    ROS_INFO("X: [%s], Y: [%s]", index_x, index_y);

}

/**
 * @brief converts x and y coordinates of a grid cell to its index
 * 
 */
void PathPlanner::gridToIndex(nav_msg msg)
{
    
    
    //nav_msgs/OccupancyGrip.msg.info.width;
    //int32 height = msgs.OccupancyGrip.msg.info.height;
    //int32 width =  msg.OccupancyGrip.msg.info.width;
    //int8[] data = msg.OccupancyGrip.msg.data;
    //return (x,y);
}

/**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
void PathPlanner::neightborsOf4(void)
{
    //for each index value less than the max
    //check to each side to see if there is an obstacle
    //also check if the neighbor is within the bounds (for edge cases)
    //return list of coordinates 
}

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
void PathPlanner::neightborsOf8(void)
{
    //for each index value less than the max
    //check to each side to see if there is an obstacle
    //check diagonals 
    //also check if the neighbor is within the bounds (for edge cases)
    //return list of coordinates 
}

/**
 * @brief derives a path from a list of coordinates 
 * 
 */
void wavefront(void)
{

}

/**
 * @brief main function for quick-testing of the class
 * 
 */
int main(int argc, char *argv[])
{
    // ROS initialization
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    // Creted an object of the class
    PathPlanner planner;

    //Subscribe to the node publishing map values 
    ros::Subscriber indexValue = nh.subscribe("/small_scout1/camera/grid_map", 1000, indexToGrid;

    ros::spin();

    return 0;
}