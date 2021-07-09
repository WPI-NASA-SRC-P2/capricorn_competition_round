#include "dyn_planning.h"    
#include "nav_msgs/Path.h"    
#include "astar.h"
#include <math.h>
using namespace std;


float DynamicPlanning::lengthOfPath(nav_msgs::Path path)
{   
    float pathLength = 0;

    for (int i = 0; i < path.poses.size() - 1; i++)
    {
        geometry_msgs::Point first_point = path.poses[i].pose.position;
        geometry_msgs::Point second_point = path.poses[i+1].pose.position;

        float length = sqrt(pow(second_point.x - first_point.x, 2) + pow(second_point.y - first_point.y, 2) * 1.0);
        pathLength += length;
    }
    
    return pathLength;
}


double DynamicPlanning::distance(geometry_msgs::Point firstpoint, geometry_msgs::Point secondpoint)
{
    return sqrt(pow(secondpoint.x - firstpoint.x, 2) + pow(secondpoint.y - firstpoint.y, 2) * 1.0);
}




 std::pair <geometry_msgs::Point, geometry_msgs::Point> DynamicPlanning::waypointOnPath(geometry_msgs::Point upcoming_point, geometry_msgs::PoseStamped robot_pose, nav_msgs::Path &path, float distance)
{
    geometry_msgs::Point pointOfInterest;
    float trackDistance1 = 0; 
    std::pair <geometry_msgs::Point, geometry_msgs::Point> waypoints;

        for (int i = 0; i < path.poses.size() - 2; i++)
        {
        geometry_msgs::Point first_point = robot_pose.pose.position;          //robot position
        geometry_msgs::Point second_point = upcoming_point;       //upcoming point
                
        trackDistance1 = sqrt(pow(second_point.x - first_point.x, 2) + pow(second_point.y - first_point.y, 2) * 1.0);    
              

        if(trackDistance1 > 6)
        {
            /*find the equation of the line using two points
            then find the point which is at 6 m from the initial point */
            double a, b , c;

            a = second_point.y - first_point.y;
            b = second_point.x - first_point.x;
           
            float m1, c1;
            m1 = tan(a/b);
            c1 = first_point.y - m1*first_point.x;
            
            geometry_msgs::Point new_point_1;
            geometry_msgs::Point new_point_2;

            new_point_1.x = first_point.x + (6*(sqrt(1/(1 + pow(m1,2)))));
            new_point_1.y = first_point.y + (m1*6*(sqrt(1/(1 + pow(m1,2)))));
            
            new_point_2.x = first_point.x - (6*(sqrt(1/(1 + pow(m1,2)))));
            new_point_2.y = first_point.y - (m1*6*(sqrt(1/(1 + pow(m1,2)))));
            

            double distance_1 = DynamicPlanning::distance(second_point, new_point_1);
            double distance_2 = DynamicPlanning::distance(second_point, new_point_2);


            if(distance_1 < distance_2)
            {  
                waypoints.first = first_point;   
                waypoints.second = new_point_1;   
                return waypoints; 
            }
            else
            {
                waypoints.first = first_point;   
                waypoints.second = new_point_2;   
                return waypoints;
            }
        }  
    }

}


/* This function divides the 6m line in 5 equal parts, 
and gives out the coordinates of each Interpolated point. */

std::vector<geometry_msgs::Point> interpolation_poses(std::pair<geometry_msgs::Point, geometry_msgs::Point> waypoints)
{   
   std::vector<geometry_msgs::Point> Intr_poses;

   geometry_msgs::Point A = waypoints.first; // first point of the line 
   geometry_msgs::Point B = waypoints.second; // last point of the line 

   for(int k = 1; k < 6; k++)
    {
        {   
            geometry_msgs::Point Intr;    
            Intr.x  = A.x*(1-(1/k)) + B.x*(1/k);    
            Intr.y  = A.y*(1-(1/k)) + B.y*(1/k);
            Intr_poses.push_back(Intr);   // interpolated points
        } 
    }

   return Intr_poses;
}

bool DynamicPlanning::checkForObstacles(nav_msgs::Path& path, nav_msgs::OccupancyGrid& oGrid, geometry_msgs::PoseStamped robot_pose) // path is giving the grid coordinates
{
    for(int i = 0; i < path.poses.size()-1 ; i++)
    {
        geometry_msgs::Point upcoming_point;
        upcoming_point = path.poses[i+1].pose.position; 

        while(upcoming_point.x == robot_pose.pose.position.x  && upcoming_point.y == robot_pose.pose.position.y)
        {

        std::pair<geometry_msgs::Point, geometry_msgs::Point> new_waypoints;
        new_waypoints = waypointOnPath(upcoming_point,robot_pose, path, 6.0);

        std::vector<geometry_msgs::Point> new_Intr_poses;
        new_Intr_poses = interpolation_poses(new_waypoints);

        for(int j = 0; j < new_Intr_poses.size() ; j++)
        {
            if(oGrid.data[AStar::indexFromPoseStamped(new_Intr_poses[j], oGrid)] > 50)  
            {                                                                                 
                ROS_INFO("Obstacle on the current path");
                return true; // trigger on !!!!
            }
            else
            {
                return false;
            }
        }
        }
    }

}
