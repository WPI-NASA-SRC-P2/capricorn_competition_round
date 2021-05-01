/*
Author BY: Ashay Aswale, Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
*/

#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>

// structure for point in cartesian plane.
struct point {
    int x, y;
};

const float WIDTH_IMAGE = 640.0;
const int RIGHT = 1, LEFT = -1, ZERO = 0;

int directionOfPoint(point A, point B, point P)
{
    // subtracting co-ordinates of point A from
    // B and P, to make A as origin

    B.x -= A.x;
    B.y -= A.y;
    P.x -= A.x;
    P.y -= A.y;
    // Determining cross Product
    int cross_product = B.x * P.y - B.y * P.x;
 
    // return RIGHT if cross product is positive
    if (cross_product > 0)
        return RIGHT;
 
    // return LEFT if cross product is negative
    if (cross_product < 0)
        return LEFT;
 
    // return ZERO if cross product is zero.
    return ZERO;
}


/**
 * @brief Checks if the obstacle is in the projected path of the robot
 * 
 * @param obj 
 * @return true : if the obstacle is in path
 * @return false : otherwise
 */
bool checkObstacle(const perception::Object& obj)
{ 
    // ROS_INFO("Rock Obstacle");
    float left_boundary = obj.center.x - (obj.size_x / 2), right_boundary = obj.center.x + (obj.size_x / 2);
    // ROS_INFO_STREAM("Left Boundary: "<<left_boundary<<", Right Boundary: "<<right_boundary);
    if(right_boundary > WIDTH_IMAGE / 2)
    {
        point A, B, P;

        A.x = 480;
        A.y = 380;
        B.x = 410;
        B.y = 320;

        P.x = left_boundary;
        P.y = obj.center.y + (obj.size_y / 2);

        // check if the lower left corner of the bb is on the right side of the line
        if(directionOfPoint(A, B, P) == LEFT)
        {
            return true;
        }
    }
    return false;
}
