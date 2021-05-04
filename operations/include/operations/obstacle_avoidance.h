/*
Author BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2

Consider the following image below, an object will be considered as an obstacle if it is in triangular region
in middle right of the image, this triangular region corresponds to the projected path of the robot
---------------------
|         |  \      |
|         |    \    |
|         |     \   |
|         |      \  |
|         |        \|
---------------------
*/

#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <map>
#include <string>

// structure for point in cartesian plane.
struct point {
    int x, y;
    point(int X, int Y) : x(X), y(Y)
    {
    }
};

const float WIDTH_IMAGE = 640.0, HARDCODED_DIRECTION = 0.74f;

// Height thresholds for corresponding object to be considered as obstacles
const std::map<std::string, int> OBSTACLE_HEIGHT_THRESHOLD = {
    {COMMON_NAMES::OBJECT_DETECTION_PROCESSING_PLANT_CLASS, 400,},
    {COMMON_NAMES::OBJECT_DETECTION_REPAIR_STATION_CLASS, 400,},
    {COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS, 100,},
    {COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_ARM_CLASS, 80,},
    {COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS, 100,},
    {COMMON_NAMES::OBJECT_DETECTION_HAULER_CLASS, 100,},
    {COMMON_NAMES::OBJECT_DETECTION_ROCK_CLASS, 60,}
};

// Line points which divides projected path of the robot (in pixels)
const point A(480, 380), B(410, 320);

/**
 * @brief This function calculates if a point of interest is on right or left side of a line connecting two points
 * 
 * @param first - First point of line
 * @param second - Second point of line
 * @param P - Point of interest
 * @return bool - True if the object is in left of the line 
 */
bool directionOfPoint(point first, point second, point P)
{
    // subtracting co-ordinates of point A from
    // B and P, to make A as origin

    second.x -= first.x;
    second.y -= first.y;
    P.x -= first.x;
    P.y -= first.y;
    // Determining cross Product
    int cross_product = second.x * P.y - second.y * P.x;
 
    // return LEFT if cross product is negative
    if (cross_product < 0)
        return true;
 
    // return RIGHT if cross product is positive or zero
    return false;
}

/**
 * @brief Checks for all the objects, if its an obstacle in the projected path of the robot, if it is, returns the magnitude of crab walk needed
 * 
 * @param obstacles - Vector having objects
 * @return direction magnitude for crab walk
 */
float checkObstacle(const std::vector<perception::Object>& obstacles)
{ 
    float result = 0.0f;

    bool obstacle_in_left_half = false;
    for(int i = 0; i < obstacles.size(); i++)
    {
        perception::Object object = obstacles.at(i);

        // verifies the object class is considered as an obstacle and checks if the height threshold, to ensure object is big enough to be an obstacle
        if(OBSTACLE_HEIGHT_THRESHOLD.find(object.label) != OBSTACLE_HEIGHT_THRESHOLD.end() && object.size_y > OBSTACLE_HEIGHT_THRESHOLD.at(object.label)) 
        {
            float left_boundary = object.center.x - (object.size_x / 2), right_boundary = object.center.x + (object.size_x / 2);
            
            // in the above projected path, every object on the left half of the center of image, is not an obstacle
            if(right_boundary > WIDTH_IMAGE / 2)
            {
                point P(left_boundary, object.center.y + (object.size_y / 2));

                // check if the lower left corner of the bb is on the right side of the line
                if(directionOfPoint(A, B, P))
                {
                    result = HARDCODED_DIRECTION;
                }
            }
            
            if(object.center.x < WIDTH_IMAGE / 2)
            {
                // if there is an obstacle in left half of the image
                obstacle_in_left_half = true;
            }
        }
    }

    if(obstacle_in_left_half)
    {
        // if there is an obstacle in left half of the image, always crab walk right
        result = -std::abs(result);
    }

    return result;
}
