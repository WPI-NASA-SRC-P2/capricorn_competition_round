#include <ros/ros.h>

WavefrontFrontiersDetector::WavefrontFrontiersDetector()
{


/**
 * @brief 
 * 
 * @return std::vector<vector<int>> 
 */
static std::vector<vector<int>> wfd()
{
}

/**
 * @brief 
 * 
 * @return std::boolean 
 */
static std::boolean is_frontier_point()
{
}

/**
 * @brief Get the Nearest Frontier object
 * 
 * @return std::boolean 
 */
static std::boolean getNearestFrontier()
{
}

/**
 * @brief Get the Euclidean Distance 
 * 
 * @param x1 
 * @param y1 
 * @param x2 
 * @param y2 
 * @return std::float
 */
static std::float getEuclideanDistance(float x1, float y1, float x2, float y2)
{
    return distance;
}


/**
 * @brief Get the Frontier Median object
 * 
 * @param frontierList 
 * @return std::int 
 */
static std::vector<int> getFrontierMedian(vector<int> frontierList)
{ 
    return Median;
}

}

WavefrontFrontiersDetector::~WavefrontFrontiersDetector()
{
}

/**
 * @brief main function for quick-testing of the class
 * 
 */
int main(int argc, char *argv[])
{
    // ROS initialization
    ros::init(argc, argv, "template_class_tester");
    ros::NodeHandle nh;

    // Creted an object of the class
    TemplateClass template_class;

    // Setting the team name
    std::string temp_string = "Team Capricorn";
    template_class.setTeamName(temp_string);

    // Getting the team name
    std::string team_name = template_class.getTeamName();

    // Printing the team name
    ROS_INFO_STREAM("Team name from class object is: " << team_name);

    return 0;
}
