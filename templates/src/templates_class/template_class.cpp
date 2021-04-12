#include <ros/ros.h>
#include <templates/template_class.h>

TemplateClass::TemplateClass()
{
}

TemplateClass::~TemplateClass()
{
}

std::string TemplateClass::getTeamName()
{
    // returns the private variable 
    return team_name;
}

void TemplateClass::setTeamName(const std::string& input_string)
{
    // team_name is assigned the value of Input string
    // Doing this, you keep the private variable safe 
    team_name = input_string;
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
    ROS_INFO_STREAM("Team name from class object is: "<<team_name);

    return 0;
}
