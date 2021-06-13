#include <geometry_msgs/PoseStamped.h>


class  BatteryLevelServer{

private:

public:
float distance_;
float soft_deadline_;
float hard_deadline_;
//repair station location
geometry_msgs::PoseStamped target_location_;
target_location_.pose.position.x = -6.0;
target_location_.pose.position.y = -6.0;
target_location_.pose.position.z = 0.65;
target_location_.pose.orientation.x = 0;
target_location_.pose.orientation.y = 0;
target_location_.pose.orientation.z = 0;
target_location_.pose.orientation.w = 1;

geometry_msgs::PoseStamped current_location_;
void BatteryLevelServer::poseCallback(geometry_msgs::PoseStamped pose);
void BatteryLevelServer::deadlinesCalculator();

};
