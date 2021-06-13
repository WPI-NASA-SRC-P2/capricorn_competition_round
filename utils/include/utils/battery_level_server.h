#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class  BatteryLevelServer{
private:

public:
float distance_;
float soft_deadline_;
float hard_deadline_;

void poseCallback(nav_msgs::Odometry odom);
void deadlinesCalculator();

};
