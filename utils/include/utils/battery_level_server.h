#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class  BatteryLevelServer{
private:

public:
static float soft_deadline_;
static float hard_deadline_;

void poseCallback(nav_msgs::Odometry odom);
void deadlinesCalculator();

};
