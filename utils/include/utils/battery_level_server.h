#include <geometry_msgs/PoseStamped.h>


class  BatteryLevelServer{

private:

public:
float distance_;
float soft_deadline_;
float hard_deadline_;
//repair station location
geometry_msgs::PoseStamped target_location_;


geometry_msgs::PoseStamped current_location_;
void poseCallback(geometry_msgs::PoseStamped pose);
void deadlinesCalculator();

};
