#include <operations/NavigationAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <operations/navigation.h>

typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;

ros::Publisher front_left_pub;
ros::Publisher front_right_pub;
ros::Publisher back_left_pub;
ros::Publisher back_right_pub;

// std_msgs::String front_left_pub_topic = "small_scout_1/front_left_wheel/drive/command/velocity";
// std_msgs::String front_right_pub_topic = "small_scout_1/front_right_wheel/drive/command/velocity";
// std_msgs::String back_left_pub_topic = "small_scout_1/back_left_wheel/drive/command/velocity";
// std_msgs::String back_right_pub_topic = "small_scout_1/back_right_wheel/drive/command/velocity";

void execute(const operations::NavigationGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  std_msgs::Float64 velocity;
  velocity.data = goal->velocity;

  front_left_pub.publish(velocity);
  front_right_pub.publish(velocity);
  back_left_pub.publish(velocity);
  back_right_pub.publish(velocity);

  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_action_server");
  ros::NodeHandle nh;

  front_left_pub = nh.advertise<std_msgs::Float64>("small_scout_1/front_left_wheel/drive/command/velocity", 1000);
  front_right_pub = nh.advertise<std_msgs::Float64>("small_scout_1/front_right_wheel/drive/command/velocity", 1000);
  back_left_pub = nh.advertise<std_msgs::Float64>("small_scout_1/back_left_wheel/drive/command/velocity", 1000);
  back_right_pub = nh.advertise<std_msgs::Float64>("small_scout_1/back_right_wheel/drive/command/velocity", 1000);

  Server server(nh, "navigation", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
