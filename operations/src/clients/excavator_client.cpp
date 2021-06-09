#include <operations/ExcavatorAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

int SLEEP_DURATION = 0.5; // The sleep duration

typedef actionlib::SimpleActionClient<operations::ExcavatorAction> Client;

std::vector<float> getDepthHeight(geometry_msgs::Point hauler_center)
{
  float l1 = 0.8; // shoulder link lenght
  float l2 = 0.8; // elbow link length
  std::vector<float> thetas;
  float D = hauler_center.x;
  float H = hauler_center.z;//+0.5;
  float d = (pow(D,2) + pow(H,2) - pow(l1,2) - pow(l2,2))/(2*l1*l2);
  // ROS_INFO_STREAM("d: "<<d);
  float theta3 = atan(sqrt(1-pow(d,2))/d);
  float theta2 = atan(H/D) - atan(l2*sin(theta3)/(l1+l2*cos(theta3)));

  thetas.push_back(-theta2);
  thetas.push_back(-theta3);
  ROS_INFO_STREAM("Degrees: " << -180/M_PI*(theta2)<< "\t Radian: "<<-theta2);
  ROS_INFO_STREAM("Degrees: " << -180/M_PI*(theta3)<< "\t Radian: "<<-theta3);
  // thetas.push_back(-acos((pow(D,2) + pow(H,2) - pow(l1,2) - pow(l2,2))/(2*l1*l2)));
  // thetas.push_back(-(atan2(H, D)-atan2(l2*sin(thetas[1]),(l1 + l2*cos(thetas[1])))));

  return thetas;
}

/**
 * @brief The main method for excavator client
 * 
 * @param argc The number of arguments passed
 * @param argv The array of arguments passed
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "excavator_client");

  float d = std::stof(argv[1]), h = std::stof(argv[2]), s = std::stof(argv[3]);
  geometry_msgs::Point hauler;
  hauler.x = d;
  hauler.y = s;
  hauler.z = h;

  getDepthHeight(hauler);

  return 0;
}