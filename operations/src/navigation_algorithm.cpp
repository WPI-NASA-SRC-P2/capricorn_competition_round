#include <operations/navigation_algorithm.h>

NavigationAlgo::NavigationAlgo(/* args */)
{
}

NavigationAlgo::~NavigationAlgo()
{
}

std::vector<float> NavigationAlgo::getSteeringAnglesRadialTurn(const float radius)
{
  std::vector<float> wheels_steer_angles;
 
  wheels_steer_angles.resize(4);
  wheels_steer_angles.at(0) = atan((wheel_sep_length_ / 2) / (radius - wheel_sep_width_ / 2));
  wheels_steer_angles.at(1) = atan((wheel_sep_length_ / 2) / (radius + wheel_sep_width_ / 2));
  wheels_steer_angles.at(2) = -atan((wheel_sep_length_ / 2) / (radius + wheel_sep_width_ / 2));
  wheels_steer_angles.at(3) = -atan((wheel_sep_length_ / 2) / (radius - wheel_sep_width_ / 2));

  return wheels_steer_angles;
}

std::vector<float> NavigationAlgo::getDrivingVelocitiessRadialTurn(const float radius, const float effort)
{
  int radius_within_robot = copysign(1, (std::abs(radius) - wheel_sep_width_));
  std::vector<float> wheels_steer_angles;
  wheels_steer_angles.resize(4);
  float hypoteneus_left = std::hypot(wheel_sep_length_ / 2, (radius - wheel_sep_width_));
  float hypoteneus_right = std::hypot(wheel_sep_length_ / 2, (radius + wheel_sep_width_));
  wheels_steer_angles.at(0) = (hypoteneus_left / std::abs(radius)) * effort;
  wheels_steer_angles.at(1) = (hypoteneus_right / std::abs(radius)) * effort * radius_within_robot;
  wheels_steer_angles.at(2) = (hypoteneus_right / std::abs(radius)) * effort * radius_within_robot;
  wheels_steer_angles.at(3) = (hypoteneus_left / std::abs(radius)) * effort;

  return wheels_steer_angles;
}

float NavigationAlgo::getRadiusInArchimedeanSpiral(const float t)
{
  float neum = (2 + std::pow(t, 2));
  float denom = std::pow((1 + std::pow(t, 2)), 1.5);
  float curvature = neum / denom;
  float radius = 1 / std::abs(curvature);
  return radius;
}

double NavigationAlgo::headingFromPose(geometry_msgs::PoseStamped* pose){
  geometry_msgs::Quaternion q = pose->pose.orientation;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief 
 * 
 * ref: https://en.wikipedia.org/wiki/Archimedean_spiral
 * ref: https://math.stackexchange.com/questions/1371668/equation-to-place-points-equidistantly-on-an-archimedian-spiral-using-arc-length
 * 
 * @param init_location 
 * @param init_theta 
 * @return std::vector<geometry_msgs::Point> 
 */
std::vector<geometry_msgs::Point> NavigationAlgo::getNArchimedeasSpiralPoints(const geometry_msgs::Point &init_location, const int N, int init_theta)
{
  std::vector<geometry_msgs::Point> points;
  points.resize(N);
  for (int i = init_theta; i < init_theta + N; i++)
  {
    float th = std::pow(i * arc_spiral_incr, 0.5) * arc_spiral_multi;
    float pre = (arc_spiral_a + (arc_spiral_b * th) / (2 * M_PI));
    points.at(i-init_theta).x = pre * cos(th) + init_location.x;
    points.at(i-init_theta).y = pre * sin(th) + init_location.y;
  }
  return points;
}

float NavigationAlgo::getFive()
{
  return 5;
}

std::vector<double> NavigationAlgo::fromQuatToEuler(geometry_msgs::PoseStamped* pose)
{
  geometry_msgs::Quaternion q = pose->pose.orientation;

  tf::Quaternion quat(q.x, q.y, q.z, q.w);

  tf::Matrix3x3 m(quat);

  double roll, pitch, yaw;

  m.getRPY(roll, pitch, yaw);

  std::vector<double> euler_angles = {roll, pitch, yaw};
  
  return euler_angles;
}

// float NavigationAlgo::getBrakingForce(const float ang_vel, const float pitch, const RobotModel robot_model)
// {
//   float mass = RobotDescription::robot_mass_map[robot_model];
//   float gravity = RobotDescription::MOON_GRAVITY;

//   //Calculates the maximum force that can be applied without skidding
//   float braking_force = -mass * gravity * std::cos(std::abs(pitch));

//   //Calculate torque, dividing by 4 to calculate per wheel value instead of the total value
//   float braking_torque = braking_force * RobotDescription::WHEEL_RADIUS / 4;

//   return braking_torque;
// }
