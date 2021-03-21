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
  int radius_within_robot = copysign(1, (std::abs(radius) - wheel_sep_width_/2 ));
  std::vector<float> wheels_steer_angles;
  wheels_steer_angles.resize(4);
  float hypoteneus_left = std::hypot(wheel_sep_length_ / 2, (radius - wheel_sep_width_/2));
  float hypoteneus_right = std::hypot(wheel_sep_length_ / 2, (radius + wheel_sep_width_/2));
  wheels_steer_angles.at(0) = (hypoteneus_left / std::abs(radius)) * effort;
  wheels_steer_angles.at(1) = (hypoteneus_right / std::abs(radius)) * effort * radius_within_robot;
  wheels_steer_angles.at(2) = (hypoteneus_right / std::abs(radius)) * effort * radius_within_robot;
  wheels_steer_angles.at(3) = (hypoteneus_left / std::abs(radius)) * effort;

  return wheels_steer_angles;
}

std::vector<float> NavigationAlgo::getSteeringAnglesRadialTurn(const geometry_msgs::Point center_of_rotation)
{
  std::vector<float> wheels_steer_angles;
  
  wheels_steer_angles.resize(4);

  float front_left_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_left_y = center_of_rotation.y - wheel_sep_width_/2;
  float front_right_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_right_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_left_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_left_y = center_of_rotation.y - wheel_sep_width_/2;

  wheels_steer_angles.at(0) = -atan(front_left_x / front_left_y);
  wheels_steer_angles.at(1) = -atan(front_right_x / front_right_y);
  wheels_steer_angles.at(2) = -atan(back_right_x / back_right_y);
  wheels_steer_angles.at(3) = -atan(back_left_x / back_left_y);

  return wheels_steer_angles;
}

std::vector<float> NavigationAlgo::getDrivingVelocitiessRadialTurn(const geometry_msgs::Point center_of_rotation, const float velocity)
{
  float radius = std::hypot(center_of_rotation.x, center_of_rotation.y);
  std::vector<float> wheels_steer_angles;
  wheels_steer_angles.resize(4);

  float front_left_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_left_y = center_of_rotation.y - wheel_sep_width_/2;
  float front_right_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_right_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_left_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_left_y = center_of_rotation.y - wheel_sep_width_/2;

  float hypoteneus_front_left = std::hypot(front_left_x, front_left_y);
  float hypoteneus_front_right = std::hypot(front_right_x, front_right_y);
  float hypoteneus_back_right = std::hypot(back_right_x, back_right_y);
  float hypoteneus_back_left = std::hypot(back_left_x, back_left_y);

  int invert_velocity_front_left = 1;//copysign(1, (std::abs(radius) - hypoteneus_front_left));
  int invert_velocity_front_right = 1;//copysign(1, (std::abs(radius) - hypoteneus_front_right));
  int invert_velocity_back_right = 1;//copysign(1, (std::abs(radius) - hypoteneus_back_right));
  int invert_velocity_back_left = 1;//copysign(1, (std::abs(radius) - hypoteneus_back_left));

  wheels_steer_angles.at(0) = (hypoteneus_front_left / std::abs(radius)) * velocity * invert_velocity_front_left;
  wheels_steer_angles.at(1) = (hypoteneus_front_right / std::abs(radius)) * velocity * invert_velocity_front_right;
  wheels_steer_angles.at(2) = (hypoteneus_back_right / std::abs(radius)) * velocity * invert_velocity_back_right;
  wheels_steer_angles.at(3) = (hypoteneus_back_left / std::abs(radius)) * velocity * invert_velocity_back_left;

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
