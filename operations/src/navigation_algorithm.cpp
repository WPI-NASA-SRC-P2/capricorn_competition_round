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

  float ratio_left = (radius != 0) ? hypoteneus_left / std::abs(radius) : 1;
  float ratio_right = (radius != 0) ? hypoteneus_right / std::abs(radius) : 1;

  wheels_steer_angles.at(0) = (ratio_left) * effort;
  wheels_steer_angles.at(1) = (ratio_right) * effort * radius_within_robot;
  wheels_steer_angles.at(2) = (ratio_right) * effort * radius_within_robot;
  wheels_steer_angles.at(3) = (ratio_left) * effort;

  return wheels_steer_angles;
}

std::vector<float> NavigationAlgo::getSteeringAnglesRadialTurn(const geometry_msgs::Point center_of_rotation)
{
  std::vector<float> wheels_steer_angles;
  wheels_steer_angles.resize(4);

  // Distances of the wheels from the center of rotation
  float front_left_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_left_y = center_of_rotation.y - wheel_sep_width_/2;
  float front_right_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_right_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_left_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_left_y = center_of_rotation.y - wheel_sep_width_/2;

  // Output Angles
  wheels_steer_angles.at(0) = -atan(front_left_x / front_left_y);
  wheels_steer_angles.at(1) = -atan(front_right_x / front_right_y);
  wheels_steer_angles.at(2) = -atan(back_right_x / back_right_y);
  wheels_steer_angles.at(3) = -atan(back_left_x / back_left_y);

  // If the center of rotation lies right between the two wheels,
  // then atan causes the wheel to rotate in the opposite direction
  // Hence the fix
  if (front_left_y == 0 && front_left_x<0)
  {
    wheels_steer_angles.at(0) *= -1;
    wheels_steer_angles.at(3) *= -1;
  }

  return wheels_steer_angles;
}

std::vector<float> NavigationAlgo::getDrivingVelocitiessRadialTurn(const geometry_msgs::Point center_of_rotation, const float velocity)
{
  // Distances of the wheels from the center of rotation
  float front_left_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_left_y = center_of_rotation.y - wheel_sep_width_/2;
  float front_right_x = center_of_rotation.x - wheel_sep_length_/2;
  float front_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_right_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_right_y = center_of_rotation.y + wheel_sep_width_/2;
  float back_left_x = center_of_rotation.x + wheel_sep_length_/2;
  float back_left_y = center_of_rotation.y - wheel_sep_width_/2;

  // Euclidian distance of the center of rotation from all wheels
  float hypoteneus_front_left = std::hypot(front_left_x, front_left_y);
  float hypoteneus_front_right = std::hypot(front_right_x, front_right_y);
  float hypoteneus_back_right = std::hypot(back_right_x, back_right_y);
  float hypoteneus_back_left = std::hypot(back_left_x, back_left_y);

  float abs_x = std::abs(center_of_rotation.x);
  float abs_y = std::abs(center_of_rotation.y);

  // Radius or rotation
  float radius = std::hypot(center_of_rotation.x, center_of_rotation.y);


  // If the center of rotation lies at the center of rotation, then 
  // the radius will be 0. Hence the output will be inf. 
  // Hence the if else condition
  float ratio_front_left = radius !=0 ? hypoteneus_front_left / radius : 1;
  float ratio_front_right = radius !=0 ? hypoteneus_front_right / radius : 1;
  float ratio_back_right = radius !=0 ? hypoteneus_back_right / radius : 1;
  float ratio_back_left = radius !=0 ? hypoteneus_back_left / radius : 1;

  std::vector<float> wheels_drive_velocities;
  wheels_drive_velocities.resize(4);

  // If the center of rotation lies between the two wheels, and in front 
  // of the robot, then the front left and back left wheels need to 
  // turn in the opposite direction to keep stability
  int invert_velocity = (abs_y > wheel_sep_width_/2)?1:-1;

  wheels_drive_velocities.at(0) = (ratio_front_left) * velocity * invert_velocity;
  wheels_drive_velocities.at(1) = (ratio_front_right) * velocity;
  wheels_drive_velocities.at(2) = (ratio_back_right) * velocity;
  wheels_drive_velocities.at(3) = (ratio_back_left) * velocity * invert_velocity;
  
  return wheels_drive_velocities;
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
