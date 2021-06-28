#include <operations/navigation_algorithm.h>

NavigationAlgo::NavigationAlgo(/* args */)
{
}

NavigationAlgo::~NavigationAlgo()
{
}

std::vector<double> NavigationAlgo::getSteeringAnglesRadialTurn(const float radius)
{
  std::vector<double> wheels_steer_angles;
 
  wheels_steer_angles.resize(4);
  wheels_steer_angles.at(0) = atan((wheel_sep_length_ / 2) / (radius - wheel_sep_width_ / 2));
  wheels_steer_angles.at(1) = atan((wheel_sep_length_ / 2) / (radius + wheel_sep_width_ / 2));
  wheels_steer_angles.at(2) = -atan((wheel_sep_length_ / 2) / (radius + wheel_sep_width_ / 2));
  wheels_steer_angles.at(3) = -atan((wheel_sep_length_ / 2) / (radius - wheel_sep_width_ / 2));

  return wheels_steer_angles;
}

std::vector<double> NavigationAlgo::getDrivingVelocitiesRadialTurn(const float radius, const float effort)
{
  int radius_within_robot = copysign(1, (std::abs(radius) - wheel_sep_width_/2 ));
  std::vector<double> wheels_steer_angles;
  wheels_steer_angles.resize(4);
  double hypoteneus_left = std::hypot(wheel_sep_length_ / 2, (radius - wheel_sep_width_/2));
  double hypoteneus_right = std::hypot(wheel_sep_length_ / 2, (radius + wheel_sep_width_/2));

  double ratio_left = (radius != 0) ? hypoteneus_left / std::abs(radius) : 1;
  double ratio_right = (radius != 0) ? hypoteneus_right / std::abs(radius) : 1;

  wheels_steer_angles.at(0) = (ratio_left) * effort;
  wheels_steer_angles.at(1) = (ratio_right) * effort * radius_within_robot;
  wheels_steer_angles.at(2) = (ratio_right) * effort * radius_within_robot;
  wheels_steer_angles.at(3) = (ratio_left) * effort;

  return wheels_steer_angles;
}

std::vector<double> NavigationAlgo::getSteeringAnglesRadialTurn(const geometry_msgs::Point center_of_rotation)
{
  std::vector<double> wheels_steer_angles;
  wheels_steer_angles.resize(4);

  // Distances of the wheels from the center of rotation
  double front_left_x = center_of_rotation.x - wheel_sep_length_/2;
  double front_left_y = center_of_rotation.y - wheel_sep_width_/2;
  double front_right_x = center_of_rotation.x - wheel_sep_length_/2;
  double front_right_y = center_of_rotation.y + wheel_sep_width_/2;
  double back_right_x = center_of_rotation.x + wheel_sep_length_/2;
  double back_right_y = center_of_rotation.y + wheel_sep_width_/2;
  double back_left_x = center_of_rotation.x + wheel_sep_length_/2;
  double back_left_y = center_of_rotation.y - wheel_sep_width_/2;

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

std::vector<double> NavigationAlgo::getDrivingVelocitiesRadialTurn(const geometry_msgs::Point center_of_rotation, const float velocity)
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
  float ratio_front_left = radius != 0 ? hypoteneus_front_left / radius : 1;
  float ratio_front_right = radius != 0 ? hypoteneus_front_right / radius : 1;
  float ratio_back_right = radius != 0 ? hypoteneus_back_right / radius : 1;
  float ratio_back_left = radius != 0 ? hypoteneus_back_left / radius : 1;

  std::vector<double> wheels_drive_velocities;
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
  const double SCALING_FACTOR = 0.4; // Tuning paramter for scaling the spiral up or down. 

  float neum = (2 + std::pow(t, 2));
  float denom = std::pow((1 + std::pow(t, 2)), 1.5);
  float curvature = neum / denom;
  float radius = 1 / std::abs(curvature);
  
  // This is to make sure that the spiral center lies to the side of the robot and not within the robot. 
  radius += NavigationAlgo::wheel_sep_width_;
  
  radius = SCALING_FACTOR*radius;
  
  return radius;
}

std::vector<geometry_msgs::PointStamped> NavigationAlgo::getRectangularScanningPoints(const int scout_number)
{
  float radius = 80;
  float start_pt_x = scout_number == 1 ? 35 : -35, theta_x = scout_number == 1 ? 5 : -5;
  int max_x = scout_number == 1 ? 80 : -80;

  std::vector<geometry_msgs::PointStamped> points;

  auto getY = [&radius](float x)->float
  {
    return abs(sqrt((radius * radius) - (x * x)));
  };

  auto getPoint = [](float x, float y)->geometry_msgs::PointStamped
  {
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = MAP;
    msg.point.x = x;
    msg.point.y = y;
    return msg;
  };

  points.push_back(getPoint(0, 0));
  points.push_back(getPoint(0, 0));
  points.push_back(getPoint(start_pt_x, 0));

  while (abs(start_pt_x) < abs(max_x))
  {
    float y = getY(start_pt_x);
    if(points.back().point.y == 0)
    {
      points.push_back(getPoint(start_pt_x, y));
    }
    else
    {
      if(points.back().point.y < 0)
      {
        points.push_back(getPoint(start_pt_x, -y));
      points.push_back(getPoint(start_pt_x, 0));
        points.push_back(getPoint(start_pt_x, y));
      }
      else
      {
        points.push_back(getPoint(start_pt_x, y));
        points.push_back(getPoint(start_pt_x, 0));
        points.push_back(getPoint(start_pt_x, -y));
      }
    }
    start_pt_x += theta_x;
  }

  for (auto pt : points)
  {
    ROS_INFO_STREAM("X: " + std::to_string(pt.point.x) + ", Y: " + std::to_string(pt.point.y));
  }

  return points;
}

std::vector<geometry_msgs::PointStamped> NavigationAlgo::getStraightScanningPoints(const int scout_number)
{
  #define PI 3.14159265

  float radius = 80;
  float start_angle = scout_number == 1 ? 10 : -10, theta_x = scout_number == 1 ? 15 : -15;
  int angle_max = scout_number == 1 ? 175 : -175;

  std::vector<geometry_msgs::PointStamped> points;

  auto getAnglePoint = [&radius](float angle)->geometry_msgs::PointStamped
  {
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = MAP;
    msg.point.x = radius * sin(angle*PI/180);
    msg.point.y = radius * cos(angle*PI/180);
    return msg;
  };

  auto getPoint = [](float x, float y)->geometry_msgs::PointStamped
  {
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = MAP;
    msg.point.x = x;
    msg.point.y = y;
    return msg;
  };

  points.push_back(getPoint(0, 0));
  points.push_back(getPoint(0, 0));

  while (abs(start_angle) <= abs(angle_max))
  {
    points.push_back(getAnglePoint(start_angle));
    points.push_back(getPoint(scout_number == 1 ? 20 : -20, 0));
    start_angle += theta_x;
  }

  for (auto pt : points)
  {
    ROS_INFO_STREAM("X: " + std::to_string(pt.point.x) + ", Y: " + std::to_string(pt.point.y));
  }

  return points;
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
std::vector<geometry_msgs::PointStamped> NavigationAlgo::getNArchimedeasSpiralPoints(const int scout_number)
{
  float arc_spiral_a = scout_number == 1 ? arc_spiral_a_1 : arc_spiral_a_2;
  int init_theta = scout_number == 1 ? init_theta_1 : init_theta_2;

  std::vector<geometry_msgs::PointStamped> points;
  points.resize(N);
  for (int i = init_theta; i < init_theta + N; i++)
  {
    float th = std::pow(i * arc_spiral_incr, 0.5);
    float pre = (arc_spiral_a + (arc_spiral_b * th) / (2 * M_PI));
    points.at(i-init_theta).header.frame_id = MAP;
    points.at(i-init_theta).point.x = pre * cos(th);
    points.at(i-init_theta).point.y = pre * sin(th);
  }
  return points;
}

double NavigationAlgo::linearToAngularVelocity(double linear_vel)
{
  return linear_vel/wheel_rad_;
}

std::vector<double> NavigationAlgo::fromQuatToEuler(const geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::Quaternion q = pose.pose.orientation;

  tf::Quaternion quat(q.x, q.y, q.z, q.w);

  tf::Matrix3x3 m(quat);

  double roll, pitch, yaw;

  m.getRPY(roll, pitch, yaw);

  std::vector<double> euler_angles = {roll, pitch, yaw};
  
  return euler_angles;
}

double NavigationAlgo::changeInPosition(const geometry_msgs::PoseStamped& current_robot_pose, const geometry_msgs::PoseStamped& target_robot_pose)
{
  //Get the change in x and y between the two poses
	double delta_x = current_robot_pose.pose.position.x - target_robot_pose.pose.position.x;
	double delta_y = current_robot_pose.pose.position.y - target_robot_pose.pose.position.y;

  //Return the distance formula from these deltas
	return pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);
}

double NavigationAlgo::changeInPosition(const geometry_msgs::PoseStamped& current_robot_pose, const geometry_msgs::PointStamped& target_robot_pose)
{
  //Get the change in x and y between the two poses
	double delta_x = current_robot_pose.pose.position.x - target_robot_pose.point.x;
	double delta_y = current_robot_pose.pose.position.y - target_robot_pose.point.y;

  //Return the distance formula from these deltas
	return pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);
}

double NavigationAlgo::changeInHeading(const geometry_msgs::PoseStamped& current_robot_pose, const geometry_msgs::PoseStamped& current_waypoint, const std::string& robot_name, const tf2_ros::Buffer& tf_buffer)
{
  // Hack (kind of) for Ashay. If the two poses are within 15cm of each other, we assume that we want to match orientations, not turn to face a waypoint
  // TODO: Make sure planner team doesn't give us two waypoints that is within this tolerance
  if(changeInPosition(current_robot_pose, current_waypoint) < 0.15)
  {
    return changeInOrientation(current_waypoint, robot_name, tf_buffer);
  }

	// Get the next waypoint in the robot's frame
	geometry_msgs::PoseStamped waypoint_relative_to_robot = current_waypoint;
  //// CHANGE BY ASHAY ////
  //  This will only provide the 'latest' trnasform irrespective of
  // the timestamp requested. I don't see a case in which we would need
  // to inquire the past transform, so this can leave be. But we must know
  // that currently we cannot fetch the past transform, and will only get 
  // the latest one. 
  waypoint_relative_to_robot.header.stamp = ros::Time(0);
  transformPose(waypoint_relative_to_robot, robot_name + ROBOT_CHASSIS, tf_buffer, 0.1);

  //Get the change in yaw between the two poses with atan2
	double change_in_yaw = atan2(waypoint_relative_to_robot.pose.position.y, waypoint_relative_to_robot.pose.position.x);

  //ROS_ERROR("[operations | nav_algo | %s]: Change in yaw before calculations %f", robot_name.c_str(), change_in_yaw);
	
	// We want the robot to turn in the direction of the smallest angle change, so if abs(change) > 180, flip its direction
	if(change_in_yaw >= M_PI)
	{
		change_in_yaw -= 2*M_PI;
	}
	
	if (change_in_yaw <= -M_PI) 
	{
		change_in_yaw += 2*M_PI;
	}

  //ROS_ERROR("[operations | nav_algo | %s]: Change in yaw after calculations %f", robot_name.c_str(), change_in_yaw);
	
	return change_in_yaw;
}

double NavigationAlgo::changeInOrientation(const geometry_msgs::PoseStamped& desired_pose, const std::string& robot_name, const tf2_ros::Buffer& tf_buffer)
{
  geometry_msgs::PoseStamped relative_to_robot = desired_pose;

  if(transformPose(relative_to_robot, robot_name + ROBOT_CHASSIS, tf_buffer, 0.1))
  {
    return fromQuatToEuler(relative_to_robot)[2];
  }
  else
  {
    ROS_ERROR("[operations | nav_algos | %s]: transformPose failed in changeInOrientation. Returning 0.", robot_name.c_str());

    return 0;
  }
}

bool NavigationAlgo::transformPose(geometry_msgs::PoseStamped& pose, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration, int tries)
{
  int count = 0;
  pose.header.stamp = ros::Time(0);
  while(count++ < tries)
  {
    try
    {
      pose = tf_buffer.transform(pose, frame, ros::Duration(duration));
      return true;
    }
    catch(tf2::ExtrapolationException e)
    {
      // do nothing, this is fine if count < tries
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }

  ROS_ERROR("[operations | nav_algos]: tf2::ExtrapolationException too many times in a row! Failed while transforming from %s to %s at time %d.%d", pose.header.frame_id.c_str(), frame.c_str(), pose.header.stamp.sec, pose.header.stamp.nsec);

	return false;
}

bool NavigationAlgo::transformPoint(geometry_msgs::PointStamped& point, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration, int tries)
{
  geometry_msgs::PoseStamped to_transform;
  to_transform.pose.position = point.point;
  to_transform.header = point.header;

  bool ret = transformPose(to_transform, frame, tf_buffer, duration, tries);

  point.point = to_transform.pose.position;
  point.header = to_transform.header;

  return ret;
}

/**
 * @brief 
 *          http://www.ambrsoft.com/TrigoCalc/Circle3D.htm
 * 
 * @param points 
 * @return double 
 */
double NavigationAlgo::getRadiusOfThreePointsCircle(const std::vector<geometry_msgs::PointStamped>& points)
{
  if(points.size()!=3)
  {
    ROS_ERROR("[operations | nav_algos]: Must get 3 points to get Radius, supplied %lu points", points.size());

    return -1.0;
  }


  std::vector<double> abcd = getABCDofThreePointsCircle(points);
  double A, B, C, D;
  A = abcd.at(0);
  B = abcd.at(1);
  C = abcd.at(2);
  D = abcd.at(3);

  return std::sqrt((B*B + C*C - 4*A*D)/(4*A*A));
}

geometry_msgs::PointStamped NavigationAlgo::getCenterOfThreePointsCircle(const std::vector<geometry_msgs::PointStamped>& points)
{
  if(points.size()!=3)
  {
    ROS_ERROR("[operations | nav_algos]: Must get 3 points to get Center Location, supplied %lu points", points.size());

    geometry_msgs::PointStamped err;
    return err;
  }

  std::vector<double> abcd = getABCDofThreePointsCircle(points);
  double A, B, C, D;
  A = abcd.at(0);
  B = abcd.at(1);
  C = abcd.at(2);
  D = abcd.at(3);

  geometry_msgs::PointStamped center;
  center.header = points.at(0).header;
  center.point.x = -B/(2*A);
  center.point.y = -C/(2*A);
  return center; 
}

std::vector<double> NavigationAlgo::getABCDofThreePointsCircle(const std::vector<geometry_msgs::PointStamped>& points)
{
  if(points.size()!=3)
  {
    ROS_ERROR("[operations | nav_algos]: Must get 3 points to calculate the A B C and D constants, supplied %lu points", points.size());
    
    // Not sure if this is right, but needs to return something. 
    std::vector<double> err;
    return err;
  }
  double x1,x2,x3,y1,y2,y3;
  x1 = points.at(0).point.x;
  y1 = points.at(0).point.y;

  x2 = points.at(1).point.x;
  y2 = points.at(1).point.y;
  
  x3 = points.at(2).point.x;
  y3 = points.at(2).point.y;
  
  std::vector<double> abcd_points;
  abcd_points.resize(4);
  abcd_points.at(0) = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2; // A
  abcd_points.at(1) = (x1*x1 + y1*y1)*(y3-y2) + (x2*x2 + y2*y2)*(y1-y3) + (x3*x3 + y3*y3)*(y2-y1); // B
  abcd_points.at(2) = (x1*x1 + y1*y1)*(x2-x3) + (x2*x2 + y2*y2)*(x3-x1) + (x3*x3 + y3*y3)*(x1-x2); // C
  abcd_points.at(3) = (x1*x1 + y1*y1)*(x3*y2-x2*y3) + (x2*x2 + y2*y2)*(x1*y3-x3*y1) + (x3*x3 + y3*y3)*(x2*y1-x1*y2); // D

  return abcd_points;
}

geometry_msgs::Pose NavigationAlgo::getPointCloserToOrigin(const geometry_msgs::Pose& ref_point, const geometry_msgs::Pose& self_point, const double closer_distance)
{
  double ref_x,ref_y;
  ref_x = ref_point.position.x;
  ref_y = ref_point.position.y;

  double self_x,self_y;
  self_x = self_point.position.x;
  self_y = self_point.position.y;

  double x = ref_x - self_x;
  double y = ref_y - self_y;
  double theta = std::atan2(y, x);
  
  double out_x = ref_x - cos(theta)*closer_distance;
  double out_y = ref_y - sin(theta)*closer_distance;

  double dist = std::hypot(out_x - self_x, out_y - self_y);

  geometry_msgs::Pose out_point;
  if(dist<closer_distance)
    out_point = self_point;
  else
  {
    out_point.position.x = out_x;
    out_point.position.y = out_y;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, theta );

    tf2::convert(myQuaternion, out_point.orientation);
  }

  return out_point;
}