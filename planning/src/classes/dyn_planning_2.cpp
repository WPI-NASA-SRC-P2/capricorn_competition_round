#include "dyn_planning_2.h"


tf2_ros::Buffer buffer_;
tf2_ros::TransformListener *listener_obstacle_;




std::unordered_map<std::string, float> DynamicPlanning2::LineEquation(geometry_msgs::Point wp1, geometry_msgs::Point wp2)
{
    std::unordered_map<std::string, float> lineParameters;
    float deltaX = wp2.x - wp1.x;
    float deltaY = wp2.y - wp1.y;
    //CHECK IF THE DELTAx IS ZERO 
    
  
    float m = deltaY/deltaX;
    lineParameters["a"]= -m;
    lineParameters["b"] = 1;
    lineParameters["c"] = wp2.y - m*wp2.x;
    lineParameters["m"] = m;
    lineParameters["wp1x"] = wp1.x;
    lineParameters["wp1y"] = wp1.y;
    lineParameters["wp2x"] = wp2.x;
    lineParameters["wp2y"] = wp2.y;
    
    return lineParameters;
}


// std::unordered_map<std::string, float> DynamicPlanning2::CircleEquation(geometry_msgs::Point centroid, float radius)
// {
//  std::unordered_map<std::string, float> parameters;
//  float  a = -2 * centroid.x;
//  float  b = -2 * centroid.y;
//  float  c = pow(radius, 2) - pow(centroid.x, 2) - pow(centroid.y, 2);
//  parameters["a"]= a;
//  parameters["b"] = b;
//  parameters["c"] = c;

//  return circle_parameters;
// }

float DynamicPlanning2::mIntersect(std::unordered_map<std::string, float> lineParameters,float radius)
{
  // ROS_INFO("mIntersect started");
  float d_0 = (abs(lineParameters["c"]))/(sqrt(pow(lineParameters["a"], 2) + pow(lineParameters["b"], 2)));

  float dIntersect = sqrt(abs((pow(radius, 2) - pow(d_0, 2))));
//   ROS_INFO_STREAM("RADIUS  " << radius   << "   d0   " << d_0);
  float mIntersect = sqrt(pow(dIntersect,2))/((pow(lineParameters["a"], 2) + pow(lineParameters["b"], 2)));

  return mIntersect;

}


float DynamicPlanning2::ObstacleRadius(float width)
{
  float radius = (width +2)/2;

  return radius;
}


float DynamicPlanning2::PerpendicularDistance(std::unordered_map<std::string, float> parameters, geometry_msgs::Point centroidOfObstacle)
{  
  // ROS_INFO("PerpendicularDistance started");

  float distance; 
  if(!isnan(parameters["m"]))
  {
    float denom = sqrt(pow(parameters["m"], 2) + 1);  
    distance = abs(-parameters["m"]*centroidOfObstacle.x + centroidOfObstacle.y - parameters["c"])/(denom);
    
  }
  else
  {
    distance = abs(centroidOfObstacle.x - parameters["wp1x"]); 

  }
  // ROS_INFO("distance %f", distance);
  return distance;
    
}

bool DynamicPlanning2::transformPose(geometry_msgs::PoseStamped& pose, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration, int tries)
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

//   ROS_ERROR("[planning | nav_algos]: tf2::ExtrapolationException too many times in a row! Failed while transforming from %s to %s at time %d.%d", pose.header.frame_id.c_str(), frame.c_str(), pose.header.stamp.sec, pose.header.stamp.nsec);

	return false;
}

nav_msgs::Path DynamicPlanning2::getPathInMapFrame(nav_msgs::Path path)
{
    ros::Time begin = ros::Time::now();
    
    nav_msgs::Path in_map_frame;
	

	// For each waypoint in the trajectories message
	for(int pt = 0; pt < path.poses.size(); pt++)
	{
		geometry_msgs::PoseStamped map_pose = path.poses[pt];

		// Transform that waypoint into the map frame
	    //map_pose.header.stamp = ros::Time(0);
	    DynamicPlanning2::transformPose(map_pose, COMMON_NAMES::MAP, buffer_);

		// Push this waypoint 
        in_map_frame.poses.push_back(map_pose);

	}

	//Reverse trajectory waypoints from planner
	std::reverse(in_map_frame.poses.begin(), in_map_frame.poses.end());

  ros::Time end = ros::Time::now();
  ros::Duration duration = end - begin;
//   ROS_INFO("pathInMapFrame function takes %lf secs", duration.toSec());
	return in_map_frame;
}

std::pair<float,float> DynamicPlanning2::solveQuadraticEquation(float a, float b, float c)
{
  float x1, x2;
    float discriminant = b*b - 4*a*c;
    float realPart, imaginaryPart;
    
    if (discriminant > 0) {
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2 = (-b - sqrt(discriminant)) / (2*a);
        // cout << "Roots are real and different." << endl;
        // cout << "x1 = " << x1 << endl;
        // cout << "x2 = " << x2 << endl;
        return std::make_pair(x1,x2);
    }
    
    else if (discriminant == 0) {
        // cout << "Roots are real and same." << endl;
        x1 = -b/(2*a);
        // cout << "x1 = x2 =" << x1 << endl;
        return std::make_pair(x1,x1);
    }

    else {
        realPart = -b/(2*a);
        imaginaryPart =sqrt(-discriminant)/(2*a);
        // cout << "Roots are complex and different."  << endl;
        // cout << "x1 = " << realPart << "+" << imaginaryPart << "i" << endl;
        // cout << "x2 = " << realPart << "-" << imaginaryPart << "i" << endl;
        // ROS_ERROR("Imaginary root found");
        return std::make_pair(-0,0);
    }
}

std::vector<float> DynamicPlanning2::PointsofIntersection(geometry_msgs::Point centroid, float mi, std::unordered_map<std::string, float>  lineParameters, float radius)
{
  // ROS_INFO("PointsofIntersection started");
  // float x_0 = -(lineParameters["a"]*(lineParameters["c"]))/(pow(lineParameters["a"], 2) + pow(lineParameters["b"], 2) );
  // float y_0 = -(lineParameters["b"]*(lineParameters["c"]))/(pow(lineParameters["a"], 2) + pow(lineParameters["b"], 2) );
  // float x_0 = sqrt(abs((pow(radius,2))-(pow((lineParameters["m"]* + lineParameters["c"] - centroid.y), 2))));
  float a = (1 + pow(lineParameters["m"], 2));
  float b = (-2*centroid.x + 2*lineParameters["m"]*(lineParameters["c"] - centroid.y));
  float c = pow((lineParameters["c"]-centroid.y), 2) - pow(radius, 2) + pow(centroid.x, 2);

  std::pair<float, float> solutions = solveQuadraticEquation(a,b,c);
  float ax = solutions.first, bx = solutions.second;
  float ay = (sqrt(abs((pow(radius, 2)-pow((ax - centroid.x), 2)))) + centroid.y);
  float by = -(sqrt(abs((pow(radius, 2)-pow((bx - centroid.x), 2)))) - centroid.y);


  // float ax = centroid.x + x_0;// + lineParameters["b"]*mi;
  // float ay = centroid.y + y_0;// - lineParameters["a"]*mi; 
  // float bx = centroid.x - x_0;// - lineParameters["b"]*mi;
  // float by = centroid.y - y_0;// + lineParameters["a"]*mi; 

  std::vector<float> iPoints;
  iPoints.push_back(ax);
  iPoints.push_back(ay);
  iPoints.push_back(bx);
  iPoints.push_back(by);


  // std::pair<float, float> point1, point2;
  // point1.first = ax;
  // point1.second = ay;
  // point2.first = bx;
  // point2.second = by;


  // std::vector<std::pair<float, float>> iPoints;
  // iPoints.push_back(point1);
  // iPoints.push_back(point2);
  //  ROS_INFO("x0: %f", x_0);
  //  ROS_INFO("y0 : %f", y_0);
  //  ROS_INFO("m-slope WP : %f", lineParameters["m"]);
  //  ROS_INFO("mi-slope IP : %f", mi);
  //  ROS_INFO("iPoint1X : %f", ax);
  //  ROS_INFO("iPoint1Y : %f", ay);
  //  ROS_INFO("iPoint2X : %f", bx);
  //  ROS_INFO("iPoint2Y : %f", by);

  return iPoints;
}


bool DynamicPlanning2::CheckIfBewtweenWaypoints(std::unordered_map<std::string, float> lineParameters, std::vector<float> iPoints)
{
    // ROS_INFO("CheckIfBewtweenWaypoints started");
  // https://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
  float point1x = iPoints.at(0);
  float point1y = iPoints.at(1);
  float point2x = iPoints.at(2);
  float point2y = iPoints.at(3);

  float wp1x = lineParameters["wp1x"]; 
  float wp1y = lineParameters["wp1y"]; 
  float wp2x = lineParameters["wp2x"]; 
  float wp2y = lineParameters["wp2y"]; 


  auto dotproduct1 = (point1x - wp1x)*(wp2x - wp1x) + (point1y - wp1y)*(wp2y - wp1y);
  auto dotproduct2 = (point2x - wp1x)*(wp2x - wp1x) + (point2y - wp1y)*(wp2y - wp1y);

  if((dotproduct1  < 0) || (dotproduct2 < 0))
  {
    return false;
  }

  float squaredlengthwps = pow((wp2x - wp1x),2) + pow((wp2y - wp1y), 2);
  if (dotproduct1 > squaredlengthwps || dotproduct2 > squaredlengthwps)
  {
    return true;
  }
  else
  {
    return true;
  }
//     std::pair<float, float> d0;  //vector
//     d0.first = lineParameters["wpx2"] - lineParameters["wpx1"];
//     d0.second = lineParameters["wpy2"] - lineParameters["wpy1"];

//     float dMagnitude = sqrt(pow(d0.first,2 ) + pow(d0.second,2));
    
//     //v ---> unit vector
//     std::pair<float, float> v;
//     v.first = d0.first/dMagnitude;
//     v.second = d0.second/dMagnitude;

//     for (int i = 0; i < iPoints.size(); i++)
//     {
//       std::pair<float,float> d1;
//       d1.first = iPoints.at(i).first - lineParameters["wpx1"];
//       d1.second = iPoints.at(i).second - lineParameters["wpy1"];

//       float dotProduct = v.first*d1.first + v.second*d1.second;
//       if(dotProduct >= 0 && dotProduct <= dMagnitude)  // ipoints between lineparameter's waypoints
//       {
//         ROS_INFO("between waypoints - Yipeeeeeee!!!!!!!!!");
//         return true;
//       }  
//     }
// return false;
}


void DynamicPlanning2::ReversePath(nav_msgs::Path& path)
{
    path.poses.pop_back();

    auto start = 0;
    auto end = path.poses.size() - 1;
    while (start < end)
    {
        auto temp = path.poses.at(start);
        path.poses[start] = path.poses.at(end);
        path.poses[end] = temp;
        start++;
        end--;
    } 
}

 

bool DynamicPlanning2::checkAllObstacles(perception::ObjectArray obstacles, nav_msgs::Path path, std::string robot_name, const tf2_ros::Buffer& tf_buffer)
{
    ros::Time begin = ros::Time::now();

    float dist;
    float radius;
    static std::vector<std::unordered_map<std::string, float>> CompletePathParameters;
    CompletePathParameters.resize(0);
    //nav_msgs::Path new_path = getPathInMapFrame(path);
    for (int i = 0; i < path.poses.size() - 1; i++) // starting from 1, because robot is located at 1st index
    {
        // ROS_INFO("checkAllObstacles - 1");

        std::unordered_map<std::string, float> parameters = LineEquation(path.poses.at(i).pose.position, path.poses.at(i+1).pose.position );
        CompletePathParameters.push_back(parameters); // should save the parameters of all the segments in path

        // ROS_INFO("checkAllObstacles - 2");

    }

    for (int l = 0; l < CompletePathParameters.size(); l++ )
    {
        // ROS_INFO("checkAllObstacles - 3");

        for (int j = 0; j < obstacles.number_of_objects; j++)
        {
          geometry_msgs::PoseStamped obstaclePoseStamped = obstacles.obj[j].point; 
	        DynamicPlanning2::transformPose(obstaclePoseStamped, COMMON_NAMES::MAP, tf_buffer);

            geometry_msgs::Point centroid;
            centroid.x = obstaclePoseStamped.pose.position.x;
            centroid.y = obstaclePoseStamped.pose.position.y;
            centroid.z = obstaclePoseStamped.pose.position.z;

            dist = PerpendicularDistance( CompletePathParameters[l], centroid);    
            radius = ObstacleRadius(obstacles.obj[j].width);
            // ROS_INFO("distance: %f", dist);
            // ROS_INFO("radius: %f", radius);
            if(dist < radius )
            {
                return true;
            }

        }
        
    }
    // delete listener_obstacle_;
    ros::Time end = ros::Time::now();
    ros::Duration duration = end - begin;
    // ROS_INFO("checkAllObstacles %lf secs", duration.toSec());

    return false;
}


 bool DynamicPlanning2::CheckIPwithRobot(geometry_msgs::PoseStamped& current_robot_pose,std::vector<float> iPoints)
 {
    float robotx = current_robot_pose.pose.position.x; 
    float roboty = current_robot_pose.pose.position.y; 
    float point1x = iPoints.at(0);
    float point1y = iPoints.at(1);
    float point2x = iPoints.at(2);
    float point2y = iPoints.at(3);
    float safeDistance1 = sqrt(pow((robotx - point1x),2) + pow((roboty - point1y),2));
    float safeDistance2 = sqrt(pow((robotx - point2x),2) + pow((roboty - point2y),2)); 

    // ROS_INFO_STREAM("safedistance1: " << safeDistance1);
    // ROS_INFO_STREAM("safedistance2: " << safeDistance2);
    if(safeDistance1 < 7 || safeDistance2 < 7)
    {
      return true;
    }
    else
    { 
      return false;
    }
 }


bool DynamicPlanning2::checkAllObstacles2(perception::ObjectArray obstacles, nav_msgs::Path path, std::string robot_name, geometry_msgs::PoseStamped current_robot_pose, const tf2_ros::Buffer& tf_buffer)
{

  // ROS_INFO("checkAllObstacles2 started");

  float dist;
  float radius;
  static std::vector<std::unordered_map<std::string, float>> CompletePathParameters; 
  CompletePathParameters.resize(0);
  //generates line parameters for all the line segments that make the path
  int pathsize = path.poses.size();
  ReversePath(path);
  // ROS_INFO_STREAM("pathsize"<< pathsize);
    for (int i = 0; i < path.poses.size() - 1; i++) // starting from 1, because robot is located at 1st index
    {
        // ROS_INFO("path points x : %f", path.poses.at(i+1).pose.position.x);
        // ROS_INFO("path points y : %f", path.poses.at(i+1).pose.position.y);
        std::unordered_map<std::string, float> parameters = LineEquation(path.poses.at(i).pose.position, path.poses.at(i+1).pose.position );
        CompletePathParameters.push_back(parameters); // should save the parameters of all the segments in path

    }

    //iterating over each line
    for (int l = 0; l < CompletePathParameters.size(); l++ )
    {   
        // ROS_INFO("WAPOINTS :%f", CompletePathParameters[l]);

        
        // checking every obstacle for the intersection with the line
        for (int j = 0; j < obstacles.number_of_objects; j++)
        {
          geometry_msgs::PoseStamped obstaclePoseStamped = obstacles.obj[j].point; 
	        DynamicPlanning2::transformPose(obstaclePoseStamped, COMMON_NAMES::MAP, tf_buffer);

            geometry_msgs::Point centroid;
            centroid.x = obstaclePoseStamped.pose.position.x;
            centroid.y = obstaclePoseStamped.pose.position.y;
            centroid.z = obstaclePoseStamped.pose.position.z;
            // ROS_INFO("-----------------------------");
            // ROS_INFO("Wapoint 1X: %f", CompletePathParameters[l]["wp1x"]);
            // ROS_INFO("Centroid X: %f", centroid.x );
            // ROS_INFO("Wapoint 2X: %f", CompletePathParameters[l]["wp2x"]);
            // ROS_INFO("-----------------------------");
            // ROS_INFO("Wapoint 1Y: %f", CompletePathParameters[l]["wp1y"]);
            // ROS_INFO("Centroid Y: %f", centroid.y);
            // ROS_INFO("Wapoint 2Y: %f", CompletePathParameters[l]["wp2y"]);
            // ROS_INFO("-----------------------------");


            dist = PerpendicularDistance( CompletePathParameters[l], centroid);    
            radius = ObstacleRadius(obstacles.obj[j].width);
            // ROS_INFO("distance: %f", dist);
            // ROS_INFO("radius: %f", radius);
            
            
            if(dist < radius && radius < 9)
            {
            //    ROS_WARN("distance less than radius");
              float mi = mIntersect(CompletePathParameters.at(l), radius);
              
              std::vector<float> iPoints;
              iPoints = PointsofIntersection(centroid,mi,CompletePathParameters.at(l), radius);
              
              
              if(CheckIfBewtweenWaypoints(CompletePathParameters.at(l), iPoints))
              {
                // ROS_INFO_STREAM("pose: " << current_robot_pose);
                if(CheckIPwithRobot(current_robot_pose,iPoints))
                {
                  return true;
                }
              }  
            }
            // ROS_INFO("---------------next obstacle-------------");
        }
        // ROS_INFO("---------------next waypoints-------------");
    }

    // ROS_INFO("******************Loop Completed*****************************");
    return false;
}
