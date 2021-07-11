#include "dyn_planning_2.h"


tf2_ros::Buffer buffer_;
tf2_ros::TransformListener *listener_;

std::unordered_map<std::string, float> DynamicPlanning2::LineEquation(geometry_msgs::Point wp1, geometry_msgs::Point wp2)
{
    std::unordered_map<std::string, float> parameters;
    float deltaX = wp2.x - wp1.x;
    float deltaY = wp2.y - wp1.y;
    float m = deltaY/deltaX;
    parameters["a"]= -deltaY/deltaX;;
    parameters["b"] = 1;
    parameters["c"] = wp2.y - m*wp2.x;
    parameters["m"] = deltaY/deltaX;;
    
    return parameters;
}

float DynamicPlanning2::ObstacleRadius(float width)
{
    float radius = (width +2)/2;

    return radius;

}

float DynamicPlanning2::PerpendicularDistance(std::unordered_map<std::string, float> parameters, geometry_msgs::Point centroidOfObstacle)
{   
  float denom = sqrt(pow(parameters["m"], 2) + 1);  
  float distance = abs(-parameters["m"]*centroidOfObstacle.x + centroidOfObstacle.y - parameters["c"])/denom;

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

  ROS_ERROR("[planning | nav_algos]: tf2::ExtrapolationException too many times in a row! Failed while transforming from %s to %s at time %d.%d", pose.header.frame_id.c_str(), frame.c_str(), pose.header.stamp.sec, pose.header.stamp.nsec);

	return false;
}

nav_msgs::Path DynamicPlanning2::getPathInMapFrame(nav_msgs::Path path)
{
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
	return in_map_frame;
}


 

bool DynamicPlanning2::checkAllObstacles(perception::ObjectArray obstacles, nav_msgs::Path path)
{
    float min_dist = INFINITY;
    float dist;
    float radius;
    std::vector<std::unordered_map<std::string, float>> CompletePathParameters;
    nav_msgs::Path new_path = getPathInMapFrame(path);
    for (int i = 0; i < new_path.poses.size() - 2; i++)
    {
        ROS_INFO("checkAllObstacles - 1");

        std::unordered_map<std::string, float> parameters = LineEquation(new_path.poses[i + 1].pose.position, new_path.poses[i + 2].pose.position );
        CompletePathParameters.push_back(parameters); // should save the parameters of all the segments in path

        ROS_INFO("checkAllObstacles - 2");

    }

    for (int l = 0; l < CompletePathParameters.size(); l++ )
    {
        ROS_INFO("checkAllObstacles - 3");

        for (int j = 0; j < obstacles.number_of_objects; j++)
        {
            geometry_msgs::Point centroid;
            centroid.x = obstacles.obj[j].center.x;
            centroid.y = obstacles.obj[j].center.y;

            dist = PerpendicularDistance( CompletePathParameters[l], centroid);
            radius = ObstacleRadius(obstacles.obj[j].width);
            if(dist < radius )
            {
                return true;
            }
            else 
            {
                return false;
            }
        }
        
    }
    return false;
}