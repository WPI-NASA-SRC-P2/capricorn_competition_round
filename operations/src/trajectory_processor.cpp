#include <ros/ros.h>
#include <operations/TrajectoryWithVelocities.h>


/**
 * @brief A callback function for messages on the imu topic.
 * 
 * @param imu_msg The message coming from the IMU topic for this robot.
 */
void traj_callback(const operations::TrajectoryWithVelocities::ConstPtr &traj_msg)
{
    //Process the message. This function should be kept short, and ideally should not do a lot of processing.

    std::cout << "Received a traj message!" << std::endl;

    std::cout << "First x coordinate: " << traj_msg->waypoints[0].pose.position.x << std::endl;
    std::cout << "Frame of reference: " << traj_msg->waypoints[0].header.frame_id << std::endl;
}

int main(int argc, char *argv[])
{
    //Initialize the node, and create a private NodeHandle.
    ros::init(argc, argv, "template");
    ros::NodeHandle nh("~");

    ros::Subscriber traj_sub = nh.subscribe("/capricorn/traj", 10, traj_callback);

    while(ros::ok())
    {
        ros::spinOnce();
    }
}
