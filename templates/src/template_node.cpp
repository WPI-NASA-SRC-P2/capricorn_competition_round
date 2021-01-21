#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

std::string robot_name;

/**
 * @brief A callback function for messages on the imu topic.
 * 
 * @param imu_msg The message coming from the IMU topic for this robot.
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    //Process the message. This function should be kept short, and ideally should not do a lot of processing.
}

int main(int argc, char *argv[])
{
    //Get robot name from parameters, and store it globally.
    std::string name(argv[1]);
    robot_name = name;

    //Initialize the node, and create a private NodeHandle.
    ros::init(argc, argv, "template");
    ros::NodeHandle nh("~");

    //Create the rate limiter for the loop.
    ros::Rate update_rate(UPDATE_HZ);
    
    //Initialize subscribers. This example subscribes to this robot's imu topic.
    ros::Subscriber imu_sub = nh.subscribe("/" + robot_name + "/imu", 10, imu_callback);

    //Initialize publishers. This example create publishers to command each wheel on this robot.
    ros::Publisher fl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/fl_wheel_controller/command", 10);
    ros::Publisher fr_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/fr_wheel_controller/command", 10);
    ros::Publisher bl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/bl_wheel_controller/command", 10);
    ros::Publisher br_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/br_wheel_controller/command", 10);

    float wheel_effort;

    //Example parameter get.
    if(nh.getParam("wheel_effort", wheel_effort))
    {
        std::cout << "Wheel effort set to " << wheel_effort << " by parameter for " + robot_name << std::endl;
    }
    else
    {
        std::cout << "No wheel_effort param set for " + robot_name + ", defaulting to 20.0." << std::endl;
        wheel_effort = 20.0;
    }

    while(ros::ok())
    {
        //Publish a constant wheel speed to each wheel.
        std_msgs::Float64 speed;
        speed.data = wheel_effort;

        fl_speed.publish(speed);
        fr_speed.publish(speed);
        bl_speed.publish(speed);
        br_speed.publish(speed);

        ros::spinOnce();

        //Rate limit the loop to UPDATE_HZ.
        update_rate.sleep();
    }
}