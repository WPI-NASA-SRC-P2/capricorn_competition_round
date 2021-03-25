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
  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  if (argc != 4)
  {
      // Getting the script name
      std::string filename = std::string(argv[0]);
      int index = filename.find_last_of('/');
      std::string input_trace_filename = filename.substr(index + 1);

      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>";);
      return -1;
  }
  else
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
    ros::Publisher fl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    ros::Publisher fr_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    ros::Publisher bl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    ros::Publisher br_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

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
}
