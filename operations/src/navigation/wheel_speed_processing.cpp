#include <ros/ros.h>
#include <utils/common_names.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 100

using namespace COMMON_NAMES;

std::string robot_name;

std_msgs::Float64 bl_speed, br_speed, fl_speed, fr_speed;

/**
 * @brief A callback function for joint states to publish encoder velocities
 * 
 * @param imu_msg The message coming from the IMU topic for this robot.
 */
void encoder_callback(const sensor_msgs::JointState::ConstPtr &joints)
{
    /*
    bl: 0
    br: 1
    fl: 2
    fr: 3
    */
    double WHEEL_RAD = 0.17;

    bl_speed.data = joints->velocity[0] * WHEEL_RAD;
    br_speed.data = joints->velocity[1] * WHEEL_RAD;
    fl_speed.data = joints->velocity[2] * WHEEL_RAD;
    fr_speed.data = joints->velocity[3] * WHEEL_RAD;
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
      ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>");
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
    
    //Initialize subscribers for each wheel
    ros::Subscriber joint_state_sub = nh.subscribe("/" + robot_name + "/joint_states", 10, encoder_callback);

    ros::Publisher bl_speed_pub = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_LEFT_WHEEL + CURRENT_SPEED, 1000);
    ros::Publisher br_speed_pub = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_RIGHT_WHEEL + CURRENT_SPEED, 1000);
    ros::Publisher fl_speed_pub = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_LEFT_WHEEL + CURRENT_SPEED, 1000);
    ros::Publisher fr_speed_pub = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_RIGHT_WHEEL + CURRENT_SPEED, 1000);

    while(ros::ok())
    {
        bl_speed_pub.publish(bl_speed);
        br_speed_pub.publish(br_speed);
        fl_speed_pub.publish(fl_speed);
        fr_speed_pub.publish(fr_speed);

        ros::spinOnce();

        //Rate limit the loop to UPDATE_HZ.
        update_rate.sleep();
    }
  }
}
