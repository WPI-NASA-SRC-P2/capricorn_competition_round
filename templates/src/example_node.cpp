#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

//We store the robot name globally, in case it is needed in functions other than main.
std::string robot_name;

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    //Log that we received the message. The IMU data will be stored in `imu_msg`. We don't do anything with it here, but usually
    //this data would either be processed in the subscriber or saved to a global variable if it needs to be processed elsewhere.

    std::cout << robot_name + " received an IMU message" << std::endl;
}

int main(int argc, char *argv[])
{
    //The first thing we will do is determine which robot this node is running for. If this is a global node (1 overall instead of 1 per robot),
    //some of the following logic can be omitted.

    //If the node is started for a robot instead of globally, it needs to be initialized in the namespace of the robot. For example, if this node
    //is started for hauler 1, it will be launched in `/capricorn/hauler_1/<node_name>`, where node_name in this case is "example_node".

    //We start by grabbing the robot name and number from the arguments as strings
    //argv[0] is usually the script name, so the first real argument is argv[1]
    std::string name(argv[1]);

    //Initialize the global name to the concatenation of name and number, with a `_` in between
    robot_name = name;



    //Initialize the node. We always pass in argc and argv, and the third argument is the node name.
    //argc and argv are used to preprocess remaps from the command line. See
    //http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown section 1.1 and 
    //http://wiki.ros.org/Remapping%20Arguments for more details.
    ros::init(argc, argv, "example_node");

    //Create a node handle. This is what allows us to create publishers and subscribers.
    ros::NodeHandle nh;

    //Create a ROS rate. We use this later to keep a constant update rate for the main loop.
    ros::Rate update_rate(UPDATE_HZ);
    


    //Example Subscriber. We setup one dummy subscriber for the purpose of the template
    //This Subscriber is subcribing to the robot specific imu topic. We will queue 10 messages.
    //Everytime the subscriber receives a message it will call imu_callback which is setup outside the main loop above.
    ros::Subscriber imu_sub = nh.subscribe("/" + robot_name + "/imu", 10, imu_callback);
    


    //Example publisher. We setup four publishers, one for each wheel on the robot (front left/right and back left/right).
    //We are publishing Float64 to each wheel's command topic. We will queue 10 messages. If we are sending messages faster
    //than ROS can process them, it will store up to 10 messages before it starts dropping them.

    //There is an optional third parameter to an nh.advertise<>() called "latch", which is a boolean. It defaults to false, but
    //if it true, it will save the most recent message published. Any node that begins subscribing will automatically receive
    //this most recent message. See http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers section 1.2 for more details.
    ros::Publisher fl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/fl_wheel_controller/command", 10);
    ros::Publisher fr_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/fr_wheel_controller/command", 10);

    ros::Publisher bl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/bl_wheel_controller/command", 10);
    ros::Publisher br_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/br_wheel_controller/command", 10);



    //ros::ok() will return false once either ROS has been shut down or the node has been shut down. This allows the node to exit without hanging.
    while(ros::ok())
    {
        //Publishes wheel speeds to their respective topics. The SRC Wiki does not provide much clarification as to the units
        //of "wheel speed", so they are assumed to be wheel effort. This means the wheels may not always travel at the exact same
        //speed. In the qualification round, we did some processing with PID controllers to match a desired speed, but in this example
        //we will just send a constant effort of 20.

        //We always need a ROS message to publish, even if it is a basic data type like an integer or float. These publishers were created
        //with std::msgs::Float64, so we need to create one to pass to publish.

        std_msgs::Float64 speed;
        speed.data = 20.0;

        fl_speed.publish(speed);
        fr_speed.publish(speed);
        bl_speed.publish(speed);
        br_speed.publish(speed);



        //We need to allow ROS to update, so we tell ROS to run its update loop once per loop.
        ros::spinOnce();

        //This will pause the node for enough time so that this while loop matches UPDATE_HZ. In this case,
        //the while loop will run 10 times per second.
        update_rate.sleep();
    }
}
