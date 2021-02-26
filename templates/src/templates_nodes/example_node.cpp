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
    // Before even starting the node, we must check if the node is being used properly. 
    // This particular node should be launched through a roslaunch, otherwise it will fail.

    // 'argc' gives us the number of arguments that have been passed to the node.
    // The first argument is usually the script name, and the arguemnts passed through command line are after this.
    // When run via roslaunch, Last two arguments will be the node name and log file location.

    // Here we expect that the node is being run via roslaunch.
    // Hence if you passed two arguments, variable 'argc' will have value of 5; 1(script name) + 2(passed args) + 1(node name) + 1(log file location)= 5
    
    // In our case, we check if one arguemnt is passed it should be the RobotName_Number
    // If incorrect number of arguments have been passed, we show error message and terminate the program
    if (argc != 4)
    {
        // argv will have the actual arguments passed. 
        // As discussed above, first arguemnt is the script name. 
        std::string filename = std::string(argv[0]);

        // Variable 'filename' will also have absolute path of the script. To only get the filename of the script,  
        // we find the location of the last forward slash, after which the script file name will be included.
        int index = filename.find_last_of('/');

        // Then the script name is cut from the whole path
        std::string input_trace_filename = filename.substr(index + 1);

        // Displaying an error message for correct usage of the script.
        ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>";);

        // Returning -1 to signify everything is NOT okay....
        return -1;
    }
    else
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

        //Create a node handle. This is what allows us to create publishers and subscribers. Note the `~` passed in. This creates a private node handle, instead
        //of a public one. I don't know all of the implications of this, but it means we can access parameters that are defined within the scope of the node (see
        //the example_nodes.launch file for clarification). If you need a public node handle, you can create one here as well by simply omitting the argument to NodeHandle.
        ros::NodeHandle nh("~");

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



        //Example parameter. We first initialize a variable that we will store the parameter's value in named wheel_effort
        float wheel_effort;

        //We will check to see if the parameter is set. Note that there is no preceding `/` on the parameter name. This means the parameter namespace
        //is the node, so we are looking for a parameter set for this node specifically. If there is a preceding `/`, then it is doing a parameter lookup
        //in the global namespace instead.
        //getParam will store the resulting value inside of wheel_effort, if the parameter exists. Otherwise, it will return false.
        if(nh.getParam("~wheel_effort", wheel_effort))
        {
            //Assert that we did get the wheel_effort parameter properly
            std::cout << "Set up wheel_effort param for " + robot_name + "." << std::endl;
        }
        else
        {
            //If the parameter was not set, then we need to set our variable to a default variable so that we can still use it later
            std::cout << "No wheel_effort param set for " + robot_name + ", defaulting to 20.0." << std::endl;
            wheel_effort = 20.0;
        }



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
            speed.data = wheel_effort;

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
}
