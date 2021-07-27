#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <operations/WheelVelocities.h>
#include <utils/common_names.h>
#include <operations/navigation_algorithm.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 50

// using namespace COMMON_NAMES;

//We store the robot name globally, in case it is needed in functions other than main.
std::string robot_name;

// The current velocity the wheels are traveling at. Used to ramp to new velocities.
std::mutex vel_mutex;
operations::WheelVelocities last_velocity;

bool interrupt_ramp;

const float VEL_EPSILON = 0.01;

ros::Publisher front_left_vel_pub_;
ros::Publisher front_right_vel_pub_;
ros::Publisher back_left_vel_pub_;
ros::Publisher back_right_vel_pub_;
ros::Publisher finished_pub_;

/**
 * @brief Helper function to avoid floating point math problems.
 *          Returns true iff the doubles are within epsilon of
 *          eachother.
 * 
 * @param velA 1st float to compare
 * @param velB 2nd float to compare
 */
bool floatEqual(const double velA, const double velB)
{
    return abs(velA - velB) <= VEL_EPSILON;
}

/**
 * @brief Helper function to avoid floating point math problems.
 *          Returns true iff both vectors are the same size,
 *          and each component of velA floatEquals
 *          the corresponding component of velB.
 * 
 * @param velA 1st float vector to compare
 * @param velB 2nd float vector compare
 */
bool floatVecEqual(const std::vector<double> velsA, const std::vector<double> velsB)
{
    bool ret = true;

    if(velsA.size() != velsB.size())
    {
        return false;
    }

    for(int i=0; i < velsA.size(); i++)
    {
        ret = ret && floatEqual(velsA[i], velsB[i]);
    }
    return ret;
}

/**
 * @brief Helper function to package and publish target velocities
 * 
 * @param publisher rostopic to publish to
 * @param data velocity to request
 */
void publishMessage(ros::Publisher& publisher, double data)
{
    std_msgs::Float64 pub_data;
    pub_data.data = data;

    publisher.publish(pub_data);
}

/**
 * @brief Callback to update the velocities to ramp to
 * 
 * @param velocities_msg incoming velocity message
 */
void wheelVelsCallback(const operations::WheelVelocities::ConstPtr &velocities_msg)
{
    // If the new velocities are the same as the current velocities, do nothing
    if(!floatVecEqual(velocities_msg->velocities,last_velocity.velocities))
    {
        std::lock_guard<std::mutex> velocity_guard(vel_mutex);
        // Update new_velocities
        last_velocity = *velocities_msg;

        // Stop current rampWheelVelocities
        interrupt_ramp = true;
    }
}

/**
 * @brief Helper function to retrieve the current target velocity
 */
operations::WheelVelocities getLastVelocityMsg()
{
    std::lock_guard<std::mutex> velocity_guard(vel_mutex);

    return last_velocity;
}

int main(int argc, char *argv[])
{
    if (argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with robot name passed as an argument.");
        return -1;
    }
    else
    {
        std::string name(argv[1]);
        robot_name = name;

        ros::init(argc, argv, "ramp_wheel_velocities");

        ros::NodeHandle nh;
        
        ros::Subscriber nav_wheel_vels = nh.subscribe(CAPRICORN_TOPIC + robot_name + "/desired_wheel_velocities", 50, wheelVelsCallback);
        
        front_left_vel_pub_  = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_LEFT_WHEEL  + VELOCITY_TOPIC, 50);
        front_right_vel_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_RIGHT_WHEEL + VELOCITY_TOPIC, 50);
        back_left_vel_pub_   = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_LEFT_WHEEL   + VELOCITY_TOPIC, 50);
        back_right_vel_pub_  = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_RIGHT_WHEEL  + VELOCITY_TOPIC, 50);
        finished_pub_        = nh.advertise<std_msgs::Bool>("/" + robot_name + RAMP_DONE_TOPIC, 50);
        std_msgs::Bool finished_data;
        finished_data.data = false;

        // Initialize the last velocity message to 0 velocities
        last_velocity.velocities = {0, 0, 0, 0};

        std::vector<double> current_velocities = {0, 0, 0, 0};
        std::vector<double> desired_velocities = {0, 0, 0, 0};

        ROS_INFO("[operations | ramp_wheel_velocities | %s]: Ramp wheel velocities started.", robot_name.c_str());


        while(ros::ok())
        {
            operations::WheelVelocities last_vels = getLastVelocityMsg();

            // If the most recent set of velocities is different than the current velocities, we need to restart the ramp
            if(!floatVecEqual(last_vels.velocities, current_velocities) && !floatVecEqual(last_vels.velocities, desired_velocities))
            {
                ROS_INFO("[operations | ramp_wheel_velocities | %s]: New set of velocities, beginning ramp.", robot_name.c_str());

                desired_velocities = getLastVelocityMsg().velocities;

                // Get the time to ramp between current and desired velocities for each wheel
                double longest_time_d = NavigationAlgo::getLongestRampTime(current_velocities, desired_velocities);
                ros::Duration longest_time(longest_time_d);

                ROS_INFO("[operations | ramp_wheel_velocities | %s]: Saving start time %f", robot_name.c_str(), longest_time_d);

                // Save start time
                ros::Time start_time = ros::Time::now();
                std::vector<double> temp_goal_vels(4);

                ROS_INFO("[operations | ramp_wheel_velocities | %s]: Beginning ramp loop", robot_name.c_str());

                bool cont_ramping = false;

                try
                {
                    cont_ramping = (ros::Time::now() - start_time < longest_time);
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }

                // Ramp all wheels for that duration
                while(cont_ramping)
                {
                    try
                    {
                        // Do the same check inside of the loop. If there are updated velocities, then we need to restart the ramping process
                        if(!floatVecEqual(last_vels.velocities, current_velocities) && !floatVecEqual(last_vels.velocities, desired_velocities))
                        {
                            ROS_INFO("[operations | ramp_wheel_velocities | %s]: Ramp interrupted! Restarting.", robot_name.c_str());

                            // Force the while loop to end by setting the start time to 100 seconds in the past (longer than any possible interpolation)
                            start_time = ros::Time::now() - ros::Duration(100.0);
                            continue;

                            // current_velocities will be updated after the while loop terminates
                        }

                        ros::Duration cur_dt = ros::Time::now() - start_time;

                        double dt = cur_dt.toNSec() * 1e-9;

                        // Check if goal velocity is less than current
                        for (int i = 0; i < desired_velocities.size(); i++)
                        {
                            // Calculate the current interpolated velocity for each wheel
                            temp_goal_vels[i] = current_velocities[i] + (dt / longest_time_d) * (desired_velocities[i] - current_velocities[i]);
                            // ROS_INFO("[operations | ramp_wheel_velocities | %s]: Wheel %d, cV %f, tV %f", robot_name.c_str(), i, current_velocities[i], temp_goal_vels[i]);
                        }

                        // Publish wheel velocities
                        publishMessage(front_left_vel_pub_,  temp_goal_vels[0]);
                        publishMessage(front_right_vel_pub_, temp_goal_vels[1]);
                        publishMessage(back_right_vel_pub_,  temp_goal_vels[2]);
                        publishMessage(back_left_vel_pub_,   temp_goal_vels[3]);
                        finished_data.data = false;
                        finished_pub_.publish(finished_data);

                        // Sleep for some set time
                        ros::Duration(0.005).sleep();
                        ros::spinOnce();
                        cont_ramping = (ros::Time::now() - start_time < longest_time);
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                    
                }

                ROS_INFO("[operations | ramp_wheel_velocities | %s]: Done ramping.", robot_name.c_str());
                finished_data.data = true;
                finished_pub_.publish(finished_data);

                // Update current_velocities_
                current_velocities = temp_goal_vels;
            }
            else
            {
                // ROS_INFO("[operations | ramp_wheel_velocities | %s]: No ramp to do.", robot_name.c_str());
                finished_data.data = true;
                finished_pub_.publish(finished_data);
                ros::Duration(0.05).sleep();
            }
            ros::spinOnce();
        }
    }
}
