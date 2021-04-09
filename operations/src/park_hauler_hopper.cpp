 
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <operations/navigation_algorithm.h>
#include <perception/ObjectArray.h>
//#include <perception/msg/Object.msg>


#define UPDATE_HZ 10

std::string robot_name;
ros::Publisher fl_steer, fr_steer, bl_steer, br_steer, fl_drive, fr_drive, bl_drive, br_drive;
ros::Subscriber image_sub; 
int hopper_x;
double hopper_z;
int processingPlant_z;
int furnace_x;

void publishMessage(std::vector<double> velocity, std::vector<double> angle)
{
    std_msgs::Float64 angle_msg, vel_msg;
    angle_msg.data = angle[0];
    fl_steer.publish(angle_msg);
    angle_msg.data = angle[1];
    fr_steer.publish(angle_msg);
    angle_msg.data = angle[2];
    br_steer.publish(angle_msg);
    angle_msg.data = angle[3];
    bl_steer.publish(angle_msg);
    
    vel_msg.data = velocity[0];
    fl_drive.publish(vel_msg);
    vel_msg.data = velocity[1];
    fr_drive.publish(vel_msg);
    vel_msg.data = velocity[2];
    br_drive.publish(vel_msg);
    vel_msg.data = velocity[3];
    bl_drive.publish(vel_msg);    
 }


void ppCentroid(const perception::ObjectArray::ConstPtr& msg){
    int n = msg-> number_of_objects; 
    for(int i = 0; i<n; i++)
    {
        perception::Object object = msg->obj.at(i);
        //If hopper is detected store hopper_x and hopper_z 
        if(object.label == "hopper"){
            hopper_x = object.center.x; // in pixels
            hopper_z = object.point.z;
            ROS_INFO_STREAM("Detected Hopper");
        }
        //If processingPlant is detected store processingPlantZ 
        if(object.label == "processingPlant"){
            processingPlant_z = object.point.z; // in meters
        }
        //If furnace is detected store furnace_x
        if(object.label == "furnace"){
            furnace_x = object.center.x; // in meters
        }
    }
 }


void goToHopper()
{
    std::vector<double> forward_steer{0, 0, 0, 0}, drive{3, 3, 3, 3}, stop{0, 0, 0, 0};
    int center_image_x = 320;
    // int distance_threshold = 4.5
    // if (processingPlant_z <  distance_threshold)
    // {
        if ((abs(hopper_x - center_image_x) < 50) && (abs(furnace_x - center_image_x) < 500))
        {
            if(hopper_z < 3.5)
            {
                ROS_INFO_STREAM("Stopping Hauler");
                publishMessage(stop, forward_steer);
            }
            else
            {
                ROS_INFO_STREAM("Driving To Hopper");
                publishMessage(drive, forward_steer);
            }
        }
        else
        {
            if (hopper_x > 320)
            {
                double radius = processingPlant_z + 1.8;
                double velocity = 3;
                geometry_msgs::Point pt;
                pt.x = radius;
                
                std::vector<double> drive = NavigationAlgo::getDrivingVelocitiesRadialTurn(pt, velocity);
                std::vector<double> steer = NavigationAlgo::getSteeringAnglesRadialTurn(pt);
                publishMessage(drive, steer);
            }
            else
            {
                double radius = processingPlant_z + 1.8;
                double velocity = 3;
                geometry_msgs::Point pt;
                pt.x = radius;
                
                std::vector<double> drive = NavigationAlgo::getDrivingVelocitiesRadialTurn(pt, -velocity);
                std::vector<double> steer = NavigationAlgo::getSteeringAnglesRadialTurn(pt);
                publishMessage(drive, steer);
            }
            
        }
    // }
    // else{
    //     //vision_based_nav
    // } 

}




int main(int argc, char *argv[])
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, "park_hauler");

    ros::NodeHandle nh;

    fl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    fr_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    bl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    br_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);

    fl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    fr_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    bl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    br_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);
   

    ros::Rate update_rate(UPDATE_HZ);
    
    image_sub = nh.subscribe("/capricorn/" + robot_name + "/object_detection/objects", 10, ppCentroid);

    while (ros::ok())
    {

        //ROS_INFO_STREAM("publishing");
        goToHopper();
        ros::spinOnce();
        update_rate.sleep();
    }

    ROS_INFO("Exiting");
}