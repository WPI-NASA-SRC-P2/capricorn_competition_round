#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <operations/ParkRobotAction.h>
#include <operations/NavigationAction.h> 
#include <string>
#include <utils/common_names.h>
#include <perception/ObjectArray.h>

#define UPDATE_HZ 10

// global variables for park excavator
bool parked = false, found_orientation = false;
float max_diff = -1;

// global variables for park hopper
int hopper_x, processingPlant_z, furnace_x, hopper_height, times_reached = 0, center_image_x = 320, target_height = 385;
double hopper_z;

bool execute_called = false;
perception::ObjectArray objects;

typedef actionlib::SimpleActionServer<operations::ParkRobotAction> Server;

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
Client* client;
operations::NavigationGoal nav_goal;

void objects_callback(const perception::ObjectArray& objs) 
{
    if(!execute_called)
    {
        return;
    }
    
    objects = objs;
}

void park_hopper()
{
    int n = objects.number_of_objects; 

    for(int i = 0; i<n; i++)
    {
        perception::Object object = objects.obj.at(i);
        //If hopper is detected store hopper_x and hopper_z 
        if(object.label == "hopper"){
            hopper_x = object.center.x; // in pixels
            hopper_z = object.point.z;
            hopper_height = object.size_y;
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

    if ((abs(hopper_x - center_image_x) < 50) && (abs(furnace_x - center_image_x) < 500)) //check to see if hopper is close to center and if furnace is detected
    {
        if(hopper_height >= target_height) //if hopper is too close, stop the robot
        {
            ROS_INFO_STREAM("Stopping Hauler");
            times_reached++;
            if(times_reached == 10)
            {
                nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
                nav_goal.forward_velocity = 0.0000001;
                nav_goal.angular_velocity = 0;
                parked = true;
            }
        }
        else //otherwise drive towards hopper
        {
            ROS_INFO_STREAM("Driving To Hopper");
            nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            nav_goal.forward_velocity = 0.5;
            nav_goal.angular_velocity = 0;
        }
    }
    else
    {
        double radius = processingPlant_z + 1.8; //set radius of orbit, 1.8 is the depth of the object 
        geometry_msgs::PointStamped pt; 
        pt.point.x = radius; 
        nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE; //all nav goals will be using the revolve drive
        nav_goal.point = pt;
        if (hopper_x > 320) // if the hopper is on the right side of the screen, orbit right 
        {
            nav_goal.forward_velocity = 0.5;
        }
        else //otherwise orbit left 
        {
            nav_goal.forward_velocity = -0.5;
        }   
    }
}

void park_excavator()
{
    bool found_ea = false, found_ra = false;
    float center_x_ea = INT_MAX, center_x_ra = INT_MAX, z_ra = INT_MAX, size_x_ea = INT_MAX, size_y_ea = INT_MAX, size_x_ra = INT_MAX, size_y_ra= INT_MAX;
    long area_ra = -1, area_ea = -1;
        
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        if(objects.obj[i].label == COMMON_NAMES::OBJECT_DETECTION_ROBOT_ANTENNA_CLASS) 
        {
            if(found_ra)
            {
                return;
            }
            found_ra = true;
            z_ra = objects.obj[i].point.z;
            size_x_ra = objects.obj[i].size_x;
            size_y_ra = objects.obj[i].size_y;
            area_ra = size_x_ra * size_y_ra;
            center_x_ea = objects.obj[i].center.x;
        }
        else if(objects.obj[i].label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_ARM_CLASS) 
        {
            if(found_ea)
            {
                return;
            }
            found_ea = true;
            size_x_ea = objects.obj[i].size_x;
            size_y_ea = objects.obj[i].size_y;
            area_ea = size_x_ea * size_y_ea;
            center_x_ra = objects.obj[i].center.x;
        }
    }

    if(found_orientation)
    {
        ROS_INFO_STREAM("Height of RA: "<<size_y_ra);
        ROS_INFO_STREAM("Depth of RA: "<<z_ra);
    }

    if(found_ra && z_ra < 3 && found_orientation)
    {
        nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        nav_goal.forward_velocity = 0.0000001;
        nav_goal.angular_velocity = 0;
        parked = true;
    }

    if(found_ea && found_ra)
    {
        float ratio = size_x_ea / size_y_ea;
        ROS_INFO_STREAM("Dimensions of RA X: "<<size_x_ra<<", Y: "<<size_y_ra);
        ROS_INFO_STREAM("Dimensions of EA X: "<<size_x_ea<<", Y: "<<size_y_ea);
        ROS_INFO_STREAM("Ratio: "<<size_x_ea / size_y_ea);

        if(center_x_ea < center_x_ra)
        {
            float diff = center_x_ra - center_x_ea;
            max_diff = std::max(max_diff, diff);

            if(diff > 150 && diff < max_diff - 20)
            {
                ROS_INFO("Stop Revolution");
                found_orientation = true;
                nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
                nav_goal.forward_velocity = 0.5;
                nav_goal.angular_velocity = 0;
            }
            ROS_INFO_STREAM("Center RA: "<<center_x_ea<<", Center EA: "<<center_x_ra);
            ROS_INFO_STREAM("Difference centers: "<<diff);
        }
        ROS_INFO_STREAM("");
    }
}

void execute(const operations::ParkRobotGoalConstPtr& goal, Server* as)
{
    execute_called = true; 
    parked = false;
    ros::Rate update_rate(UPDATE_HZ);
    bool hopper_or_excavator = false;
    
    if(goal->hopper_or_excavator == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
    {
        ROS_INFO("Parking Hopper");
        hopper_or_excavator = true;
        times_reached = 0;
        hopper_x = -1;
        processingPlant_z = -1;
        furnace_x = -1;
        hopper_height = -1;
        center_image_x = 320;
        target_height = 385;
        double hopper_z;
    }
    else
    {
        ROS_INFO("Parking Excavator");      
        found_orientation = false;
        max_diff = -1;
    }

    geometry_msgs::PointStamped pt;
    double radius = 3.5;
    pt.point.x = radius;

    nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE;
    nav_goal.point = pt;
    nav_goal.forward_velocity = 0.5;

    while (ros::ok())
    {    
        if(hopper_or_excavator)
        {
            park_hopper();
        }
        else
        {
            park_excavator();
        }

        client->sendGoal(nav_goal);    
        if(parked)
        {
            ROS_INFO("Parked");
            break;
        }
        update_rate.sleep();
    }
    
    as->setSucceeded();
    execute_called = false;
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and target passed as a command line argument!");
        return -1;
    }

    //take in robot name as arg1, usually small_hauler_1
    std::string robot_name = argv[1];

    //initialize node and node handler
    ros::init(argc, argv, robot_name + COMMON_NAMES::PARK_HAULER_HOPPER_SERVER_NODE_NAME);
    ros::NodeHandle nh;

    //subscriber for object detection
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);
    client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    Server server(nh, robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ROS_INFO("Starting Park Hauler Server");
    ros::spin();
    
    ROS_INFO("Exiting");
    return 0;
}