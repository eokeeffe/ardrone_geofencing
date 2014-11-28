#include <ros/ros.h>
#include <iostream>
/* Math Libraries */
#include <math.h>
#include <float.h>
#include <geometry_msgs/Vector3Stamped.h>
/* ArDrone Headers */
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_gps.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
/* Wii Nunchuck Headers */
#include <wii_nunchuck/nunchuck.h>
/* File parser headers */
#include <fstream>
#include <string>
#include <vector>

// Drone variables
/* Drone State and battery */
float batteryPercent;
int state;

/* Control Variables */
float speed = 0.5;
float altitude,current_altitude,magX,magY,magZ,yaw,pitch,roll;
float longitude=.0,latitude=.0,elevation=.0,gps_speed=.0;
float previous_longitude=.0,previous_latitude=.0;
float current_bearing=.0;
float velx=.0,vely=.0,velz=.0;
int reset_count = 0;
/* Flight time recording */
float takeoff_time,start_time,fly_time;

/* Drone Publishers */
ros::Publisher pub_empty_land;
ros::Publisher pub_twist;
ros::Publisher pub_empty_takeoff;
ros::Publisher pub_empty_reset;
ros::Publisher recal_trim,recal_imu;

/* Drone movement messages */
geometry_msgs::Twist twist_msg_hover,twist_msg;
std_msgs::Empty emp_msg;

/* Drone Subscibers */
ros::Subscriber droneSensors;
ros::Subscriber droneGPS;

/* Wii Nunchuck Subscribers */
ros::Subscriber buttonC,buttonZ;

//perimeter file
std::string perimeter_filename;

using namespace std;

void readDroneMagSensors(const geometry_msgs::Vector3Stamped& msg);
void readDroneGPS(const ardrone_autonomy::navdata_gps& msg);
void readDroneSensors(const ardrone_autonomy::Navdata& msg);
void droneNunchuck(const wii_nunchuck::nunchuck& msg);
void changeState(bool c,bool z);
void processJoystick(uint8_t x,uint8_t y);
void calibrateDrone();
//heading estimation
float bearing();
//momentum estimation
float momentum();
//reset the drone command message to hover
void resetTwist();
//read the gps coordinates
void read_coordinates(std::string filename);
inline bool file_exists(const std::string& name);

struct coordinate
{
    double x,y;
    coordinate(double input_x,double input_y)
    {
        x = input_x;
        y = input_y;
    }
    
};

inline bool file_exists(std::string& name)
{
    return ( access( name.c_str(), F_OK ) != -1 );
}

void read_coordinates(std::string filename)
{
    if(file_exists(filename))
    {
        vector<coordinate> points;
        string line;
        ifstream gps_file (filename.c_str());
        if (gps_file.is_open())
        {
            double x=.0,y=.0;
            while ( getline (gps_file,line) )
            {
                sscanf(line.c_str(),"%lf%*s%lf%*s",&x,&y);
                points.push_back(coordinate(x,y));
            }
            gps_file.close();
        }
   
        
        
        ROS_INFO("GPS coordinates have been parsed");
        ROS_INFO("Generating perimeter");
        
        for (int i=0;i< points.size(); i++) 
        {
            coordinate point = points.at(i);
            std::cout << point.x << ','  <<  point.y << std::endl;
        }
    }
    else
    {
        ROS_INFO("GPS Perimeter File not found");
        ROS_INFO("Shutting down");
        ros::shutdown();
    }
}

void resetTwist()
{
    twist_msg.linear.x=0.0;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
}

void readDroneMagSensors(const geometry_msgs::Vector3Stamped& msg)
{
    magX = msg.vector.x;
    magY = msg.vector.y;
    magZ = msg.vector.z;
}

void readDroneGPS(const ardrone_autonomy::navdata_gps& msg)
{
    latitude = msg.latitude;
    longitude = msg.longitude;
    elevation = msg.elevation;
    gps_speed = msg.speed;
}

void readDroneSensors(const ardrone_autonomy::Navdata& msg)
{
    batteryPercent = msg.batteryPercent;//keep track of battery left
    state = msg.state;//keep track of the current state of the drone
    current_altitude = msg.altd;//keep track of the current height

    yaw = msg.rotZ;
    pitch = msg.rotY;
    roll = msg.rotX;
}

void droneNunchuck(const wii_nunchuck::nunchuck& msg)
{
   processJoystick(msg.joy_x,msg.joy_y);
   changeState(msg.button_c,msg.button_z);
}

void changeState(bool c,bool z)
{
    /*
        Change the state to one of land,flying or emergency (beware it will fall straight out of the sky if pressed)
    */
    ros::Rate rate(100);
    if(c && z)
    {
	    if(state<=2)
	    {
	        calibrateDrone();
	    }
	    else
	    {
	        pub_empty_reset.publish(emp_msg);
	    }
	    //reset the drone
    }
    else if(c)
    {
        double start_time = (double)ros::Time::now().toSec();
        while ((double)ros::Time::now().toSec()< start_time+takeoff_time)
        {//takeoff
            pub_empty_takeoff.publish(emp_msg); //launches the drone
            pub_twist.publish(twist_msg_hover); //drone is flat
            ROS_INFO("Taking off");
            ros::spinOnce();
            rate.sleep();
        }//while takeoff
        ROS_INFO("Flight Time Beginning @ %lf",(double)ros::Time::now().toSec());
    }
    else if(z)
    {
        double start_time = (double)ros::Time::now().toSec();
        while ((double)ros::Time::now().toSec()< start_time+takeoff_time)
        {//takeoff
            pub_empty_land.publish(emp_msg); //lands the drone
            ROS_INFO("Landing");
            ros::spinOnce();
            rate.sleep();
        }//while takeoff
        ROS_INFO("Flight Time Ending @ %lf",(double)ros::Time::now().toSec());
    }
}

void processJoystick(uint8_t x,uint8_t y)
{
    if(x < 70)
    {
        ROS_INFO("turning right");
        twist_msg.linear.z = speed;
    }
    if(x > 170)
    {
        ROS_INFO("turning left");
        twist_msg.linear.z = -speed;
    }
    if(y < 100)
    {
        ROS_INFO("moving backwards");
        twist_msg.linear.x = -speed;
    }
    if(y > 150)
    {
        ROS_INFO("Moving forwards");
        twist_msg.linear.x = speed;
    }
    twist_msg.linear.y = 0.0;
    
    pub_twist.publish(twist_msg);
    resetTwist();
}

void calibrateDrone()
{
    std_msgs::String message;
    std::stringstream ss;
    ss << "Calibrating PWM";
    message.data = ss.str();
    
    ros::Rate loop_rate(1000);
    ROS_INFO("Starting Calibrations");
    double start_time = (double)ros::Time::now().toSec();
    while ((double)ros::Time::now().toSec()< start_time+takeoff_time)
    {//takeoff
        recal_trim.publish(emp_msg);
        recal_imu.publish(emp_msg);
        ROS_INFO("Calibrating rotors");
        ros::spinOnce();
        loop_rate.sleep();
    }//while takeoff
    pub_empty_reset.publish(emp_msg);
    ROS_INFO("Calibrated Quadrocopter");
}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"ARDrone_Geofencing");
    try
    {
        ros::NodeHandle node;

        ROS_INFO("Initializing");

	    //loop sleep timer
	    ros::Rate loop_rate(500);//measurement here is in hertz

        //hover message
        twist_msg_hover.linear.x=0.0;
        twist_msg_hover.linear.y=0.0;
        twist_msg_hover.linear.z=0.0;
        twist_msg_hover.angular.x=0.0;
        twist_msg_hover.angular.y=0.0;
        twist_msg_hover.angular.z=0.0;
        // timing variables
        takeoff_time=5.0;
        start_time=0.0;
        fly_time = 20.0;

        // main command message
        twist_msg.linear.x=0.0;
        twist_msg.linear.y=0.0;
        twist_msg.linear.z=0.0;
        twist_msg.angular.x=0.0;
        twist_msg.angular.y=0.0;
        twist_msg.angular.z=0.0;

        /* Ardone_Autonomy Setup */
        /* Publishers */
        pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 4); 
        /* Message queue length is just 1 */
        pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
        /* Message queue length is just 1 */
        pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 4); 
        /* Message queue length is just 1 */
        pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 2); 
        /* Message queue length is just 1 */
        

        recal_trim = node.advertise<std_msgs::Empty>("ardrone/flattrim", 2);
        recal_imu = node.advertise<std_msgs::Empty>("ardrone/imu_recalib", 2);

        /* Subscribers */
        ros::Subscriber droneSensors = node.subscribe("/ardrone/navdata", 10, readDroneSensors);
        ros::Subscriber droneMagSensors = node.subscribe("ardrone/mag",10,readDroneMagSensors);

	    /* Wii Nunchuck subscribers */
        buttonC = node.subscribe("/nunchuck", 4, droneNunchuck);

        bool calibrated = true;

        ros::param::param("~perimeter",perimeter_filename,string("none"));


        ROS_INFO("Starting GeoFencing Main Function");
	    while(ros::ok())
	    {
	        if(calibrated)
	        {
                calibrateDrone();
	            calibrated = false;
	        }
            ros::spinOnce();
            loop_rate.sleep();
	    }
        ROS_INFO("All Done , Cleaning Up Now");
    }
    catch(ros::Exception& e)
    {
        ROS_ERROR("ros error: %s", e.what());
    }
    catch(...)
    {
        ROS_FATAL("unexpected error");
    }

	return 0;
}
