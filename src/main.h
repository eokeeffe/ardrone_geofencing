#ifndef _MAIN_H
#define _MAIN_H

#include <ros/ros.h>
#include <iostream>
/* Math Libraries */
#include <math.h>
#include <float.h>
#include <geometry_msgs/Vector3Stamped.h>
/* ARDrone Headers */
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_gps.h>
#include <sensor_msgs/Imu.h>
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
/* Include PID controller */
#include <PID.h>
/* Include Kalman Filters */
#include <KF.h>

// Drone variables
/* Drone State and battery */
float batteryPercent;
int state;

/* Control Variables */
float speed = 0.5;
float altitude,current_altitude,magX,magY,magZ,yaw,pitch,roll;
double longitude=.0,latitude=.0,elevation=.0,gps_speed=.0;
float previous_longitude=.0,previous_latitude=.0;
double current_bearing=.0,min_distance=5.0;
float velx=.0,vely=.0,velz=.0;
int reset_count = 0;
/* Flight time recording */
float takeoff_time,start_time,fly_time,total;
/* GPS kalman init */
bool gps_initial = false;
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

void readDroneMagSensors(const geometry_msgs::Vector3Stamped& msg);
void readDroneGPS(const ardrone_autonomy::navdata_gps& msg);
void readDroneSensors(const ardrone_autonomy::Navdata& msg);
void droneNunchuck(const wii_nunchuck::nunchuck& msg);
void changeState(bool c,bool z,bool c2,bool z2);
void processJoystick(uint8_t x,uint8_t y,uint8_t x2,uint8_t y2);
void calibrateDrone();
//reset the drone command message to hover
void resetTwist();
//read the gps coordinates
void read_coordinates(std::string filename);
inline bool file_exists(const std::string& name);
//check if the geo codes lie within the polygon
bool isWithin(double lon,double lat);
//get distance between GPS points
double pointDistance(double lat1,double long1,double lat2,double long2);
double bearing(double lat1,double long1,double lat2,double long2);

struct coordinate
{
    double x,y;
    coordinate(){}
    coordinate(double input_x,double input_y)
    {
        x = input_x;
        y = input_y;
    }

}center;

//Predict the future GPS position from init point with bearing
coordinate predictFutureGPS(double lat,double lng,double bearing);
double degrees(double radians);
double radians(double deg);
//get the center point to the GPS points
coordinate centerPoint();

/* Get heading from Previous GPS Point */

// GeoFence coordinates
std::vector<coordinate> points;
// Center point
//struct coordinate center;

/* Kalman Filter Objects */
KalmanGPS gps;
PID yawPID(1.0,0.0,0.30);

/* Ellipsoid Declarations */
enum Datum
{
    WGS84,
    GRS80,
    AIRY1830,
    INTERNATL1924,
    CLARKEMOD1880,
    GRS67
};
int datum;

#endif
