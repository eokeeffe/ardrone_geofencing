/*
*   Geofencing application for ROS
*
*   Created by Evan O'Keeffe
*/

#include "main.h"

using namespace std;

double degrees(double radians)
{
    return radians * 180.0/M_PI;
}

double radians(double deg)
{
    return deg*M_PI/180.0;
}

bool compare(coordinate a,coordinate b)
{
    return a.y > b.y;
}

void closestPoints(double lat,double lng)
{
    distance_index.clear();

    for(int i=0;i<points.size();i++)
    {
        distance_index.push_back(
            coordinate(i ,pointDistance(lat,
                lng,points.at(i).x,points.at(i).y))
        );
    }
    //sort them so the furthest indexes are first
    std::sort (distance_index.begin(), distance_index.end(), compare);
}

coordinate predictFutureGPS(double distance_meters,double lat,double lng,double bearing)
{
    /*
        Manipulation of GPS distance_meters formula rewritten
        to find the future gps point with a known origin
        and known bearing
        @distance_meters=distance_meters from this point in meters
        @lat=current latitude
        @lng=current longitude
        @bearing=current bearing
    */
    int radiusEarthMeters = 6371 * 1000;
    bearing = degrees(bearing);
    double newlat = radians((distance_meters/radiusEarthMeters)
        * cos(bearing)) + lat;
    double newlng = radians(
        (distance_meters/(radiusEarthMeters*
            sin(degrees(newlat)))) * sin(bearing)) + lng;
    // coordinates from origin with bearing
    return coordinate(newlat,newlng);
}

bool isWithin(double lat,double lng)
{
    /*
        GPS Point Containment
        Complex Polygon Point intersection method
    */
    int sides = points.size();
    int j= sides-1;
    bool pointStatus = false;
    for(int i=0;i<sides;i++)
    {
        double lati = points.at(i).x;
        double latj = points.at(j).x;
        double lngi = points.at(i).y;
        double lngj = points.at(j).y;

        if(lngi<lng && lngj >= lng || lngj<lng && lngi >= lng)
        {
            if (lati+(lng-lngi)/(lngj-lngi)*(latj-lati) < lat)
            {
                pointStatus = !pointStatus;
            }
        }
        j = i;
    }
    return pointStatus;
}

double pointDistance(double lat1,double long1,double lat2,double long2)
{
    //cosine distance law
    //returns distance in Km

    return acos( sin(lat1) * sin(lat2) +
     cos(lat1)* cos(lat2) * cos(long2-long1) ) * 6371;
}

double bearing(double lat1,double long1,double lat2,double long2)
{
    /*
        Bearing or heading calcuation based on
        2 sets of GPS points
    */
    double y = sin(degrees(long2-long1)) * cos(lat2);
    double x = cos(degrees(lat1))*sin(degrees(lat2))-sin(degrees(lat1))*
    cos(degrees(lat2))*cos(lat2-lat1);
    return degrees(atan2(y,x));
}

inline bool file_exists(std::string& name)
{
    return ( access( name.c_str(), F_OK ) != -1 );
}

void read_coordinates(std::string filename)
{
    if(filename=="none")
    {
        ROS_INFO("GPS Perimeter File not given");
        ROS_INFO("Shutting down");
        ros::shutdown();
    }
    if(file_exists(filename))
    {
        ROS_INFO("Parsing GPS coordinates");
        string line;
        ifstream gps_file (filename.c_str());
        if (gps_file.is_open())
        {
            double x=.0,y=.0;
            while ( getline (gps_file,line) )
            {
                if (strstr(line.c_str(), "#") != NULL)
                {// comment line , skip these
                    continue;
                }
                sscanf(line.c_str(),"%lf,%lf%*s",&x,&y);
                ROS_INFO("Lat:%lf,Long:%lf",x,y);
                points.push_back(coordinate(x,y));
            }
            gps_file.close();
        }
        ROS_INFO("GPS coordinates have been parsed");
        int num_points = points.size();
        ROS_INFO("%d coordinates parsed",num_points);
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
    twist_msg.angular.x=1.0;
    twist_msg.angular.y=1.0;
    twist_msg.angular.z=0.0;
}

void readDroneMagSensors(const geometry_msgs::Vector3Stamped& msg)
{
    magX = msg.vector.x;
    magY = msg.vector.y;
    magZ = msg.vector.z;

    //needs to have magnetic deviation and inclination
    //solution included
    if(magY>0)
    {
        current_bearing = 90.0 - atan2(magX,magY) * 180.0/M_PI;
    }
    else if(magY<0)
    {
        current_bearing = 270 - atan2(magX,magY) * 180.0/M_PI;
    }
    else if(magY<0 && magX<0)
    {
        current_bearing = 180.0;
    }
    else if(magY<0 && magX>0)
    {
        current_bearing = 0.0;
    }
}

void readDroneGPS(const ardrone_autonomy::navdata_gps& msg)
{
    if(!gps_initial)
    {
        gps.SetState(msg.latitude,msg.longitude,2,msg.lastFrameTimestamp);
        gps_initial = true;
    }
    gps.Process(msg.latitude,msg.longitude,2,msg.lastFrameTimestamp);
    latitude = msg.latitude;
    longitude = msg.longitude;

    //latitude = gps.getLatitude();
    //longitude = gps.getLongitude();

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
   changeState(msg.button_c,msg.button_z,msg.button_c2,msg.button_z2);
   processJoystick(msg.joy_x,msg.joy_y,msg.joy_x2,msg.joy_y2);
}

void changeState(bool c,bool z,bool c2,bool z2)
{
    /*
        Change the state to one of land,flying or emergency (beware it will fall straight out of the sky if pressed)
    */
    ros::Rate rate(100);
    if(c && c2 && z && z2)
    {
	    if(state<=2)
	    {
	        calibrateDrone();
	    }
        //reset the drone
	    pub_empty_reset.publish(emp_msg);
    }
    else if(c && c2)
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
        total = (double)ros::Time::now().toSec();
        ROS_INFO("Flight Time Beginning");
    }
    else if(z&&z2)
    {
        double start_time = (double)ros::Time::now().toSec();
        while ((double)ros::Time::now().toSec()< start_time+takeoff_time)
        {//takeoff
            pub_empty_land.publish(emp_msg); //lands the drone
            ROS_INFO("Landing");
            ros::spinOnce();
            rate.sleep();
        }//while takeoff
        total = (double)ros::Time::now().toSec() - total;
        ROS_INFO("Flight Time Ending after @ %lf",total);
    }
}

double within(double x,double min,double max)
{
    if(x<min)
    {
        return min;
    }
    else if(x > max)
    {
        return max;
    }
    else
    {
        return x;
    }
}

void processJoystick(uint8_t x,uint8_t y,uint8_t x2,uint8_t y2)
{
    // get the position of the GPS at a certain distance
    // with known gps location and bearing
    coordinate future = predictFutureGPS(min_distance,
        latitude,longitude,current_bearing);

    ROS_INFO("Future:%lf,%lf",future.x,future.y);
    if(!isWithin(future.x,future.y))
    {
        closestPoints(future.x,future.y);
        int index = distance_index[0].x;
        double heading = bearing(future.x,future.y,
            points[index].x,points[index].y);

        ros::Rate rate(100);
        while (!isWithin(latitude,longitude))
        {//turn the craft to stay inside perimeter

            ROS_INFO("Bearing @ %lf",current_bearing);
            ROS_INFO("Heading @ %lf",heading);

            double eyaw = heading - current_bearing;
            double uyaw = yawPID.getCommand(eyaw);
            double cyaw = within(uyaw,-1.0,1.0);

            ROS_INFO("Turning %lf",cyaw);

            twist_msg.angular.z = cyaw; // turn around
            twist_msg.linear.x = speed/5; // move forward

            pub_twist.publish(twist_msg); //move the drone
            resetTwist();
            ros::spinOnce(); // let the callbacks activate
            rate.sleep();
        }//while correcting heading and future position

        ROS_INFO("Adjusted Flight Trajectory @ %lf",total);
        ROS_INFO("Lat,Lng:%lf,%lf",latitude,longitude);

        return;
    }
    ROS_INFO("Lat,Lng:%lf,%lf",latitude,longitude);

    if(x < 70)
    {
        ROS_INFO("turning right");
        twist_msg.angular.z = speed;
    }
    if(x > 170)
    {
        ROS_INFO("turning left");
        twist_msg.angular.z = -speed;
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

    if(x2 > 170)
    {
        ROS_INFO("Strafing Right");
        twist_msg.linear.y = -speed;
    }
    else if(x2 < 70)
    {
        ROS_INFO("Strafing Left");
        twist_msg.linear.y = speed;
    }
    if(y2 > 170)
    {
        ROS_INFO("Moving Up");
        twist_msg.linear.z = speed;
    }
    else if(y2 < 70)
    {
        ROS_INFO("Moving Down");
        twist_msg.linear.z = -speed;
    }

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
        ROS_DEBUG_ONCE("Calibrating rotors");
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
        twist_msg_hover.angular.x=1.0;
        twist_msg_hover.angular.y=1.0;
        twist_msg_hover.angular.z=0.0;
        // timing variables
        takeoff_time=5.0;
        start_time=0.0;
        fly_time = 20.0;
        total = 0.0;

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
        ros::Subscriber droneMagSensors = node.subscribe("/ardrone/mag",10,readDroneMagSensors);
        ros::Subscriber droneGPS = node.subscribe("/ardrone/navdata_gps",10,readDroneGPS);

	    /* Wii Nunchuck subscribers */
        buttonC = node.subscribe("/nunchuck", 4, droneNunchuck);

        bool calibrated = true;

        ros::param::param<std::string>("~perimeter",perimeter_filename,"none");
        ros::param::param<double>("~safe_distance",min_distance,5.0);
        read_coordinates(perimeter_filename);


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
