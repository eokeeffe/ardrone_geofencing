/*
*   Geofencing application for ROS
*
*   Created by Evan O'Keeffe
*/

#include "main.h"

using namespace std;


double compass =0.0;
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

coordinate centerPoint()
{
    double x=0.0,y=0.0,z=0.0;

    for(int i=0;i<points.size();i++)
    {
        double lat = radians(points.at(i).x);
        double lon = radians(points.at(i).y);
        x += cos(lat) * cos(lon);
        y += cos(lat) * sin(lon);
        z += sin(lat);
    }
    x /= points.size();
    y /= points.size();
    z /= points.size();

    double Lon = atan2(y, x);
    double hyp = sqrt(x * x + y * y);
    double Lat = atan2(z, hyp);

    ROS_INFO("Center Point:%lf,%lf",degrees(Lat),degrees(Lon));
    return coordinate(degrees(Lat),degrees(Lon));
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
    double a=.0,b=.0,f=.0;
    switch(datum)
    {
        case WGS84:
        {
            a=6378137;
            b=6356752.314245;
            f=1/298.257223563;
            break;
        }
        case GRS80:
        {
            a=6378137;
            b=63567523.14140;
            f=1/298.257222101;
            break;
        }
        case AIRY1830:
        {
            a=6377563.396;
            b=6356256.909;
            f=1/299.3249646;
            break;
        }
        case INTERNATL1924:
        {
            a=6378388;
            b=6356911.646;
            f=1/297;
            break;
        }
        case CLARKEMOD1880:
        {
            a=6378249.145;
            b=6356514.86955;
            f=1/293.465;
            break;
        }
        case GRS67:
        {
            a=6378160;
            b=6356774.719;
            f=1/298.247167;
            break;
        }
        default:
        {
            return coordinate(0.0,0.0);
        }
    };

    double heading = radians(bearing);
    double lat0 = radians(lat);
    double lon0 = radians(lng);

    double sina1 = sin(heading);
    double cosa1 = cos(heading);

    double tanU1 = (1-f) * tan(lat0);
    double cosU1 = 1.0/sqrt(1+tanU1*tanU1);
    double sinU1 = tanU1 * cosU1;

    double sigma1 = atan2(tanU1,cosa1);
    double sina = cosU1 * sina1;
    double cosSqa = 1 - sina*sina;
    double uSq = cosSqa * (a*a-b*b)/(b*b);

    double A = 1+uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
    double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));

    double cos2sigmaM,sinsigma,cossigma,delta_sigma;
    double sigma=distance_meters/(b*A),sigma_transpose;
    int iterations=0;
    do
    {
        cos2sigmaM = cos(2*sigma1+sigma);
        sinsigma = sin(sigma);
        cossigma = cos(sigma);
        delta_sigma = B*sinsigma*(cos2sigmaM+B/4*
            (cossigma*(-1+2*cos2sigmaM*cos2sigmaM)-
            B/6*cos2sigmaM*(-3+4*sinsigma*sinsigma)*
            (-3+4*cos2sigmaM*cos2sigmaM)));
        sigma_transpose = sigma;
        sigma = distance_meters / (b*A) + delta_sigma;
    }
    while(fabs(sigma-sigma_transpose)>1e-12 && ++iterations<200);

    if(iterations>=200)
    {
        return coordinate(NAN,NAN);
    }

    double x = sinU1*sinsigma - cosU1*cossigma*cosa1;
    double lat2 = atan2(sinU1*cossigma+cosU1*sinsigma*cosa1,(1-f)*
        sqrt(sina*sina+x*x));
    double delta = atan2(sinsigma*sina1,cosU1*cossigma - sinU1*sinsigma*cosa1);
    double C = f/16.0*cosSqa*(4+f*(4-3*cosSqa));
    double L = delta - (1-C) * f * sina * (sigma + C*sina*(cos2sigmaM+C*cossigma*(-1+2*cos2sigmaM*cos2sigmaM)));
    double lon2 = fmod((lon0+L+3*M_PI),(2*M_PI)) - M_PI;//normalise to -180...+180

    //double a2 = atan2(sina,-x);
    //a2 = fmod((a2+2*M_PI),(2*M_PI));//normalise to 0...360
    return coordinate(degrees(lat2),degrees(lon2));
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
        current_bearing = 90.0 - (atan2(magX,magY) * 180.0/M_PI);
    }
    else if(magY<0)
    {
        current_bearing = 270 - (atan2(magX,magY) * 180.0/M_PI);
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

    ROS_INFO("Fixed Lat,Lng:%lf,%lf",gps.getLatitude(),gps.getLongitude());
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

void readDroneIMU(const sensor_msgs::Imu& msg)
{
    velx = msg.linear_acceleration.x;
    vely = msg.linear_acceleration.y;
    velz = msg.linear_acceleration.z;
    compass = msg.orientation.z;
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
    //ROS_INFO("safe distance:%lf",min_distance);
    if(velx<0)
    {
        if(current_bearing>180){current_bearing-=180;}
        if(current_bearing<180){current_bearing+=180;}
    }
    coordinate future = predictFutureGPS(min_distance,
        latitude,longitude,degrees(current_bearing));

    //ROS_INFO("Future:%lf,%lf",future.x,future.y);
    if(!isWithin(future.x,future.y))
    {
        double heading = bearing(future.x,future.y,
        center.x,center.y);

        yawPID.reset();
        ros::Rate rate(100);
        while (!isWithin(latitude,longitude))
        {//turn the craft to stay inside perimeter

            //ROS_INFO("Bearing @ %lf",current_bearing);
            //ROS_INFO("Heading @ %lf",heading);

            double eyaw = (heading - current_bearing);
            double uyaw = yawPID.getCommand(radians(eyaw));
            double cyaw = within(uyaw,-1.0,1.0);

            ROS_INFO("cyaw %lf",cyaw);
            ROS_INFO("compass %lf",compass);

            twist_msg.angular.z = cyaw; // turn around
            // move forward every other iteration
            twist_msg.linear.x = speed/4;

            pub_twist.publish(twist_msg); //move the drone
            resetTwist();
            ros::spinOnce(); // let the callbacks activate
            rate.sleep();
        }//while correcting heading and future position

        ROS_INFO("Adjusted Flight Trajectory @ %lf",(double)ros::Time::now().toSec()-total);
        //ROS_INFO("Lat,Lng:%lf,%lf",latitude,longitude);
        return;
    }
    //ROS_INFO("Lat,Lng:%lf,%lf",latitude,longitude);

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
        ros::Subscriber droneIMU = node.subscribe("/ardrone/imu",10,readDroneIMU);
	    /* Wii Nunchuck subscribers */
        buttonC = node.subscribe("/nunchuck", 4, droneNunchuck);

        bool calibrated = true;

        ros::param::param<std::string>("~perimeter",perimeter_filename,"none");
        ros::param::param<double>("~safe_distance",min_distance,5.0);
        ros::param::param<int>("~datum",datum,3);
        read_coordinates(perimeter_filename);
        center = centerPoint();

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
