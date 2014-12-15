#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/Core>

using namespace Eigen;

class Kalman
{
    public:
        Kalman(){}
        virtual ~Kalman(){}
    private:
};

class ExtendedKalman : public Kalman
{
    public:
        ExtendedKalman();
        virtual ~ExtendedKalman();

        Vector3d getstate();
        Matrix3d confidence();
        void reset();
        void predict(double pitch,double roll,double yaw,double vx,double vy);
        void correct(Vector3d measure,Vector3d pose);
    private:
        float delta_t;//default demo mode, 15 updates a second
        Matrix3d sigma;
        Matrix3d q;
        Matrix3d r;
        Vector3d kstate;
        double last_yaw;

        double normAngle(double rad);
};

class UnscentedKalman : public Kalman
{
    public:
        UnscentedKalman();
        virtual ~UnscentedKalman();
    private:
};

class KalmanGPS : public Kalman
{
    /*
        lat_measurement_degrees
            new measurement of lattidude
        lng_measurement
            new measurement of longitude
        accuracy
            measurement of 1 standard deviation error in metres
        TimeStamp_milliseconds
            time of measurement
    */
    public:
        KalmanGPS(){minAccuracy  = 1;}
        KalmanGPS(float q_metres_per_second);
        virtual ~KalmanGPS(){};

        long get_TimeStamp();
        double get_latitude();
        double get_longitude();

        void SetState(double lat, double lng,
            float accuracy, long TimeStamp_milliseconds);
        void Process(double lat_measurement, double lng_measurement,
            float accuracy, long TimeStamp_milliseconds);
        double getLatitude();
        double getLongitude();
    private:
        float minAccuracy;
        float q_metres_per_second;
        long timestamp_milliseconds;
        double latitude,longitude;
        float variance;
        // variance = P matrix Negative means object uninitialised.
        // NB: units irrelevant, as long as same units used throughout
};
#endif
