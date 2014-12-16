#include "KF.h"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

/* Extended Kalman Filter */
ExtendedKalman::ExtendedKalman()
{
    delta_t = 1/15;// 15Hz is the default update frequency
    kstate(0) = 0.0;
    kstate(1) = 0.0;
    kstate(2) = 0.0;
    sigma = Matrix3d::Identity();
    q = Matrix3d::Zero();
    q(0,0) = 0.0003;
    q(1,1) = 0.0003;
    q(2,2) = 0.0001;
    r = Matrix3d::Zero();
    r(0,0) = 0.3;
    r(1,1) = 0.3;
    r(2,2) = 0.3;
    last_yaw = 0.0;
}

ExtendedKalman::~ExtendedKalman()
{

}

Vector3d ExtendedKalman::getstate()
{
    return this->kstate;
}

Matrix3d ExtendedKalman::confidence()
{
    return this->sigma;
}

void ExtendedKalman::reset()
{
    this->kstate(0) = 0.0;
    this->kstate(1) = 0.0;
    this->kstate(2) = 0.0;
    this->sigma = Matrix3d::Identity();
    this->q = Matrix3d::Zero();
    this->q(0,0) = 0.0003;
    this->q(1,1) = 0.0003;
    this->q(2,2) = 0.0001;
    this->r = Matrix3d::Zero();
    this->r(0,0) = 0.3;
    this->r(1,1) = 0.3;
    this->r(2,2) = 0.3;
    this->last_yaw = 0.0;
}

void ExtendedKalman::predict(double pitch,double roll,double yaw,double vx,double vy)
{
    double dt = delta_t;

    //check yaw motion
    if(this->last_yaw < 0.0)
    {
        this->last_yaw = yaw;
        return;
    }

    this->last_yaw = yaw;
    Vector3d kstate1;

    kstate1(0) = this->kstate(0) + (vx*dt) * cos(this->kstate(2)) -
                           (vy*dt) * sin(this->kstate(2));
    kstate1(1) = this->kstate(1) + (vx*dt) * sin(this->kstate(2)) +
                           (vy*dt) * cos(this->kstate(2));
    kstate1(2) = this->kstate(2) + (yaw - this->last_yaw);

    // Normalize the yaw value
    this->kstate(2) = atan2(sin(this->kstate(2)),cos(this->kstate(2)));

    //compute the G term
    Matrix3d G = Matrix3d::Zero();
    G << 1, 0, -1 * sin(this->kstate(2)) * (vx*dt) - cos(this->kstate(2)) * (vy*dt),
         0, 1, cos(this->kstate(2)) * (vx*dt) - sin(this->kstate(2)) * (vy*dt),
         0, 0, 1;

    // Compute the new sigma
    this->sigma = G * this->sigma * (G.transpose() + this->q);
}

void ExtendedKalman::correct(Vector3d measure,Vector3d pose)
{
    /*
        measure x: x position
        measure y: y position
        measure yaw: yaw rotation

        pose x: x-position
        pose y: y-position
        pose yaw: yaw rotation
    */
    Vector3d kstate1;
    double psi = this->kstate(0);
    Vector3d s;
    s(0) = this->kstate(0);
    s(1) = this->kstate(1);
    s(2) = this->kstate(2);

    measure(2) = normAngle(measure(2));
    Vector3d m;
    m(0) = measure(0);
    m(1) = measure(1);
    m(2) = measure(2);

    double z1 = cos(psi) * (pose(0) - this->kstate(0))
        + sin(psi)
        * (pose(1) - this->kstate(1));
    double z2 = -1 * sin(psi) *
        (pose(0) - this->kstate(0))
        + cos(psi)
        * (pose(1) - this->kstate(1));
    double z3 = pose(2) - psi;
    Vector3d z;
    z(0) = z1;
    z(1) = z2;
    z(2) = z3;

    //Compute the error
    double e1 = measure(0);
    double e2 = measure(1);
    double e3 = measure(2);

    Vector3d e;
    e(0) = e1;
    e(1) = e2;
    e(2) = e3;

    Matrix3d H;
    H << -cos(psi), -sin(psi), sin(psi)*(this->kstate(0)-pose(0))-cos(psi)*(this->kstate(1)-pose(1)),
         sin(psi), -cos(psi), cos(psi) * (this->kstate(0)-pose(0)) - sin(psi) * (this->kstate(1)-pose(1)),
         0.0, 0.0, -1;

    // Compute the Kalman Gain
    Matrix3d Ht = H.transpose();
    Matrix3d K = Matrix3d::Zero();
    K = sigma * Ht * ( H * sigma * Ht + r).inverse();

    Vector3d err;
    err(0) = e1;
    err(1) = e2;
    err(2) = e3;

    Vector3d c = K * err;
    this->kstate(0) +=  c(1);
    this->kstate(1) +=  c(2);

    sigma = (Matrix3d::Identity() - (K*H)) * sigma;
}

double ExtendedKalman::normAngle(double rad)
{
    while(rad > M_PI){ rad -= 2 * M_PI;}
    while(rad <-M_PI){ rad += 2 * M_PI;}
    return rad;
}
/* End Extended Kalman Filter */


/* KF GPS*/

KalmanGPS::KalmanGPS(float q_metres_per_second)
{
    this->minAccuracy  = q_metres_per_second;
}

double KalmanGPS::getLatitude()
{
    return this->latitude;
}

double KalmanGPS::getLongitude()
{
    return this->longitude;
}

void KalmanGPS::SetState(double lat, double lng,
    float accuracy, long TimeStamp_milliseconds)
{
    this->latitude = lat;
    this->longitude = lng;
    this->variance = accuracy * accuracy;
    this->timestamp_milliseconds = TimeStamp_milliseconds;
}

void KalmanGPS::Process(double lat_measurement, double lng_measurement,
    float accuracy, long TimeStamp_milliseconds)
{
    if(accuracy < this->minAccuracy)
    {
        accuracy = this->minAccuracy;
    }

    if(variance < 0)
    {
        this->timestamp_milliseconds = TimeStamp_milliseconds;
        this->latitude = lat_measurement;
        this->longitude = lng_measurement;
        this->variance = accuracy * accuracy;
    }
    else
    {
        //else apply Kalman Filter methodology

        long timeinc_milliseconds = TimeStamp_milliseconds - this->timestamp_milliseconds;
        if(timeinc_milliseconds > 0)
        {
            //time has moved on, so the uncertainty in
            //current position increases
            this->variance += timeinc_milliseconds *
            q_metres_per_second * q_metres_per_second / 1000;
            this->timestamp_milliseconds = TimeStamp_milliseconds;

            //TO DO:Use velocity info to get better estimate of
            //current position
        }

        // Kalman gain matrix K = covariance * inverse(covariance
        // + MeasurementVariance)
        float K = this->variance / (this->variance + accuracy * accuracy);
        // apply K
        this->latitude  += K * (lat_measurement - latitude);
        this->longitude += K * (lng_measurement - longitude);
        // new Covariance matrix is (IdentityMatrix - K) * Covariance
        this->variance = (1-K) * variance;
    }
}
/* End KF GPS */

/* Unscented KF */

/* End of Unscented KF */
