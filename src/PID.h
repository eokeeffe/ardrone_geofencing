#ifndef _PID_H
#define _PID_H

#include <stdio.h>
#include <math.h>
#include <time.h>

/*
*   Simple Proportional Integral Derivative Control
*   Structures and code
*/

/* Simple Yaw PID */
class PID
{
    public:
        PID();
        PID(double p,double i,double d);
        ~PID(){};
        void configure(double p,double i,double d);
        void reset();
        double getCommand(double input);
    private:
        double kp,ki,kd;
        long last_time;
        double last_error;
        double error_sum;
};
/* End of Yaw PID */

#endif /* _PID_H */
