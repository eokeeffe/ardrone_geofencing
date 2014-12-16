#ifndef _PID_H
#define _PID_H

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <chrono>
using namespace std::chrono;

/*
*   Simple Proportional Integral Derivative Control
*   Structures and code
*/

/* Single Value PID */
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
        std::clock_t last_time;
        double last_error;
        double error_sum;
};
/* End of Single Value PID */

#endif /* _PID_H */
