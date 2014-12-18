#ifndef _PID_H
#define _PID_H

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <algorithm> // copy algorithm
#include <iterator>  // ostream_iterator iterator
#include <vector>    // vector class-template definition
#include <chrono>

using namespace std;
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
        double i=.0,d=.0,prev=.0;
};
/* End of Single Value PID */

/*
    Advanced PID Controller
*/
class AdvController
{
    public:
        AdvController(double p,double i,double d,
        double clamp_lo,double clamp_hi,double smooth);
        ~AdvController();
        void reset();
        double getCommand(double input);
    private:
        double kp,ki,kd;
        std::clock_t last_time;
        double i=.0,d=.0,prev=.0;

        bool unclamped;
        double clamp;
        double clamp_lo,clamp_hi;
        double alpha;
};
/* End of Advanced PID Controller */

/* Filters For Controllers */
class Limiter
{
    public:
        Limiter(double lo,double hi)
        {
            this->lo=lo;
            this->hi=hi;
        }
        ~Limiter();
        double work(double x)
        {
            return max(this->lo,min(x,this->hi));
        }
    private:
        double lo,hi;
};

class Discretizer
{
    public:
        Discretizer(double binwidth)
        {
            this->binwidth=binwidth;
        }
        ~Discretizer(){};
        double work(double u)
        {
            return this->binwidth*((int)(u/this->binwidth));
        }
    private:
        double binwidth;
};

class Hysteresis
{
    public:
        Hysteresis(double threshold)
        {
            this->threshold=threshold;
            this->prev=.0;
        }
        ~Hysteresis(){};
        double work(double u)
        {
            double y = this->prev;
            if(fabs(u-this->prev)>this->threshold)
            {
                y = u;
                this->prev = u;
            }
            return y;
        }
    private:
        double threshold,prev;
};

class FixedFilter
{
    public:
        FixedFilter(int n){this->n=n;};
        ~FixedFilter(){};
        double work(double x)
        {
            data.push_back(x);
            if(data.size()>this->n)
            {
                data.erase( data.begin() );
            }

            return (std::accumulate(data.begin(), data.end(), 0.0)/data.size());
        }
    private:
        int n;
        vector<double> data;
};

class Integrator
{
    public:
        Integrator(){this->data=.0;this->dt=.0;}
        ~Integrator(){};
        double work(double u)
        {
            std::clock_t timer = std::clock();
            double dt = 1000.0 * (timer-this->last_time) / CLOCKS_PER_SEC;
            this->data += u;
            this->last_time = timer;
            return dt*this->data;
        }
    private:
        double data;
        std::clock_t last_time;
        double dt;
};

class RecursiveFilter
{
    public:
        RecursiveFilter(double alpha)
        {
            this->alpha=alpha;
            this->y=0;
        }
        ~RecursiveFilter(){};
        double work(double x)
        {
            this->y = this->alpha*x + (1-this->alpha) * this->y;
            return this->y;
        };
    private:
        double alpha,y;
};
/* End of Filters */

/* Step Points */
bool impulse(double t,double t0,double DT)
{
    /* test for floating point or integer time */
    if(abs(t-t0)<DT){return true;}
    return false;
}
bool step(double t,double t0)
{
    if(t>t0){return true;}
    return false;
}
bool double_step(double t,double t0,double t1)
{
    if(t>=t0&&t<t1){return true;}
    return false;
}
bool pulses(double t,double t0,double tp)
{
    if(t>=t0&&fmod((t-t0),tp)==0){return true;}
    return false;
}
double harmonic(double t,double t0,double tp)
{
    if(t>=t0){return sin(2*M_PI*(t-t0)/tp);}
    return 0;
}
bool relay(double t,double t0,double tp)
{
    if(t>=t0)
    {
        if(ceil(sin(2*M_PI*(t-t0)/tp))>0)
        {
            return true;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}
/* End of SetPoints */

#endif /* _PID_H */
