#include <PID.h>

PID::PID(double p,double i,double d)
{
    configure(p,i,d);
}

void PID::configure(double p,double i,double d)
{
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->prev = .0;
    this->i = .0;
    this->d = .0;
    this->last_time = 0;
}

void PID::reset()
{
    this->i = .0;
    this->d = .0;
    this->last_time = 0;
    this->prev = .0;
}

double PID::getCommand(double input)
{
    /*
        Had to use the new chrono for millisecond
        time to prevent the division by zero problem
        with the dt value
    */
    std::clock_t timer = std::clock();
    double dt = 1000.0 * (timer-this->last_time) / CLOCKS_PER_SEC;
    this->i += dt*input;
    this->d = (input-this->prev)/dt;
    this->prev = input;
    this->last_time = timer;

    return (this->kp*input)+(this->ki*this->i)+(this->kd*this->d);
}

AdvController::AdvController(double p,double i,double d,
double clamp_lo,double clamp_hi,double smooth)
{
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->prev = .0;
    this->last_time = 0;

    this->i = .0;
    this->d = .0;

    this->unclamped = true;
    this->clamp_lo = clamp_lo;
    this->clamp_hi = clamp_hi;
    this->alpha = smooth;
}

void AdvController::reset()
{
    this->prev = .0;
    this->last_time = 0;

    this->i = .0;
    this->d = .0;
}

double AdvController::getCommand(double input)
{
    std::clock_t timer = std::clock();
    double dt = 1000.0 * (timer-this->last_time) / CLOCKS_PER_SEC;

    if( this->unclamped )
    {
        this->i += dt*input;
    }
    this->d = (this->alpha*(input-this->prev)/dt + (1.0-this->alpha)*this->d);
    double u = this->kp * input + this->ki * this->i + this->kd * this->d;
    this->unclamped = (this->clamp_lo<u<this->clamp_hi);
    this->prev = input;
    return u;
}
