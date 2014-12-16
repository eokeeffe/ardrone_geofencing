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
}

void PID::reset()
{
    this->last_error = FP_INFINITE;
    this->last_time = 0.0;
    this->error_sum = 0.0;
}

double PID::getCommand(double input)
{
    double timer = time(NULL);
    double dt = (timer - this->last_time) / 1000;

    double de = 0.0;
    if(this->last_time!=0.0)
    {
        // Compute error derivation
        if(this->last_error < FP_INFINITE)
        {
            de = (input - this->last_error) / dt;
        }

        this->error_sum += input * dt;
    }

    this->last_time = timer;
    this->last_error = input;

    double output = kp * input +
                    ki * error_sum +
                    kd * de;
    return output;
}
