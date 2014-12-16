#include <PID.h>

PID::PID(double p,double i,double d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::configure(double p,double i,double d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::reset()
{
    last_error = FP_INFINITE;
    last_time = 0.0;
    error_sum = 0.0;
}

double PID::getCommand(double input)
{
    int timer = (int)time(NULL);
    double dt = (timer - last_time) / 1000;

    double de = 0.0;
    if(last_time!=0.0)
    {
        // Computing error derivation
        if(last_error < FP_INFINITE)
        {
            de = (input - last_error) / dt;
        }

        error_sum += input * dt;
    }

    last_time = timer;
    last_error = input;

    double output = kp * input +
                    ki * error_sum +
                    kd * de;
    return output;
}
