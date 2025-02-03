#include "dom.h"

/* PID */
PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) { reset(0.0); }
PID::PID(PIDConst k) : kp(k.p), ki(k.i), kd(k.d) { reset(0.0); }

void PID::reset(double error) {
    prev_error = error;
    total_error = 0.0;
}

double PID::update(double error, double dt) {
    double derivative = (error - prev_error) / dt;
    total_error += error * dt;
    prev_error = error;
    return (kp * error) + (ki * total_error) + (kd * derivative);
}

/* Odom */


/* Chassis */
