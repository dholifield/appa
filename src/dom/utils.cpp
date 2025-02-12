#include "dom.h"
namespace dom {

/* PID */
PID::PID(Gains k) : k(k) { reset(0.0); }
PID::PID(double kp, double ki, double kd) : k(kp, ki, kd) { reset(0.0); }

void PID::reset(double error) {
    prev_error = error;
    total_error = 0.0;
}
double PID::update(double error, int dt) {
    double dt_s = (double)dt / 1000.0;
    double derivative = (error - prev_error) / dt_s;
    total_error += error * dt_s;
    prev_error = error;
    return (k.p * error) + (k.i * total_error) + (k.d * derivative);
}

/* Utils */
double to_rad(double deg) { return deg * M_PI / 180; }
double to_deg(double rad) { return rad * 180 / M_PI; }
double limit(double val, double limit) { return val > limit ? limit : val < -limit ? -limit : val; }

} // namespace dom
