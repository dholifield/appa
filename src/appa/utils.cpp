#include "appa.h"

namespace appa {

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

// Ziegler-Nichols PID tuning
void PID::auto_tune(Chassis& chassis, double speed) {
    // move robot forward 12 inches

    // set target 6 inches backward with bang bang controller: forward and backward at speed

    // store millis and max error each time robot crosses target and repeat like 30 times

    // calcuate average oscillation period and amplitude (maybe ignore first few until steady state)

    // Ku = 4A/pi/speed:
    //   PD:  Kp = 0.8*Ku, Kd = 0.1*Ku*Tu
    //   PID: kp = 0.6*Ku, Ki = 1.2*Ku/Tu, Kd = 0.075*Ku*Tu
}

/* Imu */
Imu::Imu(std::initializer_list<uint8_t> ports) {
    for (auto port : ports) {
        imus.emplace_back(port);
    }
}
Imu::Imu(uint8_t port) { imus.emplace_back(port); }

bool Imu::calibrate() {
    for (auto& imu : imus) {
        imu.reset(false);
        imu.set_data_rate(5);
    }
    uint32_t start = pros::millis();
    bool all_calibrated = false;
    while ((pros::millis() - start < 3000) && !all_calibrated) {
        all_calibrated = true;
        for (auto& imu : imus) {
            if (imu.is_calibrating()) all_calibrated = false;
        }
        pros::delay(50);
    }
    return all_calibrated;
}
double Imu::get() {
    double rotation = 0;
    for (auto& imu : imus) {
        rotation -= imu.get_rotation();
    }
    return rotation / imus.size();
}
void Imu::set(double angle) {
    for (auto& imu : imus) {
        imu.set_rotation(-angle);
    }
}

/* Options */
Options Options::defaults() {
    return Options(
        AUTO, AUTO, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, Gains(), Gains(), false, false, false);
}

Options Options::operator<<(const Options& other) const {
    Options result = *this;

    if (other.dir) result.dir = other.dir;
    if (other.turn) result.turn = other.turn;
    if (other.speed) result.speed = other.speed;
    if (other.accel) result.accel = other.accel;
    if (other.lead) result.lead = other.lead;
    if (other.lookahead) result.lookahead = other.lookahead;
    if (other.exit) result.exit = other.exit;
    if (other.offset) result.offset = other.offset;
    if (other.settle) result.settle = other.settle;
    if (other.timeout) result.timeout = other.timeout;
    if (other.lin_PID) result.lin_PID = other.lin_PID;
    if (other.ang_PID) result.ang_PID = other.ang_PID;
    if (other.thru) result.thru = other.thru;
    if (other.relative) result.relative = other.relative;
    if (other.async) result.async = other.async;
    if (other.exit_fn) result.exit_fn = other.exit_fn;

    return result;
}
Options Options::operator>>(const Options& other) const { return other << *this; }
void Options::operator<<=(const Options& other) { *this = *this << other; }
void Options::operator>>=(const Options& other) { *this = *this >> other; }

Options MoveConfig::options() const {
    return Options{.speed = speed,
                   .lead = lead,
                   .lookahead = lookahead,
                   .exit = exit,
                   .lin_PID = lin_PID,
                   .ang_PID = ang_PID};
}
Options TurnConfig::options() const {
    return Options{.speed = speed, .exit = exit, .ang_PID = ang_PID};
}

/* Point */
Point Point::operator+(const Point& other) const { return Point({x + other.x, y + other.y}); }
Point Point::operator-(const Point& other) const { return Point({x - other.x, y - other.y}); }
Point Point::operator*(double scalar) const { return Point({x * scalar, y * scalar}); }
void Point::operator+=(const Point& other) {
    x += other.x;
    y += other.y;
}
void Point::operator-=(const Point& other) {
    x -= other.x;
    y -= other.y;
}
void Point::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
}
void Point::operator=(const Point& p) {
    x = p.x;
    y = p.y;
}
double Point::dist(const Point& other) const {
    Point diff = *this - other;
    return sqrt(diff.x * diff.x + diff.y * diff.y);
}
double Point::angle(const Point& other, double offset) const {
    Point diff = other - *this;
    return std::remainder(atan2(diff.y, diff.x) - offset, 2 * M_PI);
}
Point Point::rotate(double theta) const {
    if (theta == 0) return *this;
    double sine = sin(theta);
    double cosine = cos(theta);
    return {x * cosine - y * sine, x * sine + y * cosine};
}

/* Pose */
Pose::operator Point() const { return Point{x, y}; }
Point Pose::p() const { return {x, y}; }
Pose Pose::operator+(const Point& p) const { return Pose(x + p.x, y + p.y, theta); }
Pose Pose::operator-(const Point& p) const { return Pose(x - p.x, y - p.y, theta); }
void Pose::operator+=(const Point& p) {
    x += p.x;
    y += p.y;
}
void Pose::operator-=(const Point& p) {
    x -= p.x;
    y -= p.y;
}
void Pose::operator=(const Point& p) {
    x = p.x;
    y = p.y;
}

Pose Pose::operator+(const Pose& p) const { return Pose(x + p.x, y + p.y, theta + p.theta); }
Pose Pose::operator-(const Pose& p) const { return Pose(x - p.x, y - p.y, theta - p.theta); }
void Pose::operator+=(const Pose& p) {
    x += p.x;
    y += p.y;
    theta += theta;
}
void Pose::operator-=(const Pose& p) {
    x -= p.x;
    y -= p.y;
    theta -= theta;
}
void Pose::operator=(const Pose& p) {
    x = p.x;
    y = p.y;
    theta = p.theta;
}

double Pose::dist(const Point& other) const { return p().dist(other); }
double Pose::angle(const Point& other) const { return p().angle(other, theta); }
Point Pose::project(double d) const { return p() + Point{d * cos(theta), d * sin(theta)}; }

/* Utils */
double to_rad(double deg) { return deg * M_PI / 180; }
double to_deg(double rad) { return rad * 180 / M_PI; }
double limit(double val, double limit) {
    return val > limit ? limit : (val < -limit ? -limit : val);
}

} // namespace appa
