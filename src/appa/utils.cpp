#include "appa.h"

namespace appa {

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

/* Imu */
Imu::Imu(std::initializer_list<uint8_t> ports) {
    for (auto port : ports) {
        imus.emplace_back(port);
    }
}
Imu::Imu(uint8_t port) { imus.emplace_back(port); }

bool Imu::calibrate() {
    if (imus.empty()) return true;
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

/* Encoder */
Encoder::Encoder(int8_t port) : enc(abs(port), abs(port) + 1, port < 0) {}
Encoder::Encoder(uint8_t expander, int8_t port)
    : enc({expander, abs(port), abs(port) + 1}, port < 0) {}
double Encoder::get_value() { return enc.get_value(); }

/* Tracker */
Tracker::Tracker(Encoder x_encoder, Encoder y_encoder, Imu imu_port, double tpu)
    : type(Type::TWO_WHEEL_IMU), imu(std::move(imu_port)), tpu(tpu), angle_offset(0.0) {
    trackers.emplace_back(std::move(x_encoder));
    trackers.emplace_back(std::move(y_encoder));
}

Tracker::Tracker(Encoder lx_encoder, Encoder rx_encoder, Encoder y_encoder, double tpu,
                 double width)
    : type(Type::THREE_WHEEL), tpu(tpu), width(width), angle_offset(0.0) {
    trackers.emplace_back(std::move(lx_encoder));
    trackers.emplace_back(std::move(rx_encoder));
    trackers.emplace_back(std::move(y_encoder));
}

Pose Tracker::get() {
    switch (type) {
    case Type::TWO_WHEEL_IMU: {
        double x = trackers[0].get_value() * tpu;
        double y = trackers[1].get_value() * tpu;
        double theta = to_rad(imu.get());
        return Pose(x, y, theta);
        break;
    }
    case Type::THREE_WHEEL: {
        double l = trackers[0].get_value() * tpu;
        double r = trackers[1].get_value() * tpu;
        double y = trackers[2].get_value() * tpu;
        double theta = (r - l) / width + angle_offset;
        double x = (r + l) / 2;
        return Pose(x, y, theta);
        break;
    }
    }
    return Pose();
}

void Tracker::set_angle(double angle) {
    if (type == Type::TWO_WHEEL_IMU) {
        imu.set(angle);
    } else if (type == Type::THREE_WHEEL) {
        angle_offset = angle - get().theta;
    }
}

/* ExitSpeed */
bool ExitSpeed::check(Pose dp, int dt) {
    if (settle == 0) return false;
    double d_d = dp.x * dp.x + dp.y * dp.y;
    double d_a = fabs(dp.theta);
    if (d_d < linear * linear && d_a < to_rad(angular)) timer += dt;
    else timer = 0;
    if (timer > settle) {
        timer = 0;
        return true;
    }
    return false;
}

/* Options */
Options Options::operator<<(const Options& other) const {
    Options result = *this;

    if (other.dir) result.dir = other.dir;
    if (other.turn) result.turn = other.turn;
    if (other.thru) result.thru = other.thru;
    if (other.relative) result.relative = other.relative;
    if (other.async) result.async = other.async;
    if (other.speed) result.speed = other.speed;
    if (other.accel) result.accel = other.accel;
    if (other.lin_PID) result.lin_PID = other.lin_PID;
    if (other.ang_PID) result.ang_PID = other.ang_PID;
    if (other.lead) result.lead = other.lead;
    if (other.lookahead) result.lookahead = other.lookahead;
    if (other.lin_exit) result.lin_exit = other.lin_exit;
    if (other.ang_exit) result.ang_exit = other.ang_exit;
    if (other.ang_dz) result.ang_dz = other.ang_dz;
    if (other.offset) result.offset = other.offset;
    if (other.exit_speed) result.exit_speed = other.exit_speed;
    if (other.settle) result.settle = other.settle;
    if (other.timeout) result.timeout = other.timeout;
    if (other.exit_fn) result.exit_fn = other.exit_fn;

    return result;
}
Options Options::operator>>(const Options& other) const { return other << *this; }
void Options::operator<<=(const Options& other) { *this = *this << other; }
void Options::operator>>=(const Options& other) { *this = *this >> other; }

/* Parameters */
Parameters::Parameters(const Config& config) {
    dir = AUTO;
    turn = AUTO;
    thru = false;
    relative = false;
    async = false;
    speed = config.speed;
    accel = config.accel;
    lin_PID = config.linear_PID;
    ang_PID = config.angular_PID;
    lead = config.lead;
    lookahead = config.lookahead;
    offset = 0.0;
    lin_exit = config.linear_exit;
    ang_exit = config.angular_exit;
    ang_dz = config.angular_deadzone;
    exit_speed = config.exit_speed;
    settle = config.settle;
    timeout = config.timeout;
    exit_fn = nullptr;
}

Parameters Parameters::apply(const Options& opts) const {
    Parameters result = *this;

    if (opts.dir) result.dir = opts.dir.value();
    if (opts.turn) result.turn = opts.turn.value();
    if (opts.thru) result.thru = opts.thru.value();
    if (opts.relative) result.relative = opts.relative.value();
    if (opts.async) result.async = opts.async.value();
    if (opts.speed) result.speed = opts.speed.value();
    if (opts.accel) result.accel = opts.accel.value();
    if (opts.lin_PID) result.lin_PID = opts.lin_PID.value();
    if (opts.ang_PID) result.ang_PID = opts.ang_PID.value();
    if (opts.lead) result.lead = opts.lead.value();
    if (opts.lookahead) result.lookahead = opts.lookahead.value();
    if (opts.offset) result.offset = opts.offset.value();
    if (opts.lin_exit) result.lin_exit = opts.lin_exit.value();
    if (opts.ang_exit) result.ang_exit = opts.ang_exit.value();
    if (opts.ang_dz) result.ang_dz = opts.ang_dz.value();
    if (opts.exit_speed) result.exit_speed = opts.exit_speed.value();
    if (opts.settle) result.settle = opts.settle.value();
    if (opts.timeout) result.timeout = opts.timeout.value();
    if (opts.exit_fn) result.exit_fn = opts.exit_fn;

    return result;
}

/* Utils */
double to_rad(double deg) { return deg * M_PI / 180; }
double to_deg(double rad) { return rad * 180 / M_PI; }
double limit(double val, double limit) {
    int sign = val > 0 ? 1 : -1;
    return sign * val > limit ? sign * limit : val;
}

} // namespace appa
