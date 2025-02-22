#pragma once

#include "appa.h"

namespace appa {

/* PID */
struct Gains {
    double p, i, d;
    Gains(double p = 0.0, double i = 0.0, double d = 0.0) : p(p), i(i), d(d) {}
};

class PID {
    Gains k;
    double prev_error, total_error;

  public:
    PID(Gains k);
    PID(double kp, double ki, double kd);
    void reset(double error = 0.0);
    double update(double error, int dt);
};

/* Imu */
struct Imu {
    std::vector<pros::Imu> imus{};

    Imu(std::initializer_list<uint8_t> ports) {
        for (auto port : ports) {
            imus.emplace_back(port);
        }
    }
    Imu(uint8_t port) { imus.emplace_back(port); }

    bool calibrate() {
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

    double get() {
        double rotation = 0;
        for (auto& imu : imus) {
            rotation -= imu.get_rotation();
        }
        return rotation / imus.size();
    }

    void set(double angle) {
        for (auto& imu : imus) {
            imu.set_rotation(-angle);
        }
    }
};

/* Utils */
enum Direction { AUTO, FORWARD, REVERSE, CCW, CW };

#define AUTO appa::AUTO
#define FORWARDS appa::FORWARD
#define REVERSE appa::REVERSE
#define CCW appa::CCW
#define CW appa::CW

struct Options {
    std::optional<Direction> dir, turn;
    std::optional<double> speed, accel, lead, lookahead, exit;
    std::optional<int> timeout;
    std::optional<Gains> lin_PID, ang_PID;
    std::optional<bool> thru, relative, async;
    std::function<bool()> exit_fn = nullptr;

    static Options defaults() {
        return Options(
            AUTO, AUTO, 0.0, 0.0, 0.0, 0.0, 0.0, 0, Gains(), Gains(), false, false, false);
    }

    Options operator<<(const Options& other) const {
        Options result = *this;

        if (other.dir) result.dir = other.dir;
        if (other.turn) result.turn = other.turn;
        if (other.speed) result.speed = other.speed;
        if (other.accel) result.accel = other.accel;
        if (other.lead) result.lead = other.lead;
        if (other.lookahead) result.lookahead = other.lookahead;
        if (other.exit) result.exit = other.exit;
        if (other.timeout) result.timeout = other.timeout;
        if (other.lin_PID) result.lin_PID = other.lin_PID;
        if (other.ang_PID) result.ang_PID = other.ang_PID;
        if (other.thru) result.thru = other.thru;
        if (other.relative) result.relative = other.relative;
        if (other.async) result.async = other.async;
        if (other.exit_fn) result.exit_fn = other.exit_fn;

        return result;
    }
    Options operator>>(const Options& other) const { return other << *this; }
    void operator<<=(const Options& other) { *this = *this << other; }
    void operator>>=(const Options& other) { *this = *this >> other; }
};

struct MoveConfig {
    double exit, speed, lead, lookahead;
    Gains lin_PID, ang_PID;

    Options options() const {
        return Options{.speed = speed,
                       .lead = lead,
                       .lookahead = lookahead,
                       .exit = exit,
                       .lin_PID = lin_PID,
                       .ang_PID = ang_PID};
    }
};

struct TurnConfig {
    double exit, speed;
    Gains ang_PID;
    Options options() const { return Options{.speed = speed, .exit = exit, .ang_PID = ang_PID}; }
};

struct Point {
    // clang-format off
    union {
        struct { double x, y; };
        struct { double left, right; };
        struct { double linear, angular; };
    };
    // clang-format on

    Point(double x = NAN, double y = NAN) : x(x), y(y) {}

    Point operator+(const Point& other) const { return Point({x + other.x, y + other.y}); }
    Point operator-(const Point& other) const { return Point({x - other.x, y - other.y}); }
    Point operator*(double mult) const { return Point({x * mult, y * mult}); }
    void operator+=(const Point& other) {
        x += other.x;
        y += other.y;
    }
    void operator-=(const Point& other) {
        x -= other.x;
        y -= other.y;
    }
    void operator*=(double mult) {
        x *= mult;
        y *= mult;
    }
    void operator=(const Point& p) {
        x = p.x;
        y = p.y;
    }
    double dist(const Point& other) const {
        Point diff = *this - other;
        return sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    double angle(const Point& other, double offset = 0.0) const {
        Point diff = other - *this;
        return std::fmod(atan2(diff.y, diff.x) - offset, M_PI);
    }
    Point rotate(double theta) const {
        if (theta == 0) return *this;
        double sine = sin(theta);
        double cosine = cos(theta);
        return {x * cosine - y * sine, x * sine + y * cosine};
    }
};

struct Pose {
    double x, y, theta;

    Pose(double x = NAN, double y = NAN, double theta = NAN) : x(x), y(y), theta(theta) {}
    Pose(const Point& p, double theta) : x(p.x), y(p.y), theta(theta) {}

    Pose operator+(const Point& p) const { return Pose(x + p.x, y + p.y, theta); }
    Pose operator+(const Pose& p) const { return Pose(x + p.x, y + p.y, theta + p.theta); }
    Pose operator-(const Point& p) const { return Pose(x - p.x, y - p.y, theta); }
    Pose operator-(const Pose& p) const { return Pose(x - p.x, y - p.y, theta - p.theta); }
    void operator+=(const Point& p) {
        x += p.x;
        y += p.y;
    }
    void operator+=(const Pose& p) {
        x += p.x;
        y += p.y;
        theta += theta;
    }
    void operator-=(const Point& p) {
        x -= p.x;
        y -= p.y;
    }
    void operator-=(const Pose& p) {
        x -= p.x;
        y -= p.y;
        theta -= theta;
    }
    void operator=(const Point& p) {
        x = p.x;
        y = p.y;
    }
    void operator=(const Pose& p) {
        x = p.x;
        y = p.y;
        theta = p.theta;
    }

    operator Point() const { return Point{x, y}; }
    Point p() const { return {x, y}; }
    double dist(const Point& other) const { return p().dist(other); }
    double angle(const Point& other) const { return p().angle(other, theta); }
    Point project(double d) const { return p() + Point{d * cos(theta), d * sin(theta)}; }
};

double to_rad(double deg);
double to_deg(double rad);
double limit(double val, double limit);

} // namespace appa