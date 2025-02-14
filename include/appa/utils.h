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

// Utils //
enum Direction { FORWARD = 1, AUTO = 0, REVERSE = -1, CCW = 2, CW = -2 };

struct Options {
    std::optional<Direction> dir;

    std::optional<double> exit;
    // std::optional<int> settle;
    std::optional<int> timeout;

    std::optional<double> speed;
    std::optional<double> accel;

    std::optional<Gains> lin_PID;
    std::optional<Gains> ang_PID;

    std::optional<bool> async;
    std::optional<bool> thru;
    std::optional<bool> relative;
};

struct MoveConfig {
    double exit, speed;
    Gains lin_PID, ang_PID;
};

struct TurnConfig {
    double exit, speed;
    Gains ang_PID;
};

struct Point {
    // clang-format off
    union {
        struct { double x, y; };
        struct { double left, right; };
        struct { double linear, angular; };
    };
    // clang-format on

    Point(double x = 0, double y = 0) : x(x), y(y) {}

    Point operator+(const Point& other) const { return Point({x + other.x, y + other.y}); }
    Point operator-(const Point& other) const { return Point({x - other.x, y - other.y}); }
    Point operator*(const double mult) const { return Point({x * mult, y * mult}); }
    void operator+=(const Point& other) {
        x += other.x;
        y += other.y;
    }
    void operator-=(const Point& other) {
        x -= other.x;
        y -= other.y;
    }
    void operator*=(const double mult) {
        x *= mult;
        y *= mult;
    }
    double dist(const Point& other) const {
        Point diff = *this - other;
        return sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    double angle(const Point& other, double offset = 0.0) const {
        Point diff = other - *this;
        return std::fmod(atan2(diff.y, diff.x) - offset, M_PI);
    }
    Point rotate(const double theta) const {
        if (theta == 0) return *this;
        return {x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta)};
    }
};

struct Pose {
    double x, y, theta;

    Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
    Pose(Point p, double theta) : x(p.x), y(p.y), theta(theta) {}

    Pose operator+(const Point& p) const { return Pose({x + p.x, y + p.y}, theta); }
    void operator+=(const Point& p) {
        x += p.x;
        y += p.y;
    }

    Point p() const { return {x, y}; }
    double dist(const Point& other) const {
        Point diff = this->p() - other;
        return sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    double angle(const Point& other) const { return p().angle(other, theta); }
};

double to_rad(double deg);
double to_deg(double rad);
double limit(double val, double limit);

} // namespace appa