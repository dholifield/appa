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

    Imu(std::initializer_list<uint8_t> ports);
    Imu(uint8_t port);

    bool calibrate();
    double get();
    void set(double angle);
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

    static Options defaults();

    Options operator<<(const Options& other) const;
    Options operator>>(const Options& other) const;
    void operator<<=(const Options& other);
    void operator>>=(const Options& other);
};

struct MoveConfig {
    double exit, speed, lead, lookahead;
    Gains lin_PID, ang_PID;

    Options options() const;
};

struct TurnConfig {
    double exit, speed;
    Gains ang_PID;
    Options options() const;
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

    Point operator+(const Point& other) const;
    Point operator-(const Point& other) const;
    Point operator*(double mult) const;
    void operator+=(const Point& other);
    void operator-=(const Point& other);
    void operator*=(double mult);
    void operator=(const Point& p);

    double dist(const Point& other) const;
    double angle(const Point& other, double offset = 0.0) const;
    Point rotate(double theta) const;
};

struct Pose {
    double x, y, theta;

    Pose(double x = NAN, double y = NAN, double theta = NAN) : x(x), y(y), theta(theta) {}
    Pose(const Point& p, double theta) : x(p.x), y(p.y), theta(theta) {}

    operator Point() const;
    Point p() const;
    Pose operator+(const Point& p) const;
    Pose operator-(const Point& p) const;
    void operator+=(const Point& p);
    void operator-=(const Point& p);
    void operator=(const Point& p);

    Pose operator+(const Pose& p) const;
    Pose operator-(const Pose& p) const;
    void operator+=(const Pose& p);
    void operator-=(const Pose& p);
    void operator=(const Pose& p);

    double dist(const Point& other) const;
    double angle(const Point& other) const;
    Point project(double d) const;
};

struct Path {
    std::vector<Pose> path;

    Path() = default;

    Path(std::initializer_list<Point> points) {
        if (points.size() < 2) return;

        auto it = points.begin();
        Point prev = *it++;
        path.emplace_back(prev, NAN);

        for (; it != points.end(); ++it) {
            path.emplace_back(*it, prev.angle(*it));
            prev = *it;
        }
    }
};

struct Bezier : public Path {
    Bezier(std::initializer_list<Point> anchors, std::initializer_list<Point> weights,
           double resolution) {
        // calculate resulution number of points on the bezier curve(s) defined by anchors and
        // weights
    }
};

double to_rad(double deg);
double to_deg(double rad);
double limit(double val, double limit);

} // namespace appa