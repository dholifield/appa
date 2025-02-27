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
    Point operator*(double scalar) const;
    void operator+=(const Point& other);
    void operator-=(const Point& other);
    void operator*=(double scalar);
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
    double length;
    int index = 0;

    Path() = default;

    Path(std::initializer_list<Point> points) {
        if (points.size() < 2) throw std::invalid_argument("Path must have at least 2 points.");

        auto it = points.begin();
        Point prev = *it++;
        path.emplace_back(prev, NAN);
        length = 0;

        for (; it != points.end(); ++it) {
            length += prev.dist(*it);
            path.emplace_back(*it, prev.angle(*it));
            prev = *it;
        }
    }

    Pose get() { return path[index]; }
    bool inc() { return ++index >= size(); }
    int size() { return path.size(); }
};

struct Bezier {
    std::vector<Point> anchors;
    std::vector<Point> weights;
    const double length;
    const int sections;
    const double dt;

    double current_t = 0;
    double current_length;

    Bezier(std::initializer_list<Point> anchor_list, std::initializer_list<Point> weight_list,
           int resolution)
        : anchors(anchor_list),
          weights(weight_list),
          sections(anchor_list.size() - 1),
          dt(1.0 / resolution),
          length(total_len(anchor_list, weight_list, resolution)),
          current_length(length) {
        if (anchor_list.size() != weight_list.size())
            throw std::invalid_argument("Anchor and weight lists must be of the same size.");
    }

    static Point bez(const std::array<Point, 4>& b, double t) {
        Point b1 = b[0] * 2 - b[1];
        double u = 1 - t;
        return b[0] * (u * u * u) + b1 * (3 * u * u * t) + b[2] * (3 * u * t * t) +
               b[3] * (t * t * t);
    }

    static double len(const std::array<Point, 4>& b, int resolution) {
        Point prev = bez(b, 0);

        const double res = 1.0 / resolution;
        double len = 0.0;

        for (double i = res; i <= 1; i += res) {
            Point p = bez(b, i);
            len += prev.dist(p);
            prev = p;
        }
        return len;
    }

    static double total_len(const std::vector<Point>& a, const std::vector<Point>& w,
                            int resolution) {
        double length = 0;
        for (int i = 0; i < a.size() - 1; ++i) {
            length += len({a[i], w[i], a[i + 1], w[i + 1]}, resolution);
        }
        return length;
    }

    Point next() {
        int section = ceil(current_t * sections);
        //
    }
};

double to_rad(double deg);
double to_deg(double rad);
double limit(double val, double limit);

} // namespace appa