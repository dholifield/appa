#pragma once

#include "appa.h"

namespace appa {

/* Point */
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

/* Pose */
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

    Imu(std::initializer_list<uint8_t> ports = {});
    Imu(uint8_t port);

    bool calibrate();
    double get();
    void set(double angle);
};

/* Encoder */
struct Encoder {
    pros::adi::Encoder enc;

    Encoder(int8_t port);
    Encoder(uint8_t expander, int8_t port);

    double get_value();
};

/* Trackers */
struct Tracker {
    enum class Type { TWO, THREE };
    Imu imu;
    double tpu, width;
    std::vector<Encoder> trackers{};

    // Two Tracker
    Tracker(Encoder x_encoder, Encoder y_encoder, Imu imu_port, double tpu);
    // Three Tracker
    Tracker(Encoder lx_encoder, Encoder rx_encoder, Encoder y_encoder, double tpu, double width);

    Pose get();

  private:
    Type type;
};

/* Options and Parameters */
enum Direction { AUTO, FORWARD, REVERSE, CCW, CW };

#define AUTO appa::AUTO
#define FORWARDS appa::FORWARD
#define REVERSE appa::REVERSE
#define CCW appa::CCW
#define CW appa::CW

struct ExitSpeed {
    double linear, angular;
    int settle;

    ExitSpeed(double linear = 0.0, double angular = 0.0, int settle = 0)
        : linear(linear), angular(angular), settle(settle) {}
    bool check(Pose dp, int dt);

  private:
    int timer = 0;
};

struct Options {
    // direction
    std::optional<Direction> dir;
    std::optional<Direction> turn;
    // flags
    std::optional<bool> thru;
    std::optional<bool> relative;
    std::optional<bool> async;
    // speed
    std::optional<double> speed;
    std::optional<double> accel;
    std::optional<Gains> lin_PID;
    std::optional<Gains> ang_PID;
    // carrot
    std::optional<double> lead;
    std::optional<double> lookahead;
    std::optional<double> offset;
    // exit conditions
    std::optional<double> lin_exit;
    std::optional<double> ang_exit;
    std::optional<double> ang_dz;
    std::optional<ExitSpeed> exit_speed;
    std::optional<int> settle;
    std::optional<int> timeout;
    std::function<bool()> exit_fn = nullptr;

    Options operator<<(const Options& other) const;
    Options operator>>(const Options& other) const;
    void operator<<=(const Options& other);
    void operator>>=(const Options& other);
};

struct Config {
    double speed, accel;
    Gains linear_PID, angular_PID;
    double lead, lookahead;
    double linear_exit, angular_exit;
    double angular_deadzone;
    ExitSpeed exit_speed;
    int settle, timeout;
};

// Parameters                      // X = used, L = last point
struct Parameters {                // point  pose  path  turn
    Direction dir;                 //   X     X     X     X
    Direction turn;                //                     X
    bool thru;                     //   X     X     X     X
    bool relative;                 //   X     X     X     X
    bool async;                    //   X     X     X     X
    double speed;                  //   X     X     X     X
    double accel;                  //   X     X     X     X
    Gains lin_PID;                 //   X     X     X
    Gains ang_PID;                 //   X     X     X     X
    double lead;                   //         X     L
    double lookahead;              //               X
    double offset;                 //   X     X     L
    double lin_exit;               //   X     X     L
    double ang_exit;               //         X     L     X
    double ang_dz;                 //         X     L     X
    ExitSpeed exit_speed;          //   X     X     X     X
    int settle;                    //   X     X     L     X
    int timeout;                   //   X     X     X     X
    std::function<bool()> exit_fn; //   X     X     X     X

    Parameters(const Config& config);
    Parameters apply(const Options& options) const;
};

/* Utils */
double to_rad(double deg);
double to_deg(double rad);
double limit(double val, double limit);

} // namespace appa