#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <mutex>

namespace appa {

/* Odom */
class Odom : public Localization {
  private:
    Pose odom_pose = {0.0, 0.0, 0.0};
    mutable pros::Mutex odom_mutex;
    pros::Task* odom_task = nullptr;

    Tracker& tracker;
    double angular_offset = 0.0;

    Point tracker_linear_offset;
    double tracker_angular_offset;

  public:
    std::atomic<bool> debug{false};

    Odom(Tracker& tracker, Point tracker_linear_offset, double tracker_angular_offset);
    ~Odom();

    void task();
    void start();

    Pose get() const override;
    Pose get_local() const;
    void set(Pose pose);
    void set(Point point, double theta = NAN);
    void set(double x, double y, double theta = NAN);
    void set_local(Pose pose);
    void set_x(double x);
    void set_y(double y);
    void set_theta(double theta);

    void set_offset(Point linear);
};

// Two Wheels + IMU
struct TwoWheelIMU : public Tracker {
    Encoder x_encoder, y_encoder;
    Imu imu;
    double tpu;

    TwoWheelIMU(Encoder x_encoder, Encoder y_encoder, Imu imu_port, double tpu);

    Pose get() override;
    bool init() override;
};

// Three Wheels
struct ThreeWheel : public Tracker {
    Encoder rx_encoder, lx_encoder, y_encoder;
    double tpu, width;

    ThreeWheel(Encoder lx_encoder, Encoder rx_encoder, Encoder y_encoder, double tpu, double width);

    Pose get() override;
    bool init() override { return true; }
};

} // namespace appa