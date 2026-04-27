#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <mutex>

namespace appa {

/* Odom */
class Odom : public Localization {
  private:
    Pose odom_pose_ = {0.0, 0.0, 0.0};
    mutable pros::Mutex odom_mutex_;
    std::atomic<bool> running_{false};
    pros::Task* odom_task_ = nullptr;

    Tracker& tracker_;
    double angular_offset_ = 0.0;

    Point tracker_linear_offset_;
    double tracker_angular_offset_;

    void task();

  public:
    std::atomic<bool> debug{false};

    Odom(Tracker& tracker, Point tracker_linear_offset, double tracker_angular_offset);
    ~Odom();

    void start(Pose pose = {0.0, 0.0, 0.0});
    void stop();

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
    EncoderWheel x_encoder, y_encoder;
    Imu imu;

    TwoWheelIMU(EncoderWheel x_encoder, EncoderWheel y_encoder, Imu imu_port);

    Pose get() override;
    void calibrate(bool blocking = true);
};

// Three Wheels
struct ThreeWheel : public Tracker {
    EncoderWheel lx_encoder, rx_encoder, y_encoder;
    double width;

    ThreeWheel(EncoderWheel lx_encoder, EncoderWheel rx_encoder, EncoderWheel y_encoder,
               double width);

    Pose get() override;
};

} // namespace appa