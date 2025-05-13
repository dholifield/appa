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
    std::atomic<bool> running{false};
    pros::Task* odom_task = nullptr;

    Tracker& tracker;
    double angular_offset = 0.0;

    Point tracker_linear_offset;
    double tracker_angular_offset;

    void task();

  public:
    std::atomic<bool> debug{false};

    Odom(Tracker& tracker, Point tracker_linear_offset, double tracker_angular_offset);
    ~Odom();

    void start();
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
    void calibrate();
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