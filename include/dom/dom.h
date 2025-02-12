#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <mutex>
#include <optional>

namespace dom {

/* Odom */
class Odom {
  private:
    Pose odom_pose = (0, 0, 0);
    pros::Mutex odom_mutex;
    pros::Task* odom_task = nullptr;

    pros::adi::Encoder x_tracker, y_tracker;
    pros::Imu imu;
    double tpi;

    Point tracker_linear_offset;
    double tracker_angular_offset;

  public:
    Odom(int8_t x_port, int8_t y_port, int8_t imu_port, int tpi, Point tracker_linear_offset,
         double tracker_angular_offset);
    Odom(std::array<int8_t, 2> x_port, std::array<int8_t, 2> y_port, int8_t imu_port, int tpi,
         Point tracker_linear_offset, double tracker_angular_offset);

    void task();
    void start();

    Pose get();
    Pose get_local();
    void set(Pose pose);
    void set(Point point, double theta);
    void set(double x, double y, double theta);
    void set_point(Point point);
    void set_point(double x, double y);
    void set_x(double x);
    void set_y(double y);
    void set_theta(double theta);

    void set_offset(Point linear);

    void debug();
};

/* Chassis */
class Chassis {
  private:
    pros::MotorGroup left_motors, right_motors;
    Odom& odom;
    Options df_move_opts, df_turn_opts;
    Point prev_speeds = (0.0, 0.0);

    pros::Task* chassis_task = nullptr;
    pros::Mutex chassis_mutex;
    std::atomic<bool> is_driving{false};

  public:
    Chassis(std::initializer_list<int8_t> left_motors, std::initializer_list<int8_t> right_motors,
            Odom& odom, Options default_move_options, Options default_turn_ptions);
    ~Chassis();

    void task();
    void wait();

    void move_task(Point target, Options options);
    void move(Point target, Options options = {});
    void move(double target, Options options = {});
    void turn_task(double target, Options options);
    void turn(Point target, Options options = {});
    void turn(double target, Options options = {});

    void tank(double left_speed, double right_speed);
    void tank(Point speeds);
    void tank(pros::Controller& controller);
    void arcade(double linear, double angular);
    void arcade(pros::Controller& controller);
    void stop(bool stop_task = true);

    void set_brake_mode(pros::motor_brake_mode_e_t mode);
};
} // namespace dom