#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <functional>
#include <mutex>
#include <optional>

namespace appa {

/* Odom */
class Odom {
  private:
    Pose odom_pose = {0.0, 0.0, 0.0};
    pros::Mutex odom_mutex;
    pros::Task* odom_task = nullptr;

    Tracker tracker;
    double tpu;

    Point tracker_linear_offset;
    double tracker_angular_offset;

  public:
    std::atomic<bool> debug{false};

    Odom(int8_t x_port, int8_t y_port, Imu imu_port, double tpu, Point tracker_linear_offset,
         double tracker_angular_offset);
    Odom(std::array<int8_t, 2> x_port, std::array<int8_t, 2> y_port, Imu imu_port, double tpu,
         Point tracker_linear_offset, double tracker_angular_offset);
    Odom(int8_t l_port, int8_t r_port, int8_t y_port, double tpu, Point tracker_linear_offset,
         double tracker_angular_offset);
    Odom(std::array<int8_t, 2> r_port, std::array<int8_t, 2> l_port, std::array<int8_t, 2> y_port,
         Imu imu_port, double tpu, Point tracker_linear_offset, double tracker_angular_offset);

    void task();
    void start();

    Pose get();
    Pose get_local();
    void set(Pose pose);
    void set(Point point, double theta = NAN);
    void set(double x, double y, double theta = NAN);
    void set_local(Pose pose);
    void set_x(double x);
    void set_y(double y);
    void set_theta(double theta);

    void set_offset(Point linear);
};

/* Chassis */
class Chassis {
  private:
    pros::MotorGroup left_motors, right_motors;
    Odom& odom;
    Parameters df_params;
    Point prev_speeds = {0.0, 0.0};
    double path_length = 0.0;

    pros::Task* chassis_task = nullptr;
    pros::Mutex chassis_mutex;
    std::atomic<bool> is_running{false};

    enum Motion { MOVE, PATH, TURN };

    void motion_task(Pose target, const Parameters prm, const Motion motion);
    void motion_handler(const std::vector<Pose>& target, const Options& options,
                        const Motion& motion);

  public:
    std::atomic<bool> debug{false};

    Chassis(const std::initializer_list<int8_t>& left_motors,
            const std::initializer_list<int8_t>& right_motors, Odom& odom, const Config& config);
    ~Chassis();

    void task();
    void wait();

    void move(const Pose& target, const Options& options = {}, const Options& override = {});
    void follow(const std::vector<Point>& path, const Options& options = {},
                const Options& override = {});
    void turn(const Point& target, const Options& options = {}, const Options& override = {});

    void tank(double left_speed, double right_speed);
    void tank(const Point& speeds);
    void tank(pros::Controller& controller);
    void arcade(double linear, double angular);
    void arcade(pros::Controller& controller);
    void stop(bool stop_task = true);

    void set_brake_mode(pros::motor_brake_mode_e mode);
};
} // namespace appa