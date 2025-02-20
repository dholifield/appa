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
    void set(Point point, double theta = NAN);
    void set(double x, double y, double theta = NAN);
    void set_local(Pose pose);
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
    Options df_move, df_turn;
    Point prev_speeds = {0.0, 0.0};

    pros::Task* chassis_task = nullptr;
    pros::Mutex chassis_mutex;

    enum Motion { MOVE, PATH, TURN };

    void motion_task(Pose target, const Options options, const Motion motion);
    void motion_handler(const Pose& target, const Options& options, const Motion& motion);
    void path_handler(const std::vector<Pose>& path, const Options& options);

  public:
    Chassis(const std::initializer_list<int8_t>& left_motors,
            const std::initializer_list<int8_t>& right_motors, Odom& odom,
            const MoveConfig& move_config, const TurnConfig& turn_config,
            const Options& default_options = {});
    ~Chassis();

    void task();
    void wait();

    void move(Pose target, Options options = {}, const Options& override = {});
    void follow(const std::vector<Point>& path, Options options = {}, const Options& override = {});
    void turn(const Point& target, Options options = {}, const Options& override = {});

    void tank(double left_speed, double right_speed);
    void tank(const Point& speeds);
    void tank(pros::Controller& controller);
    void arcade(double linear, double angular);
    void arcade(pros::Controller& controller);
    void stop(bool stop_task = true);

    void set_brake_mode(pros::motor_brake_mode_e mode);
};
} // namespace appa