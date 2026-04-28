#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <mutex>

namespace appa {

/* Chassis */
class Chassis {
  private:
    pros::MotorGroup left_motors_, right_motors_;
    Localization& loc_;
    Parameters df_params_;
    Point prev_speeds_ = {0.0, 0.0};
    double path_length_ = 0.0;

    pros::Task* chassis_task_ = nullptr;
    pros::Mutex chassis_mutex_;
    std::atomic<bool> running_{false};

    enum Motion { MOVE, PATH, TURN };

    void move_motors(Point speeds);
    void motion_task(Pose target, const Parameters prm, const Motion motion);
    void motion_handler(const std::vector<Pose>& target, const Options& options,
                        const Motion& motion);

  public:
    std::atomic<bool> debug{false};

    Chassis(const std::initializer_list<int8_t>& left_motors,
            const std::initializer_list<int8_t>& right_motors, Localization& loc,
            const Config& config);
    ~Chassis();

    void wait();
    void stop(bool blocking = true);

    void move(const Target& target, const Options& options = {}, const Options& overwrite = {});
    void follow(const std::vector<Point>& path, const Options& options = {},
                const Options& overwrite = {});
    void turn(const Target& target, const Options& options = {}, const Options& overwrite = {});

    void tank(double left_speed, double right_speed, bool override = false);
    void tank(pros::Controller& controller, bool override = false);
    void arcade(double linear, double angular, bool override = false);
    void arcade(pros::Controller& controller, bool override = false);

    void set_brake_mode(pros::motor_brake_mode_e mode);
};
} // namespace appa