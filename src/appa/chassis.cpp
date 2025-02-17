#include "appa.h"

namespace appa {

/* Chassis */
Chassis::Chassis(std::initializer_list<int8_t> left_motors,
                 std::initializer_list<int8_t> right_motors, Odom& odom, MoveConfig move_config,
                 TurnConfig turn_config, Options default_options)
    : left_motors(left_motors), right_motors(right_motors), odom(odom) {

    df_move = Options::defaults() << default_options
                                  << Options{.exit = move_config.exit,
                                             .speed = move_config.speed,
                                             .lin_PID = move_config.lin_PID,
                                             .ang_PID = move_config.ang_PID};
    df_turn = Options::defaults() << default_options
                                  << Options{.exit = to_rad(turn_config.exit),
                                             .speed = turn_config.speed,
                                             .ang_PID = turn_config.ang_PID};
}

Chassis::~Chassis() { stop(true); }

void Chassis::wait() {
    if (chassis_task) { chassis_task->join(); }
}

void Chassis::motion_task(Pose target, Options opts, Motion motion) {
    // set up variables
    const int dt = 10; // ms

    Direction dir = opts.dir.value();
    const bool auto_dir = dir == AUTO;
    const Direction turn_dir = opts.turn.value();
    const double exit = opts.exit.value();
    const int timeout = opts.timeout.value();
    const double max_speed = opts.speed.value();
    const double accel_step = opts.accel.value() * dt / 1000;
    const double lead = opts.lead.value();
    PID lin_PID(opts.lin_PID.value());
    PID ang_PID(opts.ang_PID.value());
    const bool thru = opts.thru.value();
    const bool relative = opts.relative.value();

    Pose pose = odom.get();
    Point error, carrot, speeds;
    double lin_speed, ang_speed;

    // relative motion
    if (relative)
        target = Pose{pose.p() + target.p().rotate(pose.theta), target.theta + pose.theta};

    // timing
    uint32_t start_time = pros::millis();
    uint32_t now = pros::millis();
    bool running = true;

    // control loop
    while (running) {
        // find error and direction based on motion type
        switch (motion) {
        case MOVE:
            // error
            error = {pose.dist(target.p()), pose.angle(target.p())};
            if (target.theta != NAN) { // move to pose
                carrot = target.p() - Point{error.linear * lead, 0}.rotate(target.theta);
                error = {pose.dist(carrot), pose.angle(carrot)};
            }
            // direction
            if (auto_dir) dir = fabs(error.angular) > M_PI_2 ? REVERSE : FORWARD;
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            break;
        case TURN:
            // error
            if (target.theta == NAN) error = {0.0, pose.angle(target.p())}; // turn to point
            else error = {0.0, std::fmod(target.theta - pose.theta, M_PI)}; // turn to heading
            // direction
            if (turn_dir == CW && error.angular < 0) error.angular += 2 * M_PI;
            else if (turn_dir == CCW && error.angular > 0) error.angular -= 2 * M_PI;
            break;
        default:
            running = false;
            continue;
        }

        // calculate PID
        lin_speed = lin_PID.update(error.linear, dt);
        ang_speed = ang_PID.update(error.angular, dt);
        if (thru && motion == MOVE) lin_speed = lin_speed > 0 ? max_speed : -max_speed;
        else if (thru && motion == TURN) ang_speed = ang_speed > 0 ? max_speed : -max_speed;

        // apply limits
        lin_speed = limit(lin_speed, max_speed);
        ang_speed = limit(ang_speed, max_speed);

        // calculate motor speeds
        speeds = {lin_speed - ang_speed, lin_speed + ang_speed};

        // scale motor speeds
        if (speeds.left > max_speed) {
            speeds.left = max_speed;
            speeds.right *= max_speed / speeds.left;
        }
        if (speeds.right > max_speed) {
            speeds.right = max_speed;
            speeds.left *= max_speed / speeds.right;
        }

        // limit acceleration
        if (accel_step) {
            if (speeds.left - prev_speeds.left > accel_step)
                speeds.left = prev_speeds.left + accel_step;
            if (speeds.right - prev_speeds.right > accel_step)
                speeds.right = prev_speeds.right + accel_step;
        }

        // set motor speeds
        tank(speeds);

        // check exit conditions
        if (timeout > 0 && pros::millis() - start_time > timeout) running = false;
        if (error.linear < exit) running = false;

        // delay task
        pros::c::task_delay_until(&now, dt);
    }
    if (!thru) stop(false);
}

void Chassis::motion_run(Pose target, Options opts, Motion motion) {
    // stop task if robot is already moving
    if (chassis_task) {
        chassis_task->remove();
        delete chassis_task;
    }

    // start task if async
    if (opts.async.value()) {
        chassis_task = new pros::Task(
            [this, target, opts, motion] { motion_task(target, opts, motion); }, "chassis_task");
    } else {
        motion_task(target, opts, motion);
    }
}

void Chassis::move(Pose target, Options opts, Options override) {
    // configure target
    if (target.y == NAN) {
        target.y = 0.0;
        opts.relative = true;
    }
    if (target.theta != NAN) target.theta = to_rad(target.theta);

    // merge options
    opts = df_move << opts << override;

    // run motiong
    motion_task(target, opts, MOVE);
}

void Chassis::turn(Point target, Options opts, Options override) {
    // configure target
    Pose target_pose;
    if (target.y == NAN) target_pose.theta = to_rad(target.x);
    else target_pose = target;

    // merge options
    opts = df_move << opts << override;

    // run motiong
    motion_task(target_pose, opts, TURN);
}

void Chassis::tank(double left_speed, double right_speed) {
    left_motors.move_voltage(left_speed * 120);
    right_motors.move_voltage(right_speed * 120);
    std::lock_guard<pros::Mutex> lock(chassis_mutex);
    prev_speeds = (left_speed, right_speed);
}

void Chassis::tank(Point speeds) { tank(speeds.left, speeds.right); }

void Chassis::tank(pros::Controller& controller) {
    double left_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double right_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 1.27;
    tank(left_speed, right_speed);
}

void Chassis::arcade(double linear, double angular) {
    double left_speed = linear + angular;
    double right_speed = linear - angular;
    tank(left_speed, right_speed);
}

void Chassis::arcade(pros::Controller& controller) {
    double linear = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double angular = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.27;
    arcade(linear, angular);
}

void Chassis::stop(bool stop_task) {
    if (stop_task && chassis_task) {
        chassis_task->remove();
        delete chassis_task;
        chassis_task = nullptr;
    }
    tank(0, 0);
}

void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

} // namespace appa

/**
 * TODO: add settle time
 * TODO: implement settle exit for stuck robot
 * TODO: make all movements run in 1 task
 */