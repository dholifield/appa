#include "appa.h"

namespace appa {

/* Chassis */
Chassis::Chassis(const std::initializer_list<int8_t>& left_motors,
                 const std::initializer_list<int8_t>& right_motors, Odom& odom,
                 const MoveConfig& move_config, const TurnConfig& turn_config,
                 const Options& default_options)
    : left_motors(left_motors), right_motors(right_motors), odom(odom) {

    df_move = Options::defaults() << default_options << move_config.options();
    df_turn = Options::defaults() << default_options << turn_config.options();
}

Chassis::~Chassis() { stop(true); }

void Chassis::wait() {
    if (chassis_task) {
        chassis_task->join();
        chassis_task->remove();
        delete chassis_task;
    }
}

void Chassis::motion_task(Pose target, const Options opts, const Motion motion) {
    // set up variables
    const int dt = 10; // ms

    Direction dir = opts.dir.value();
    const bool auto_dir = dir == AUTO;
    const Direction turn_dir = opts.turn.value();
    const double max_speed = opts.speed.value();
    const double accel_step = opts.accel.value() * dt / 1000;
    const double lead = opts.lead.value();
    const double lookahead = opts.lookahead.value();
    const double exit = opts.exit.value();
    const int timeout = opts.timeout.value();
    PID lin_PID(opts.lin_PID.value());
    PID ang_PID(opts.ang_PID.value());
    const bool thru = opts.thru.value();
    const bool relative = opts.relative.value();
    const std::function<bool()> exit_fn = opts.exit_fn;

    Pose pose = odom.get();
    Point error, carrot, speeds;
    double lin_speed, ang_speed;

    // convert to radians
    if (!std::isnan(target.theta)) target.theta = to_rad(target.theta);

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
        pose = odom.get();
        switch (motion) {
        case MOVE:
            // error
            error = {pose.dist(target.p()), pose.angle(target.p())};
            if (!std::isnan(target.theta)) { // move to pose
                carrot = target.project(-error.linear * lead);
                // maybe better boomerang?
                // carrot = (pose - target).p().rotate(-target.theta);
                // carrot = target.project(-fabs(carrot.y) - error.linear * lead);
                error = {pose.dist(carrot), pose.angle(carrot)};
            }
            // direction
            if (auto_dir) dir = fabs(error.angular) > M_PI_2 ? REVERSE : FORWARD;
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            break;
        case PATH: {
            // error
            carrot = (target.p() - pose.p()).rotate(-target.theta);
            // double cy = carrot.y * carrot.y; // magic equation!
            // double dist = carrot.x + lookahead * (1 - cy / (lookahead * lookahead + 0.5 * cy));
            double dist = carrot.x + lookahead - fabs(carrot.y) / 2;
            if (dist > 0) {
                running = false;
                continue;
            }
            carrot = target.project(dist);
            error = {pose.dist(carrot), pose.angle(carrot)};
            // direction
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            break;
        }
        case TURN:
            // error
            if (std::isnan(target.theta)) error = {0.0, pose.angle(target.p())}; // turn to point
            else error = {0.0, std::fmod(target.theta - pose.theta, M_PI)};      // turn to heading
            // direction
            if (dir == REVERSE) error.angular += error.angular > 0 ? -M_PI : M_PI;
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
        if (speeds.left > 100) {
            speeds.right *= 100 / speeds.left;
            speeds.left = 100;
        }
        if (speeds.right > 100) {
            speeds.left *= 100 / speeds.right;
            speeds.right = 100;
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
        if (fabs(error.linear) < exit) running = false;
        if (exit_fn && exit_fn()) running = false;

        // delay task
        pros::c::task_delay_until(&now, dt);
    }
    if (!thru) stop(false);
}

void Chassis::motion_handler(const Pose& target, const Options& opts, const Motion& motion) {
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

void Chassis::path_handler(const std::vector<Pose>& path, const Options& options) {
    // stop task if robot is already moving
    if (chassis_task) {
        chassis_task->remove();
        delete chassis_task;
    }
    // follow path
    auto follow_path = [this, path, options] {
        // code
        for (int i = 0; i < path.size() - 1; ++i) {
            motion_task(path[i], options << Options{.thru = true}, PATH);
        }
        motion_task(path[path.size() - 1], options, MOVE);
    };

    if (options.async.value()) {
        chassis_task = new pros::Task(follow_path, "chassis_task");
    } else {
        follow_path();
    }
}

void Chassis::move(Pose target, Options options, const Options& override) {
    // configure target
    if (std::isnan(target.y)) { // relative straight
        target.y = 0.0;
        options.relative = true;
        options.dir = AUTO;
    }

    // merge options
    options = df_move << options << override;

    // run motion
    motion_handler(target, options, MOVE);
}

void Chassis::follow(const std::vector<Point>& path, Options options, const Options& override) {
    // merge options
    options = df_move << options << override;
    bool relative = options.relative.value();
    options.relative = false;

    // copy points to poses and convert if relative
    Pose pose = odom.get();
    std::vector<Pose> poses;
    for (int i = 0; i < path.size(); i++) {
        Point target = path[i];
        if (relative) target = pose.p() + target.rotate(pose.theta);
        poses.push_back({target, NAN});
    }

    // calculate heading for poses
    poses[0].theta = to_deg(pose.p().angle(path[0]));
    for (int i = 1; i < poses.size(); i++) {
        poses[i].theta = to_deg(poses[i].p().angle(poses[i - 1].p()) + M_PI);
    }

    // run motion
    path_handler(poses, options);
}

void Chassis::turn(const Point& target, Options options, const Options& override) {
    // configure target
    Pose target_pose;
    if (std::isnan(target.y)) target_pose.theta = target.x;
    else target_pose = target;

    // merge options
    options = df_turn << options << override;

    // run motion
    motion_handler(target_pose, options, TURN);
}

void Chassis::tank(double left_speed, double right_speed) {
    left_motors.move_voltage(left_speed * 120);
    right_motors.move_voltage(right_speed * 120);
    std::lock_guard<pros::Mutex> lock(chassis_mutex);
    prev_speeds = (left_speed, right_speed);
}

void Chassis::tank(const Point& speeds) { tank(speeds.left, speeds.right); }

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
 * TODO:
 */

/**
 * TESTING:
 * back to back movements. confirm tasks operate as exepected
 * asynchronous movement. async then wait. async then cancel with new movement
 * test all motions
 *      move distance
 *      move to point
 *      move to pose
 *      turn to heading
 *      turn to point
 *      follow path
 * test all options
 *      dir - move (point + pose) and turn (either)
 *      turn - turn (CW and CCW)
 *      exit - move (any) and turn (any)
 *      timeout - any
 *      speed - any
 *      accel - any
 *      lead - move (pose)
 *      lookahead - follow
 *      lin_PID - move (any)
 *      ang_PID - move (any) and turn (any)
 *      thru - move (point + pose) and turn (any)
 *      relative - move (point + pose) and turn (heading + point)
 */