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

void Chassis::motion_task(Pose target, const Options options, const Motion motion) {
    // set up variables
    const int dt = 10; // ms

    Direction dir = options.dir.value();
    const bool auto_dir = dir == AUTO;
    const Direction turn_dir = options.turn.value();
    const double max_speed = options.speed.value();
    const double accel_step = options.accel.value() * dt / 1000;
    const double lead = options.lead.value();
    const double lookahead = options.lookahead.value();
    const double exit = options.exit.value();
    const double offset = options.offset.value();
    const int settle = options.settle.value();
    const int timeout = options.timeout.value();
    PID lin_PID(options.lin_PID.value());
    PID ang_PID(options.ang_PID.value());
    const bool thru = options.thru.value();
    const bool relative = options.relative.value();
    const std::function<bool()> exit_fn = options.exit_fn;

    Pose pose = odom.get();
    Point error, carrot, speeds;
    double lin_speed, ang_speed;

    // convert to radians
    if (!std::isnan(target.theta)) target.theta = to_rad(target.theta);

    // relative motion
    if (relative)
        target = Pose{pose.p() + target.p().rotate(pose.theta), target.theta + pose.theta};

    // timing
    uint32_t start_time, now;
    start_time = now = pros::millis();
    int settle_time = 0;
    bool running = true;
    bool settling = false;

    // control loop
    while (running && is_running.load()) {
        // find error and direction based on motion type
        pose = odom.get();
        switch (motion) {
        case MOVE:
            // error
            error = {pose.dist(target), pose.angle(target)};
            if (!std::isnan(target.theta)) { // move to pose
                carrot = target.project(-error.linear * lead);
                // carrot = (pose - target).p().rotate(-target.theta); // maybe better boomerang?
                // carrot = target.project(-fabs(carrot.y) - error.linear * lead);
                error.angular = pose.angle(carrot);
            }
            error.linear -= offset;
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
            double dist = carrot.x + lookahead - fabs(carrot.y) / 2; // circle approximation
            if (dist > 0) running = false; // exit when carrot reaches waypoint
            carrot = target.project(dist);
            error = {pose.dist(target) + path_length - offset, pose.angle(carrot)};
            // direction
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            break;
        }
        case TURN:
            // error
            if (std::isnan(target.theta)) error = {0.0, pose.angle(target)}; // turn to point
            else
                error = {0.0,
                         std::remainder(target.theta - pose.theta, 2 * M_PI)}; // turn to heading
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
            chassis_mutex.take();
            if (speeds.left - prev_speeds.left > accel_step)
                speeds.left = prev_speeds.left + accel_step;
            if (speeds.right - prev_speeds.right > accel_step)
                speeds.right = prev_speeds.right + accel_step;
            chassis_mutex.give();
        }

        // set motor speeds
        tank(speeds);

        // check exit conditions
        //   timeout
        if (timeout > 0 && pros::millis() - start_time > timeout) running = false;
        //   exit error
        if (motion == TURN) settling = fabs(error.angular) < to_rad(exit);
        else settling = fabs(error.linear) < exit;
        //   settling
        if (settling) {
            settle_time += dt;
            if (settle_time >= settle) running = false;
        } else settle_time = 0;
        //   custom lambda
        if (exit_fn && exit_fn()) running = false;

        // delay task
        pros::c::task_delay_until(&now, dt);
    }
    if (!thru && !(motion == PATH)) stop(false);
}

void Chassis::motion_handler(const std::vector<Pose>& target, const Options& options,
                             const Motion& motion) {
    // stop task if chassis is already moving
    if (chassis_task) {
        chassis_task->remove();
        delete chassis_task;
    }

    // determine movement lambda
    std::function<void()> movement;
    if (motion == PATH) {
        movement = [this, path = target, options] {
            is_running.store(true);
            for (int i = 0; i < path.size() - 1; ++i) {
                motion_task(path[i], options, PATH);
                path_length -= path[i].dist(path[i + 1]);
            }
            motion_task(path[path.size() - 1], options, MOVE);
            is_running.store(false);
        };
    } else {
        movement = [this, target, options, motion] {
            is_running.store(true);
            motion_task(target[0], options, motion);
            is_running.store(false);
        };
    }

    // start task if async
    if (options.async.value()) {
        chassis_task = new pros::Task(movement, "chassis_task");
    } else {
        movement();
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
    motion_handler({target}, options, MOVE);
}

void Chassis::turn(const Point& target, Options options, const Options& override) {
    // configure target
    Pose target_pose;
    if (std::isnan(target.y)) target_pose.theta = target.x;
    else target_pose = target;

    // merge options
    options = df_turn << options << override;

    // run motion
    motion_handler({target_pose}, options, TURN);
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
    path_length = 0;
    for (int i = 1; i < poses.size(); i++) {
        poses[i].theta = to_deg(poses[i].p().angle(poses[i - 1]) + M_PI);
        path_length += poses[i].dist(poses[i - 1]);
    }

    // run motion
    motion_handler(poses, options, PATH);
}

void Chassis::tank(double left_speed, double right_speed) {
    std::lock_guard<pros::Mutex> lock(chassis_mutex);
    left_motors.move_voltage(left_speed * 120);
    right_motors.move_voltage(right_speed * 120);
    prev_speeds = {left_speed, right_speed};
}
void Chassis::tank(const Point& speeds) { tank(speeds.left, speeds.right); }
void Chassis::tank(pros::Controller& controller) {
    double left_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double right_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 1.27;
    tank(left_speed, right_speed);
}
void Chassis::arcade(double linear, double angular) { tank(linear + angular, linear - angular); }
void Chassis::arcade(pros::Controller& controller) {
    double linear = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double angular = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.27;
    arcade(linear, angular);
}

void Chassis::stop(bool stop_task) {
    is_running.store(false);
    if (stop_task && chassis_task) {
        chassis_task->remove();
        delete chassis_task;
        chassis_task = nullptr;
    }
    tank(0, 0);
}

void Chassis::set_brake_mode(const pros::motor_brake_mode_e_t mode) {
    std::lock_guard<pros::Mutex> lock(chassis_mutex);
    left_motors.set_brake_mode_all(mode);
    right_motors.set_brake_mode_all(mode);
}

// Ziegler-Nichols PID tuning
void Chassis::auto_tune_linear(double speed, bool test) {
    // set target 6 inches forward with bang bang controller: forward and backward at speed
    // store millis and max error each time robot crosses target and repeat like 30 times
    std::vector<uint32_t> times;
    std::vector<double> amplitudes;

    double distance = 6;
    odom.set(0, 0, 0);

    printf("Starting Auto Tune...\n");

    for (int i = 0; i <= 20; ++i) {
        uint32_t time = pros::millis();
        double x = odom.get().x;

        double min = distance;
        arcade(speed, 0);
        while (x < distance) {
            x = odom.get().x;
            if (x < min) min = x;
            pros::delay(10);
        }

        double max = distance;
        arcade(-speed, 0);
        while (x > distance) {
            x = odom.get().x;
            if (x > max) max = x;
            pros::delay(10);
        }

        times.push_back(pros::millis() - time);
        amplitudes.push_back(max - min);
    }

    // calcuate average oscillation period and amplitude (ignore first few until steady state)
    int skip = 5;    // skip to steady state
    double Tu = 0.0; // oscillation period
    double A = 0.0;  // amplitude

    for (int i = skip - 1; i < times.size(); ++i) {
        Tu += times[i] / 1000.0;
        A += amplitudes[i];
    }
    Tu /= times.size() - skip;
    A /= times.size() - skip;

    // Ku = 4A/pi/speed:
    //   PD:  Kp = 0.8*Ku, Kd = 0.1*Ku*Tu
    //   PID: kp = 0.6*Ku, Ki = 1.2*Ku/Tu, Kd = 0.075*Ku*Tu
    double Ku = 4 * A / M_PI / speed;
    Gains P = {0.5 * Ku, 0.0, 0.0};
    Gains PI = {0.45 * Ku, 0.54 * Ku / Tu, 0.0};
    Gains PD = {0.8 * Ku, 0.0, 0.1 * Ku * Tu};
    Gains PID = {0.6 * Ku, 1.2 * Ku / Tu, 0.075 * Ku * Tu};
    Gains psn = {0.7 * Ku, 1.75 * Ku / Tu, 0.105 * Ku * Tu};
    Gains some = {0.3333 * Ku, 0.6667 * Ku / Tu, 0.1111 * Ku * Tu};
    Gains no = {0.20 * Ku, 0.40 * Ku / Tu, 0.0667 * Ku * Tu};

    printf("Auto Tune Results\n");
    printf("  Ku = %.5f, Tu = %.5f\n", Ku, Tu);
    printf("  P:\n    P = %.5f\n", P.p);
    printf("  PD:\n    P = %.5f, D = %.5f\n", PD.p, PD.d);
    printf("  PID:\n    P = %.5f, I = %.5f, D = %.5f\n", PID.p, PID.i, PID.d);
    printf("  Pessen Integration Rule:\n    P = %.5f, I = %.5f, D = %.5f\n", psn.p, psn.i, psn.d);
    printf("  Some Overshoot:\n    P = %.5f, I = %.5f, D = %.5f\n", some.p, some.i, some.d);
    printf("  No Overshoot:\n    P = %.5f, I = %.5f, D = %.5f\n", no.p, no.i, no.d);

    if (test) {
        Options test_options = {.speed = speed, .settle = 250, .timeout = 3000, .lin_PID = no};
        stop();
        pros::delay(250);
        move(12, test_options);
        pros::delay(250);
        move(-12, test_options);
    }
}

} // namespace appa

/**
 * TODO: maybe make path struct. calculate headings in constructor and store path length
 * TODO: maybe add radius to turn so you can do arc movements
 * TODO: add new op control setting for fancy curves and scaling and deadzone
 */

/**
 * TESTING:
 * back to back movements. confirm tasks operate as exepected
 * asynchronous movement
 *      async then wait
 *      async then cancel with new movement
 * test all motions
 *      move distance
 *      move to point
 *      move to pose
 *      turn to heading
 *      turn to point
 *      follow path
 * test all options
 *      dir - move (point + pose) and turn (either)
 *          - negative distance for move
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