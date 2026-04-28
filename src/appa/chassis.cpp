#include "appa.h"

namespace appa {

/* Chassis */
Chassis::Chassis(const std::initializer_list<int8_t>& left_motors,
                 const std::initializer_list<int8_t>& right_motors, Localization& loc,
                 const Config& config)
    : left_motors_(left_motors), right_motors_(right_motors), loc_(loc), df_params_(config) {}

Chassis::~Chassis() { stop(true); }

void Chassis::wait() {
    if (chassis_task_ != nullptr) {
        chassis_task_->join(); // confirm that join behaves properly for a task that is finished
        delete chassis_task_;
        chassis_task_ = nullptr;
    }
}

void Chassis::motion_task(Pose target, const Parameters prm, const Motion motion) {
    // set up variables
    const int dt = 10; // ms

    Direction dir = prm.dir;
    ExitSpeed exit_speed = prm.exit_speed;
    const bool auto_dir = dir == AUTO;
    const double accel_step = prm.accel * dt / 1000;
    PID lin_PID(prm.lin_PID);
    PID ang_PID(prm.ang_PID);

    Pose prev_pose; // prev_pose is NAN so first iteration never exits
    Pose pose = loc_.get();
    Point error, carrot, speeds, abs_speeds;
    double lin_speed, ang_speed;

    // convert to radians
    const bool has_theta = !std::isnan(target.theta);
    if (has_theta) target.theta = to_rad(target.theta);

    // relative motion
    if (prm.relative)
        target = Pose{pose.p() + target.p().rotate(pose.theta), target.theta + pose.theta};

    // timing
    uint32_t start_time, now;
    start_time = now = pros::millis();
    int settle_timer = 0;
    bool exit = false;
    bool settling = false;

    int counter = 0;

    // control loop
    while (!exit && running_.load()) {
        // find error and direction based on motion type
        pose = loc_.get();
        switch (motion) {
        case MOVE:
            // error
            error = {pose.dist(target), pose.angle(target)};
            if (has_theta) { // move to pose
                carrot = target.project(-error.linear * prm.lead);
                error.angular = pose.angle(carrot);
            }
            error.linear -= prm.offset;
            // direction
            if (auto_dir) dir = fabs(error.angular) > M_PI_2 ? REVERSE : FORWARD;
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            // different error scaling for small distances
            if (fabs(error.linear) < prm.ang_dz) {
                // turn to target theta when close
                if (has_theta) error.angular = std::remainder(target.theta - pose.theta, 2 * M_PI);
                // stop turning if no target theta
                else error.angular = 0;
            } else if (fabs(error.linear) < 2 * prm.ang_dz)
                error.angular *= (fabs(error.linear) - prm.ang_dz) / prm.ang_dz;
            // focus on turning first
            error.linear *= cos(error.angular);
            break;
        case PATH: {
            // error
            carrot = (target.p() - pose.p()).rotate(-target.theta);
            double dist = carrot.x + prm.lookahead - fabs(carrot.y) / 2; // circle approximation
            if (dist > 0) exit = true; // exit when carrot reaches waypoint
            carrot = target.project(dist);
            error = {pose.dist(target) + path_length_ - prm.offset, pose.angle(carrot)};
            // direction
            if (dir == REVERSE) {
                error.angular += error.angular > 0 ? -M_PI : M_PI;
                error.linear *= -1;
            }
            break;
        }
        case TURN:
            // error
            if (has_theta) // turn to heading
                error = {0.0, std::remainder(target.theta - pose.theta, 2 * M_PI)};
            else error = {0.0, pose.angle(target)}; // turn to point
            // direction
            if (dir == REVERSE) error.angular += error.angular > 0 ? -M_PI : M_PI;
            if (prm.turn == CW && error.angular < 0) error.angular += 2 * M_PI;
            else if (prm.turn == CCW && error.angular > 0) error.angular -= 2 * M_PI;
            break;
        default:
            running_.store(false);
            continue;
        }

        // calculate PID
        lin_speed = lin_PID.update(error.linear, dt);
        ang_speed = ang_PID.update(error.angular, dt);
        if (prm.thru && motion == MOVE) lin_speed = lin_speed > 0 ? prm.speed : -prm.speed;
        else if (prm.thru && motion == TURN) ang_speed = ang_speed > 0 ? prm.speed : -prm.speed;

        // apply limits
        lin_speed = limit(lin_speed, prm.speed);
        ang_speed = limit(ang_speed, prm.speed);

        // calculate motor speeds
        speeds = {lin_speed - ang_speed, lin_speed + ang_speed};
        abs_speeds = {fabs(speeds.left), fabs(speeds.right)};

        // scale motor speeds
        if (abs_speeds.left > 100) speeds *= 100 / abs_speeds.left;
        if (abs_speeds.right > 100) speeds *= 100 / abs_speeds.right;

        // limit acceleration
        if (accel_step) {
            std::lock_guard<pros::Mutex> lock(chassis_mutex_);
            if (abs_speeds.left - fabs(prev_speeds_.left) > accel_step)
                speeds.left = prev_speeds_.left + (speeds.left > 0 ? accel_step : -accel_step);
            if (abs_speeds.right - fabs(prev_speeds_.right) > accel_step)
                speeds.right = prev_speeds_.right + (speeds.right > 0 ? accel_step : -accel_step);
        }

        // set motor speeds
        move_motors(speeds);

        // exit conditions
        //   timeout
        if (prm.timeout > 0 && pros::millis() - start_time > prm.timeout) exit = true;
        //   exit error
        settling = (fabs(error.linear) < prm.lin_exit); // will always be true for turns
        if (has_theta || motion == TURN) settling *= fabs(error.angular) < to_rad(prm.ang_exit);
        //   settling
        if (settling) {
            settle_timer += dt;
            if (settle_timer > prm.settle) exit = true;
        } else settle_timer = 0;
        //   minimum speed
        if (exit_speed.check(pose - prev_pose, dt)) exit = true;
        prev_pose = pose;
        //   custom lambda
        if (prm.exit_fn && prm.exit_fn()) running_.store(false);

        // delay task
        pros::c::task_delay_until(&now, dt);

        // debug
        if (!(++counter % 10) && debug.load()) {
            printf("lin: %.2f, ang: %.2f, left: %.2f, right: %.2f\n",
                   error.linear,
                   to_deg(error.angular),
                   speeds.left,
                   speeds.right);
            counter = 0;
        }
    }
    if (!running_.load() || (!prm.thru && !(motion == PATH))) move_motors({0, 0});
}

void Chassis::motion_handler(const std::vector<Pose>& target, const Options& options,
                             const Motion& motion) {
    // stop task if chassis is already moving
    stop(true);

    // apply options
    Parameters params = df_params_.apply(options);

    // determine movement function
    std::function<void()> movement;
    if (motion == PATH) {
        movement = [this, path = target, params] {
            running_.store(true);
            for (int i = 0; i < path.size() - 1 && running_.load(); ++i) {
                motion_task(path[i], params, PATH);
                path_length_ -= path[i].dist(path[i + 1]);
            }
            motion_task(path[path.size() - 1], params, MOVE);
            running_.store(false);
        };
    } else {
        movement = [this, target, params, motion] {
            running_.store(true);
            motion_task(target[0], params, motion);
            running_.store(false);
        };
    }

    // start task if async
    if (params.async) {
        chassis_task_ = new pros::Task(movement, "chassis_task_");
    } else {
        movement();
    }
}

void Chassis::move(const Target& target, const Options& options, const Options& overwrite) {
    // merge options
    Options combined_options = target.options << options << overwrite;

    // configure target
    Pose target_pose = target.pose;
    if (std::isnan(target_pose.y)) { // relative straight
        target_pose.y = 0.0;
        combined_options <<= {.dir = AUTO, .relative = true};
    }

    // run motion
    motion_handler({target_pose}, combined_options, MOVE);
}

void Chassis::turn(const Target& target, const Options& options, const Options& overwrite) {
    // merge options
    Options combined_options = target.options << options << overwrite;

    // configure target
    Pose target_pose;
    if (std::isnan(target.pose.y)) target_pose.theta = target.pose.x;
    else target_pose = target.pose;

    // run motion
    motion_handler({target_pose}, combined_options, TURN);
}

void Chassis::follow(const std::vector<Point>& path, const Options& options,
                     const Options& overwrite) {
    // merge options
    Options combined_options = options << overwrite;

    bool relative = combined_options.relative.value();
    combined_options.relative = false;

    // copy points to poses and convert if relative
    Pose pose = loc_.get();
    std::vector<Pose> poses;
    for (Point target : path) {
        if (relative) target = pose.p() + target.rotate(pose.theta);
        poses.push_back({target, NAN});
    }

    // calculate heading for poses and path length
    poses[0].theta = to_deg(pose.p().angle(path[0]));
    path_length_ = 0;
    for (int i = 1; i < poses.size(); i++) {
        poses[i].theta = to_deg(poses[i].p().angle(poses[i - 1]) + M_PI);
        path_length_ += poses[i].dist(poses[i - 1]);
    }

    // run motion
    motion_handler(poses, combined_options, PATH);
}

void Chassis::move_motors(Point speeds) {
    if (!speeds.is_valid()) return;
    std::lock_guard<pros::Mutex> lock(chassis_mutex_);
    left_motors_.move_voltage(speeds.left * 120);
    right_motors_.move_voltage(speeds.right * 120);
    prev_speeds_ = speeds;
}

void Chassis::tank(double left_speed, double right_speed, bool override) {
    if (running_.load()) {         // if active movement
        if (override) stop(false); // stop if override, non-blocking
        else return;               // return if not override
    }
    move_motors({left_speed, right_speed});
}
void Chassis::tank(pros::Controller& controller, bool override) {
    double left_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double right_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 1.27;
    tank(left_speed, right_speed, override);
}

void Chassis::arcade(double linear, double angular, bool override) {
    tank(linear + angular, linear - angular, override);
}
void Chassis::arcade(pros::Controller& controller, bool override) {
    double linear = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;
    double angular = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 1.27;
    arcade(linear, angular, override);
}

void Chassis::stop(bool blocking) {
    running_.store(false);
    if (blocking) wait();
}

void Chassis::set_brake_mode(const pros::motor_brake_mode_e_t mode) {
    std::lock_guard<pros::Mutex> lock(chassis_mutex_);
    left_motors_.set_brake_mode_all(mode);
    right_motors_.set_brake_mode_all(mode);
}

} // namespace appa
