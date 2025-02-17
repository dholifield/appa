#include "appa.h"

namespace appa {

/* Chassis */
Chassis::Chassis(std::initializer_list<int8_t> left_motors,
                 std::initializer_list<int8_t> right_motors, Odom& odom, MoveConfig move_config,
                 TurnConfig turn_config, Options default_options)
    : left_motors(left_motors),
      right_motors(right_motors),
      odom(odom),
      move_config(move_config),
      turn_config(turn_config),
      df_options(default_options) {}

Chassis::~Chassis() {
    stop();
    if (chassis_task) {
        chassis_task->remove();
        delete chassis_task;
        chassis_task = nullptr;
    }
}

void Chassis::start() {
    if (chassis_task == nullptr) chassis_task = new pros::Task([this] { task(); }, "chassis_task");
}

void Chassis::task() {
    bool driving = false;

    const int dt = 10; // ms
    uint32_t now = pros::millis();

    // chassis loop
    while (true) {
        // wait for motion
        while (true) {
            chassis_mutex.take();
            Motion current_motion = cmd.motion;
            chassis_mutex.give();
            if (current_motion != IDLE) break;
            pros::delay(dt);
        }

        // set variables
        chassis_mutex.take();
        const Motion motion = cmd.motion;
        const Pose target = cmd.target;
        const Options opts = cmd.options;
        chassis_mutex.give();

        uint32_t start_time = pros::millis();
        const int timeout = opts.timeout.value();

        Direction dir = opts.dir.value();
        const bool auto_dir = dir == AUTO;
        const Direction turn_dir = opts.turn.value();
        const double lead = opts.lead.value();
        const double exit = opts.exit.value();
        const double max_speed = opts.speed.value();
        const double accel_step = opts.accel.value() * dt / 1000;
        const bool thru = opts.thru.value();

        PID lin_PID(opts.lin_PID.value());
        PID ang_PID(opts.ang_PID.value());

        Pose pose;
        Point error, carrot, speeds;
        double lin_speed, ang_speed;

        // control loop
        while (true) {
            // get error
            if (motion == MOVE) { // for move
                error = (pose.dist(target.p()), pose.angle(target.p()));
                if (target.theta != NAN) { // for move to pose
                    carrot = target.p() - Point{error.linear * lead, 0}.rotate(target.theta);
                    error = (pose.dist(carrot), pose.angle(carrot));
                }
            } else if (motion == TURN) { // for turn
                error = (0.0, std::fmod(target.theta - pose.theta, M_PI));
            } else
                break;

            // determine direction
            if (motion == MOVE) { // for move
                if (auto_dir) dir = fabs(error.angular) > M_PI_2 ? REVERSE : FORWARD;
                if (dir == REVERSE) {
                    error.angular += error.angular > 0 ? -M_PI : M_PI;
                    error.linear *= -1;
                }
            } else if (motion == TURN) { // for turn
                if (turn_dir == CW && error.angular < 0)
                    error.angular += 2 * M_PI;
                else if (turn_dir == CCW && error.angular > 0)
                    error.angular -= 2 * M_PI;
            }

            // update PID
            lin_speed = lin_PID.update(error.linear, dt);
            ang_speed = ang_PID.update(error.angular, dt);

            // max speed if thru
            if (thru) {
                if (motion == MOVE)
                    lin_speed = lin_speed > 0 ? max_speed : -max_speed;
                else if (motion == TURN)
                    ang_speed = ang_speed > 0 ? max_speed : -max_speed;
            }

            // limit speeds
            lin_speed = limit(lin_speed, max_speed);
            ang_speed = limit(ang_speed, max_speed);

            // calculate motor speeds
            speeds = (lin_speed - ang_speed, lin_speed + ang_speed);

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
            if (timeout > 0 && pros::millis() - start_time > timeout) break;
            if (error.linear < exit) break;

            // delay task
            pros::c::task_delay_until(&now, dt);
        }
        stop();
    }
}

void Chassis::wait() {
    while (true) {
        chassis_mutex.take();
        Motion m = cmd.motion;
        chassis_mutex.give();
        if (m == IDLE) break;
        pros::delay(10);
    }
}

void Chassis::move(Pose target, Options opts, Options override) {
    chassis_mutex.take();
    Command current_cmd = cmd;
    chassis_mutex.give();

    // stop if motion is currently running
    if (current_cmd.motion != IDLE) {
        stop();
        pros::delay(10);
    }

    // determine options
    Direction dir = override.dir.value_or(opts.dir.value_or(df_options.dir.value_or(AUTO)));
    Direction turn = AUTO;

    double exit = override.exit.value_or(opts.exit.value_or(move_config.exit));
    int timeout = override.timeout.value_or(opts.timeout.value_or(df_options.timeout.value_or(0)));

    double speed = override.speed.value_or(opts.speed.value_or(move_config.speed));
    double accel = override.accel.value_or(opts.accel.value_or(df_options.accel.value_or(0)));
    double lead = override.lead.value_or(opts.lead.value_or(move_config.lead));

    Gains lin_PID = override.lin_PID.value_or(opts.lin_PID.value_or(move_config.lin_PID));
    Gains ang_PID = override.ang_PID.value_or(opts.ang_PID.value_or(move_config.ang_PID));

    bool thru = override.thru.value_or(opts.thru.value_or(df_options.thru.value_or(false)));
    bool async = override.async.value_or(opts.async.value_or(df_options.async.value_or(false)));
    bool relative =
        override.relative.value_or(opts.relative.value_or(df_options.relative.value_or(false)));
    if (target.y == NAN) relative = true;

    Options final_opts(
        dir, turn, exit, timeout, speed, accel, lead, lin_PID, ang_PID, thru, async, relative);

    // update target
    if (target.theta != NAN) target.theta = to_rad(target.theta);

    // if relative move
    if (relative) {
        Pose pose = odom.get();
        if (target.y == NAN) {
            target = Pose{pose.p() + Point{target.x, 0}.rotate(pose.theta), 0.0};
        } else
            target += pose;
    }

    // set command
    chassis_mutex.take();
    cmd = Command(MOVE, target, final_opts);
    chassis_mutex.give();

    // wait if async
    if (async) wait();
}

void Chassis::turn(Point target, Options opts, Options override) {
    chassis_mutex.take();
    Command current_cmd = cmd;
    chassis_mutex.give();

    // stop if motion is currently running
    if (current_cmd.motion != IDLE) {
        stop();
        pros::delay(10);
    }

    // determine options
    Direction dir = override.dir.value_or(opts.dir.value_or(df_options.dir.value_or(AUTO)));
    Direction turn = override.dir.value_or(opts.turn.value_or(df_options.turn.value_or(AUTO)));

    double exit = override.exit.value_or(opts.exit.value_or(move_config.exit));
    int timeout =
        override.timeout.value_or(opts.timeout.value_or(df_options.timeout.value_or(0.0)));

    double speed = override.speed.value_or(opts.speed.value_or(move_config.speed));
    double accel = override.accel.value_or(opts.accel.value_or(df_options.accel.value_or(0.0)));
    double lead = 0.0;

    Gains lin_PID;
    Gains ang_PID = override.ang_PID.value_or(opts.ang_PID.value_or(move_config.ang_PID));

    bool thru = override.thru.value_or(opts.thru.value_or(df_options.thru.value_or(false)));
    bool async = override.async.value_or(opts.async.value_or(df_options.async.value_or(false)));
    bool relative =
        override.relative.value_or(opts.relative.value_or(df_options.relative.value_or(false)));

    Options final_opts(
        dir, turn, exit, timeout, speed, accel, lead, lin_PID, ang_PID, thru, async, relative);

    // update target
    Pose final_target;
    Pose pose = odom.get();
    if (target.y == NAN) {
        final_target.theta = to_rad(target.x);
        if (relative) final_target.theta += pose.theta;
    } else {
        if (relative) target += pose.p();
        final_target.theta = pose.p().angle(target);
    }

    // set command
    chassis_mutex.take();
    cmd = Command(TURN, final_target, final_opts);
    chassis_mutex.give();

    // wait if async
    if (async) wait();
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

void Chassis::stop() {
    tank(0, 0);
    std::lock_guard<pros::Mutex> lock(chassis_mutex);
    cmd.motion = IDLE;
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