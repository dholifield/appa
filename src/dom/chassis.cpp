#include "dom.h"

namespace dom {

/* Chassis */
Chassis::Chassis(std::initializer_list<int8_t> left_motors,
                 std::initializer_list<int8_t> right_motors,
                 Odom& odom,
                 Options default_move_options,
                 Options default_turn_options)
    : left_motors(left_motors),
      right_motors(right_motors),
      odom(odom),
      df_move_opts(default_move_options),
      df_turn_opts(default_turn_options) {}
      
Chassis::~Chassis() {
    stop();
}

void Chassis::init() {
    set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Chassis::wait() {
    if (chassis_task) {
        chassis_task->join();
    }
}

void Chassis::move_task(Point target, Options opts) {
    // set up variables
    Direction dir = opts.dir.value_or(df_move_opts.dir.value_or(AUTO));
    bool auto_dir = dir == AUTO;

    double exit = opts.exit.value_or(df_move_opts.exit.value_or(1.0));
    // int settle = opts.settle.value_or(df_move_opts.settle.value_or(250));
    int timeout = opts.timeout.value_or(df_move_opts.timeout.value_or(10000));

    double max_speed = opts.speed.value_or(df_move_opts.speed.value_or(100));
    double accel = opts.accel.value_or(df_move_opts.accel.value_or(50));

    PID lin_PID(opts.lin_PID.value_or(df_move_opts.lin_PID.value_or(Gains{10, 0, 0})));
    PID ang_PID(opts.ang_PID.value_or(df_move_opts.ang_PID.value_or(Gains{50, 0, 0})));

    bool thru = opts.thru.value_or(df_move_opts.thru.value_or(false));
    bool relative = opts.relative.value_or(df_move_opts.relative.value_or(false));

    Pose pose = odom.get();
    Point error = (pose.dist(target), pose.angle(target));

    lin_PID.reset(error.linear);
    ang_PID.reset(error.angular);

    double lin_speed, ang_speed;
    Point speeds;
    double prev_left = left_motors.get_voltage() / 120;
    double prev_right = right_motors.get_voltage() / 120;
    
    // if relative motion
    if (relative) target = pose.p() + target.rotate(pose.theta);

    // timing
    int dt = 10; // ms
    int start_time = pros::millis();
    uint32_t now = pros::millis();

    double accel_step = accel * dt / 1000;

    // control loop
    while(true) {
        // calculate error
        pose = odom.get();

        error = (pose.dist(target), 
                 pose.angle(target));

        // determine direction
        if (auto_dir) {
            if (fabs(error.angular) > M_PI_2) dir = REVERSE;
            else dir = FORWARD;
        }
        if (dir == REVERSE) error.angular = M_PI - error.angular;

        // calculate PID
        lin_speed = thru ? max_speed : lin_PID.update(error.linear, dt);
        ang_speed = ang_PID.update(error.angular, dt);

        // apply limits
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

        // accelerate
        if (speeds.left - prev_speeds.left > accel_step) speeds.left = prev_speeds.left + accel_step;
        if (speeds.right - prev_speeds.right > accel_step) speeds.right = prev_speeds.right + accel_step;
        chassis_mutex.take();
        prev_speeds = speeds;
        chassis_mutex.give();

        // set motor speeds
        tank(speeds);

        // check exit conditions
        if (timeout > 0 && pros::millis() - start_time > timeout) break;
        if (error.linear < exit) break;

        // delay task
        pros::c::task_delay_until(&now, dt);
    }
}

void Chassis::move(Point target, Options opts) {
    // stop task if robot is already moving
    if (chassis_task)
        chassis_task->remove();
        delete chassis_task;

    // start task
    chassis_task = new pros::Task([this, target, opts] { move_task(target, opts); }, "chassis_task");

    // wait if not asynchroneous
    if (!opts.async.value_or(df_move_opts.async.value_or(false))) {
        wait();
    }
}

void Chassis::move(double target, Options options) {
    options.relative = true;
    move({target, 0}, options);
}

void Chassis::turn(Point target, Options options) {

}

void Chassis::turn(double target, Options options) {

}

void Chassis::tank(double left_speed, double right_speed) {
    left_motors.move_voltage(left_speed * 120);
    right_motors.move_voltage(right_speed * 120);
}

void Chassis::tank(Point speeds) {
    tank(speeds.left, speeds.right);
}

void Chassis::arcade(double linear, double angular) {
    double left_speed = linear + angular;
    double right_speed = linear - angular;
    tank(left_speed, right_speed);
}

void Chassis::arcade(pros::Controller& controller) {
    double linear = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double angular = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    arcade(linear, angular);
}

void Chassis::stop() {
    if (chassis_task) {
        chassis_task->remove();
        delete chassis_task;
        chassis_task = nullptr;
    }
    tank(0, 0);
}

void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_motors.set_brake_mode(mode);
    right_motors.set_brake_mode(mode);
}

} // namespace dom

/**
 * TODO: ignore timeout if equal to 0
 * TODO: add settle time
 * TODO: implement accel
 * TODO: implement settle exit for stuck robot
 * TODO: implement turn_task
 */