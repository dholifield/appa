#include "dom.h"

/* PID */
PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) { reset(0.0); }
PID::PID(Gains k) : kp(k.p), ki(k.i), kd(k.d) { reset(0.0); }

void PID::reset(double error) {
    prev_error = error;
    total_error = 0.0;
}

double PID::update(double error, double dt) {
    double derivative = (error - prev_error) / dt;
    total_error += error * dt;
    prev_error = error;
    return (kp * error) + (ki * total_error) + (kd * derivative);
}

/* Odom */
Odom::Odom(int x_port, int y_port, int imu_port, int tpi, Point tracker_linear_offset, double tracker_angular_offset) :
        tpi(tpi),
        tracker_linear_offset(tracker_linear_offset),
        tracker_angular_offset(toRad(tracker_angular_offset)),
        imu(imu_port),
        x_tracker(abs(x_port), abs(x_port) + 1, x_port < 0),
        y_tracker(abs(y_port), abs(y_port) + 1, y_port < 0),
        odom_task(nullptr) {
    
    imu.reset(true);
    imu.set_data_rate(5);

    set({0, 0, 0});
}

void Odom::task() {
    Pose prev_track = {0, 0, 0};
    uint32_t now = pros::millis();

    while(true) {
        // get current sensor values
        Pose track = {x_tracker.get_value() / tpi,
                      y_tracker.get_value() / tpi,
                      toRad(-imu.get_rotation())};

        // calculate change in sensor values
        Point dtrack = {track.x - prev_track.x,
                        track.y - prev_track.y};
        double dtheta = track.theta - prev_track.theta;

        // set previous sensor values for next loop
        prev_track = track;
        
        // arc approximation
        if (dtheta != 0)
            dtrack *= 2 * sin(dtheta / 2) / dtheta;

        // rotate tracker differential to global frame
        dtrack = dtrack.rotate(track.theta + tracker_angular_offset);

        // update tracker pose
        std::lock_guard<pros::Mutex> lock(odom_mutex);
        odom_pose += dtrack;
        odom_pose.theta = track.theta;
        lock.unlock();
        
        // loop every 2 ms
        pros::task_delay_until(&now, 2);
    }
}

void Odom::start() {
    odom_task = pros::Task([this] { task(); }, 16, TASK_STACK_DEPTH_DEFAULT, "odom_task");
}

void Odom::suspend() {
    odom_task.suspend();
}

void Odom::resume() {
    odom_task.resume();
}

Pose Odom::get() {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    // translate the tracker offsets to the global frame
    return odom_pose + tracker_linear_offset.rotate(odom_pose.theta);
}

Pose Odom::getLocal() {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    return odom_pose;
}

void Odom::set(Pose pose) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    setTheta(pose.theta);
    odom_pose = pose;
}

void Odom::setPoint(Point point) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose.x = point.x;
    odom_pose.y = point.y;
}

void Odom::setTheta(double theta) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    imu.set_rotation(-theta);
    odom_pose.theta = theta;
}

void Odom::set(Point point, double theta) { set({point.x, point.y, theta}); }
void Odom::set(double x, double y, double theta) { set({x, y, theta}); }
void Odom::setPoint(double x, double y) { setPoint({x, y}); }

void Odom::setOffset(Point linear) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    tracker_linear_offset = linear;
}

void Odom::debug() {
    printf("odom_task priority: %d\n", odom_task.get_priority());
    Pose pose = get();
    printf("x: %.2f, y: %.2f, theta: %.2f\n", pose.x, pose.y, pose.theta);
}

/* Chassis */
Chassis::Chassis(std::initializer_list<int8_t> left_motors,
                 std::initializer_list<int8_t> right_motors,
                 Odom& odom,
                 Options default_move_options,
                 Options default_turn_options)
    : left_motors(left_motors),
      right_motors(right_motors),
      odom(odom),
      default_move_options(default_move_options),
      default_turn_options(default_turn_options),
      chassis_task(nullptr) {}

void Chassis::wait() {
    chassis_task.join();
}

void Chassis::move(Point target, Options opts) {
    // stop task if robot is already moving
    if (chassis_task.get_state() != pros::E_TASK_STATE_DELETED)
        chassis_task.remove();

    // start task
    chassis_task = pros::Task([this] {
        // sort out options
        Direction dir = opts.dir.value_or(df_move_opts.dir.value_or(AUTO));

        double exit = opts.exit.value_or(df_move_opts.exit.value_or(1.0));
        int settle = opts.settle.value_or(df_move_opts.settle.value_or(250));
        int timeout = opts.timeout.value_or(df_move_opts.timeout.value_or(10000));

        double speed = opts.speed.value_or(df_move_opts.speed.value_or(100));
        double accel = opts.accel.value_or(df_move_opts.accel.value_or(50));

        PID lin_PID(opts.lin_PID.value_or(df_move_opts.lin_PID.value_or(Gains{10.0, 0.0, 0.0})));
        PID ang_PID(opts.ang_PID.value_or(df_move_opts.ang_PID.value_or(Gains{100.0, 0.0, 0.0})));

        bool async = opts.async.value_or(df_move_opts.async.value_or(false));
        bool thru = opts.thru.value_or(df_move_opts.thru.value_or(false));
        bool relative = opts.relative.value_or(df_move_opts.relative.value_or(false));

        // set up variables
        double dt = 0.01; // 10 ms
        Pose pose = odom.get();
        if (relative) target = pose.p() + target.rotate(pose.theta);

        // run control loop
        while(true) {
            // get current position
            pose = odom.get();

            // calculate error
            double lin_error = pose.dist(target);
            double ang_error = pose.angle(target);

            // calculate PID
            double lin_output = lin_PID.update(lin_error, dt);
            double ang_output = ang_PID.update(ang_error, dt);

            // apply limits
            lin_output = lin_output > speed ? speed : lin_output < -speed ? -speed : lin_output;
            ang_output = 

            // determine direction
            bool reverse = dir == REVERSE || (dir == AUTO && ang_output);

            // loop every 2 ms
            pros::delay((int)(dt * 1000));
        }
    });
    // wait if not asynchroneous
    if (!async) {
        chassis_task.join();
    }

}


void Chassis::move(double target, Options options) {

}

void Chassis::turn(Point target, Options options) {

}

void Chassis::turn(double target, Options options) {

}

void Chassis::tank(double left_speed, double right_speed) {
    left_motors.move_voltage(left_speed * 120);
    right_motors.move_voltage(right_speed * 120);
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
    tank(0, 0);
}