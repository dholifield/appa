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

    set({0, 0, 0});
}

void Odom::task() {
    Pose prev_track = {0, 0, 0};
    uint32_t now = pros::millis();

    while(true) {
        // get current sensor values
        Pose track = {x_tracker.get_value() / tpi,
                      y_tracker.get_value() / tpi,
                      toRad(imu.get_rotation())};

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
        odom_pose.x += dtrack.x;
        odom_pose.y += dtrack.y;
        odom_pose.theta = track.theta;
        lock.unlock();
        
        // loop every 2 ms
        pros::task_delay_until(&now, 2);
    }
}

void Odom::start() {
    odom_task = new pros::Task([this] { task(); }, 16, TASK_STACK_DEPTH_DEFAULT, "odom_task");
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
    imu.set_rotation(theta);
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
