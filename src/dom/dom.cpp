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
        tracker_angular_offset(tracker_angular_offset),
        imu(imu_port),
        x_tracker(abs(x_port), abs(x_port) + 1, x_port < 0),
        y_tracker(abs(y_port), abs(y_port) + 1, y_port < 0),
        odom_task(nullptr) {

    set({0, 0, 0});
}

void Odom::task() {
    Pose prev_pose = get();
    Pose prev_track = {0, 0, 0};

    while(true) {
        Pose track = {x_tracker.get_value() / tpi, y_tracker.get_value() / tpi, toRad(imu.get_rotation())};
        double dx = track.x - prev_track.x;
        double dy = track.y - prev_track.y;
        double dtheta = track.theta - prev_track.theta;

        prev_track = track;



    }
}

void Odom::start() {
    odom_task = new pros::Task([this] { task(); }, 16, TASK_STACK_DEPTH_DEFAULT, "odom_task");
}

void Odom::stop() {
    odom_task.suspend();
}

Pose Odom::get() {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    return odom_pose;
}

void Odom::set(Pose pose) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose = pose;
}

void Odom::setPoint(Point point) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose.x = point.x;
    odom_pose.y = point.y;
}

void Odom::setTheta(double theta) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose.theta = theta;
}

void Odom::set(Point point, double theta) { set({point.x, point.y, theta}); }
void Odom::set(double x, double y, double theta) { set({x, y, theta}); }
void Odom::setPoint(double x, double y) { setPoint({x, y}); }


void Odom::debug() {
    printf("odom_task priorety: %d\n", odom_task.get_priority());
}

/* Chassis */
