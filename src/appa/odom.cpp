#include "appa.h"

namespace appa {

/* Odom */
Odom::Odom(int8_t x_port, int8_t y_port, int8_t imu_port, int tpi, Point tracker_linear_offset,
           double tracker_angular_offset)
    : x_tracker(abs(x_port), abs(x_port) + 1, x_port < 0),
      y_tracker(abs(y_port), abs(y_port) + 1, y_port < 0),
      imu(imu_port),
      tpi(tpi),
      tracker_linear_offset(tracker_linear_offset),
      tracker_angular_offset(to_rad(tracker_angular_offset)) {}

Odom::Odom(std::array<int8_t, 2> x_port, std::array<int8_t, 2> y_port, int8_t imu_port, int tpi,
           Point tracker_linear_offset, double tracker_angular_offset)
    : x_tracker({x_port[0], abs(x_port[1]), abs(x_port[1]) + 1}, x_port[1] < 0),
      y_tracker({y_port[0], abs(y_port[1]), abs(y_port[1]) + 1}, y_port[1] < 0),
      imu(imu_port),
      tpi(tpi),
      tracker_linear_offset(tracker_linear_offset),
      tracker_angular_offset(to_rad(tracker_angular_offset)) {}

void Odom::task() {
    printf("odom task started\n");
    Pose prev_track = {0.0, 0.0, 0.0};
    uint32_t now = pros::millis();

    int count = 0;

    while (true) {
        // get current sensor values
        Pose track = {
            x_tracker.get_value() / tpi, y_tracker.get_value() / tpi, to_rad(-imu.get_rotation())};

        // calculate change in sensor values
        Point dtrack = {track.x - prev_track.x, track.y - prev_track.y};
        double dtheta = track.theta - prev_track.theta;

        // set previous sensor values for next loop
        prev_track = track;

        // arc approximation
        if (dtheta != 0) dtrack *= 2 * sin(dtheta / 2) / dtheta;

        // rotate tracker differential to global frame
        dtrack = dtrack.rotate(track.theta + tracker_angular_offset);

        // update tracker pose
        odom_mutex.take();
        odom_pose += dtrack;
        odom_pose.theta = track.theta;
        odom_mutex.give();

        // debugging
        // if (!(count++ % 50)) debug();

        // loop every 5 ms
        pros::c::task_delay_until(&now, 5);
    }
}

void Odom::start() {
    printf("calibrating imu...");
    if (imu.reset(true) != 1) {
        printf("\nERROR: IMU reset failed with error code %d\n", errno);
        printf("odometry was not started\n");
        return;
    }
    imu.set_data_rate(5);
    printf("done\n");

    set({0.0, 0.0, 0.0});
    if (odom_task == nullptr)
        odom_task = new pros::Task([this] { task(); }, 16, TASK_STACK_DEPTH_DEFAULT, "odom_task");
}

Pose Odom::get() {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    // translate the tracker offsets to the global frame
    return odom_pose + tracker_linear_offset.rotate(odom_pose.theta);
}

Pose Odom::get_local() {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    return odom_pose;
}

void Odom::set(Pose pose) {
    odom_mutex.lock();
    pose -= tracker_linear_offset.rotate(odom_pose.theta);
    odom_mutex.unlock();
    set_local(pose);
}

void Odom::set_local(Pose pose) {
    const std::lock_guard<pros::Mutex> lock(odom_mutex);
    if (std::isnan(pose.x)) pose.x = odom_pose.x;
    if (std::isnan(pose.y)) pose.y = odom_pose.y;
    if (std::isnan(pose.theta)) pose.theta = odom_pose.theta;
    imu.set_rotation(-pose.theta);
    odom_pose = pose;
}

void Odom::set_x(double x) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose.x = x;
}

void Odom::set_y(double y) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    odom_pose.y = y;
}

void Odom::set_theta(double theta) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    imu.set_rotation(-theta);
    odom_pose.theta = theta;
}

void Odom::set(Point point, double theta) { set({point.x, point.y, theta}); }
void Odom::set(double x, double y, double theta) { set({x, y, theta}); }

void Odom::set_offset(Point linear) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    tracker_linear_offset = linear;
}

void Odom::debug() {
    // if (odom_task)
    //     printf("odom_task priority: %d\n", odom_task->get_priority());
    Pose pose = get();
    printf("x: %.2f, y: %.2f, theta: %.2f\n", pose.x, pose.y, to_deg(pose.theta));
}

} // namespace appa