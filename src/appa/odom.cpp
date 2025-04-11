#include "appa.h"

namespace appa {

/* Odom */
Odom::Odom(Tracker& tracker, Point tracker_linear_offset, double tracker_angular_offset)
    : tracker(tracker),
      tracker_linear_offset(tracker_linear_offset),
      tracker_angular_offset(to_rad(tracker_angular_offset)) {}

Odom::~Odom() {
    if (odom_task) {
        odom_task->remove();
        delete odom_task;
        odom_task = nullptr;
    }
}
void Odom::task() {
    printf("odom task started\n");
    Pose prev_track = {0.0, 0.0, 0.0};
    uint32_t now = pros::millis();

    int count = 0;

    while (true) {
        // get current sensor values
        Pose track = tracker.get();

        // calculate change in sensor values
        Point dtrack = track - prev_track;
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
        odom_pose.theta = track.theta + angular_offset;
        odom_mutex.give();

        // debugging
        if (!(++count % 20) && debug.load()) {
            Pose p = get();
            printf("\r(%6.2f,%6.2f,%7.2f)", p.x, p.y, to_deg(p.theta));
            fflush(stdout);
            count = 0;
        }

        // loop every 5 ms
        pros::c::task_delay_until(&now, 5);
    }
}

void Odom::start() {
    printf("calibrating tracker...");
    if (!tracker.init()) {
        printf("\nERROR: Tracker failed to initialize %d\nodometry was not started\n", errno);
        return;
    }
    printf("done\n");

    set({0.0, 0.0, 0.0});
    if (odom_task) {
        odom_task->remove();
        delete odom_task;
    }
    odom_task = new pros::Task([this] { task(); }, 16, TASK_STACK_DEPTH_DEFAULT, "odom_task");
}

Pose Odom::get() const {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    // translate the tracker offsets to the global frame
    return odom_pose + tracker_linear_offset.rotate(odom_pose.theta);
}

Pose Odom::get_local() const {
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
    else angular_offset = pose.theta - odom_pose.theta;
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
    angular_offset = theta - odom_pose.theta;
    odom_pose.theta = theta;
}

void Odom::set(Point point, double theta) { set({point.x, point.y, theta}); }
void Odom::set(double x, double y, double theta) { set({x, y, theta}); }

void Odom::set_offset(Point linear) {
    std::lock_guard<pros::Mutex> lock(odom_mutex);
    tracker_linear_offset = linear;
}

/* Tracker */
// Two Wheels + IMU
TwoWheelIMU::TwoWheelIMU(Encoder x_encoder, Encoder y_encoder, Imu imu_port, double tpu)
    : imu(std::move(imu_port)),
      tpu(tpu),
      x_encoder(std::move(x_encoder)),
      y_encoder(std::move(y_encoder)) {}

Pose TwoWheelIMU::get() {
    double x = x_encoder.get_value() * tpu;
    double y = y_encoder.get_value() * tpu;
    double theta = to_rad(imu.get());
    return Pose(x, y, theta);
}

bool TwoWheelIMU::init() {
    if (!imu.calibrate()) return false;
    imu.set(0.0);
    return true;
}

// Three Wheels
ThreeWheel::ThreeWheel(Encoder lx_encoder, Encoder rx_encoder, Encoder y_encoder, double tpu,
                       double width)
    : tpu(tpu),
      width(width),
      lx_encoder(std::move(lx_encoder)),
      rx_encoder(std::move(rx_encoder)),
      y_encoder(std::move(y_encoder)) {}

Pose ThreeWheel::get() {
    double l = lx_encoder.get_value() * tpu;
    double r = rx_encoder.get_value() * tpu;
    double y = y_encoder.get_value() * tpu;
    double theta = (r - l) / width;
    double x = (r + l) / 2;
    return Pose(x, y, theta);
}

} // namespace appa

/**
 * TODO:
 */