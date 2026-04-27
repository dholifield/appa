#include "appa.h"

namespace appa {

/* Odom */
Odom::Odom(Tracker& tracker, Point tracker_linear_offset, double tracker_angular_offset)
    : tracker_(tracker),
      tracker_linear_offset_(tracker_linear_offset),
      tracker_angular_offset_(to_rad(tracker_angular_offset)) {}

Odom::~Odom() { stop(); }

void Odom::task() {
    printf("odom task started\n");
    Pose prev_track = tracker_.get();
    uint32_t now = pros::millis();

    int count = 0;
    running_.store(true);

    while (running_.load()) {
        // get current sensor values
        Pose track = tracker_.get();

        // calculate change in sensor values
        Point dtrack = track - prev_track;
        double dtheta = track.theta - prev_track.theta;

        // set previous sensor values for next loop
        prev_track = track;

        // arc approximation
        if (dtheta != 0) dtrack *= 2 * sin(dtheta / 2) / dtheta;

        // rotate tracker differential to global frame
        dtrack = dtrack.rotate(track.theta + tracker_angular_offset_);

        // update tracker pose
        {
            std::lock_guard<pros::Mutex> lock(odom_mutex_);
            odom_pose_ += dtrack;
            odom_pose_.theta = track.theta + angular_offset_;
        }

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

void Odom::start(Pose pose) {
    set(pose);
    if (running_.load()) stop();
    odom_task_ = new pros::Task([this] { task(); }, 12, TASK_STACK_DEPTH_DEFAULT, "odom_task");
}

void Odom::stop() {
    running_.store(false);
    if (odom_task_ != nullptr) {
        odom_task_->join();
        delete odom_task_;
        odom_task_ = nullptr;
    }
}

Pose Odom::get() const {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    // translate the tracker offsets to the global frame
    return odom_pose_ + tracker_linear_offset_.rotate(odom_pose_.theta);
}

Pose Odom::get_local() const {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    return odom_pose_;
}

void Odom::set(Pose pose) {
    {
        std::lock_guard<pros::Mutex> lock(odom_mutex_);
        pose -= tracker_linear_offset_.rotate(odom_pose_.theta);
    }
    set_local(pose);
}

void Odom::set_local(Pose pose) {
    const std::lock_guard<pros::Mutex> lock(odom_mutex_);
    if (std::isnan(pose.x)) pose.x = odom_pose_.x;
    if (std::isnan(pose.y)) pose.y = odom_pose_.y;
    if (std::isnan(pose.theta)) pose.theta = odom_pose_.theta;
    else angular_offset_ = pose.theta - odom_pose_.theta;
    odom_pose_ = pose;
}

void Odom::set_x(double x) {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    odom_pose_.x = x;
}

void Odom::set_y(double y) {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    odom_pose_.y = y;
}

void Odom::set_theta(double theta) {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    angular_offset_ = theta - odom_pose_.theta;
    odom_pose_.theta = theta;
}

void Odom::set(Point point, double theta) { set({point.x, point.y, theta}); }
void Odom::set(double x, double y, double theta) { set({x, y, theta}); }

void Odom::set_offset(Point linear) {
    std::lock_guard<pros::Mutex> lock(odom_mutex_);
    tracker_linear_offset_ = linear;
}

/* Tracker */
// Two Wheels + IMU
TwoWheelIMU::TwoWheelIMU(EncoderWheel x_encoder, EncoderWheel y_encoder, Imu imu_port)
    : imu(imu_port), x_encoder(x_encoder), y_encoder(y_encoder) {}

Pose TwoWheelIMU::get() {
    double x = x_encoder.get_value();
    double y = y_encoder.get_value();
    double theta = to_rad(imu.get());
    return Pose(x, y, theta);
}

void TwoWheelIMU::calibrate(bool blocking) {
    printf("calibrating tracker...");
    if (!imu.calibrate(blocking)) {
        if (blocking) printf("\nERROR: Tracker failed to initialize: %d\n", errno);
        else printf("WARNING: Tracker calibrating asynchronously\n");
        return;
    }
    imu.set(0.0);
    printf("done\n");
}

// Three Wheels
ThreeWheel::ThreeWheel(EncoderWheel lx_encoder, EncoderWheel rx_encoder, EncoderWheel y_encoder,
                       double width)
    : width(width), lx_encoder(lx_encoder), rx_encoder(rx_encoder), y_encoder(y_encoder) {}

Pose ThreeWheel::get() {
    double l = lx_encoder.get_value();
    double r = rx_encoder.get_value();
    double y = y_encoder.get_value();
    double theta = (r - l) / width;
    double x = (r + l) / 2;
    return Pose(x, y, theta);
}

} // namespace appa

/**
 * TODO:
 */