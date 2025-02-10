#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>
#include <optional>
#include <mutex>

/* PID */
class PID {
	double kp, ki, kd;
	double prev_error, total_error;

public:
	PID(double kp, double ki, double kd);
	PID(Gains k);
	void reset(double error = 0.0);
	double update(double error, double dt);
};

/* Odom */
class Odom {
private:
	Pose odom_pose;
	pros::Mutex odom_mutex;
	pros::Task odom_task;

	pros::adi::Encoder x_tracker, y_tracker;
	pros::Imu imu;
	double tpi;

	Point tracker_linear_offset;
	double tracker_angular_offset;

public:
	Odom(int x_port, int y_port, int imu_port, int tpi, Point tracker_linear_offset, double tracker_angular_offset);

	void task();
	void start();
	void suspend();
	void resume();

	Pose get();
	Pose getLocal();
	void set(Pose pose);
	void set(Point point, double theta);
	void set(double x, double y, double theta);
	void setPoint(Point point);
	void setPoint(double x, double y);
	void setTheta(double theta);

	void setOffset(Point linear);

	void debug();
};

/* Chassis */
class Chassis {
private:
	pros::MotorGroup left_motors, right_motors;
	Odom& odom;
	Options df_move_opts, df_turn_opts;

	pros::Task chassis_task;
	std::atomic<bool> is_driving{false};

public:
	Chassis(
		std::initializer_list<int8_t> left_motors,
		std::initializer_list<int8_t> right_motors,
		Odom& odom,
		Options default_move_options,
		Options default_turn_ptions
	);

	/**
	 * Move the robot to a point or a distance ahead
	 * @param target target distance or point
	 * @param options additional options
	 */
	void move(Point target, Options options = {});
	void move(double target, Options options = {});
	void turn(Point target, Options options = {});
	void turn(double target, Options options = {});

	void wait();

	void tank(double left_speed, double right_speed);
	void arcade(double linear, double angular);
	void arcade(pros::Controller& controller);
	void stop();
};