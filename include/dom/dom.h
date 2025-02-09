#pragma once

#include "api.h"
#include "utils.h"
#include <atomic>

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
	Pose pose;
	pros::Mutex mutex;

	pros::adi::Encoder x_tracker, y_tracker;
	pros::Imu imu;
	double tpi;

	double prev_x, prev_y;

	Point linear_offset;
	double angular_offset;

public:
	Odom(int x_port, int y_port, int imu_port, int tpi, Point linear_offset, double angular_offset);

	void task();

	void start();
	void reset(Pose pose = {0, 0, 0});
	void reset(double x, double y, double theta);
	void reset(Point point);
	void reset(double x, double y);
	Pose get();
	void set(Pose pose);
};

/* Chassis */
class Chassis {
private:
	pros::MotorGroup left_motors, right_motors;
	Odom& odom;
	Options default_move_options, default_turn_options;
	std::atomic<bool> is_driving{false};

public:
	Chassis(
		std::initializer_list<int8_t> left_motors,
		std::initializer_list<int8_t> right_motors,
		Odom& odom,
		Options default_move_options,
		Options default_turn_ptions
	);

	void init();
	void wait();
	/**
		* Moves the robot to a point or a distance ahead
		* @param target target distance or point
		* @param speed maximum speed of movement
		* @param options 
		* @param direction -1 reverse, 0 automatic, 1 forward
		* @param exit_dist maximum distance from target to exit
		* @param lin_PID pid constants for linear movement
		* @param ang_PID pid constants for angular movement
		*/
	void move(Point target, Options options = {});
	void move(double target, Options options = {});
	void turn(Point target, Options options = {});
	void turn(double target, Options options = {});

	void tank(double left_speed, double right_speed);
	void arcade(double linear, double angular);
	void arcade(pros::Controller& controller);
	void stop();
};