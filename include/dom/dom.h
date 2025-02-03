#pragma once

#include "../api.h"
#include "utils.h"
#include <mutex>
#include <optional>
#include <cmath>

/* PID */
class PID {
	double kp, ki, kd;
	double prev_error, total_error;

public:
	PID(double kp, double ki, double kd);
	PID(PIDConst k);
	void reset(double error = 0.0);
	double update(double error, double dt);
};

/* Odom */
class Odom {
private:
	Pose pose;
	std::mutex mutex;

	pros::adi::AnalogIn x_tracker, y_tracker;
	pros::Imu imu;
	double tpi;

	double prev_x, prev_y;

	Point linear_offset;
	double angular_offset;

public:
	Odom(double linear_offset, double angular_offset);

	void task();
	void customTask();

	void start();
	void reset(Pose pose = {0, 0, 0});
	void get();
	void set(Pose pose);
};

/* Chassis */
class Chassis {
private:
	std::unique_ptr<pros::MotorGroup> left_motors, right_motors;

	Odom odom;

	PIDConst default_move_PID, default_ang_PID, default_turn_PID;

	Options defaultMoveOptions, defaultTurnOptions;

public:
  	Chassis(
		std::initializer_list<int8_t> left_motors,
		std::initializer_list<int8_t> right_motors,

		PIDConst default_lin_PID,
		PIDConst default_ang_PID,
		PIDConst default_turn_PID,
		double default_exit_dist,
		double default_exit_angle,
		double default_exit_timeout
	);

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
	void move(Point target, double speed = 100, Options options = {});
	void move(double target, double speed = 100, Options options = {});
    void turn(Point target, double speed = 100, Options options = {});
    void turn(double target, double speed = 100, Options options = {});

	void tank(double left_speed, double right_speed);
	void arcade(double linear, double angular);
	void stop();
};