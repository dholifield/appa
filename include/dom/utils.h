#pragma once

#include "dom.h"

// Utils //
enum Direction {
	FORWARD = 1,
	AUTO = 0,
	REVERSE = -1,
	CCW = 1,
	CW = -1
};

struct Point {
	double x, y;

	Point operator+(const Point& other) const {
        return Point({x + other.x, y + other.y});
    }
	Point operator-(const Point& other) const {
        return Point({x - other.x, y - other.y});
    }
	double angle(const Point& other, double offset = 0.0) const {
    	Point diff = *this - other;
    	return std::fmod(atan2(diff.y, diff.x) - offset, M_PI);
	}
	double dist(const Point& other) const {
    	Point diff = *this - other;
		return sqrt(diff.x * diff.x + diff.y * diff.y);
	}
};

struct Pose { 
	double x, y, theta; 

	Pose(double x_ = 0, double y_ = 0, double theta_ = 0) : x(x_), y(y_), theta(theta_) {}
	Pose(Point p, double theta_) : x(p.x), y(p.y), theta(theta_) {}

	Point p() const { return {x, y}; }
	double angle(const Point& other) const {
    	Point diff = this->p() - other;
    	return std::fmod(atan2(diff.y, diff.x) - this->theta, M_PI);
	}
};

struct PIDConst { double p, i, d; };

struct Options {
	std::optional<Direction> dir;

	std::optional<double> exit;
	std::optional<double> settle;
	std::optional<double> timeout;

	std::optional<double> accel;

	std::optional<PIDConst> lin_PID;
	std::optional<PIDConst> ang_PID;

	std::optional<bool> async;
	std::optional<bool> thru;
	std::optional<bool> relative;
};

double toRad(double deg);
double toDeg(double rad);