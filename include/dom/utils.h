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

struct Gains { double p, i, d; };

struct Options {
	std::optional<Direction> dir;

	std::optional<double> exit;
	std::optional<double> settle;
	std::optional<double> timeout;

	std::optional<double> speed;
	std::optional<double> accel;

	std::optional<Gains> lin_PID;
	std::optional<Gains> ang_PID;

	std::optional<bool> async;
	std::optional<bool> thru;
	std::optional<bool> relative;
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

	Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
	Pose(Point p, double theta) : x(p.x), y(p.y), theta(theta) {}

	Point p() const { return {x, y}; }
	double angle(const Point& other) const {
		Point diff = this->p() - other;
		return std::fmod(atan2(diff.y, diff.x) - this->theta, M_PI);
	}
};

double toRad(double deg);
double toDeg(double rad);