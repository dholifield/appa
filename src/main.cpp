#include "main.h"

dom::Odom odom({2, 3},	// x tracker
			   {2, 1},	// y tracker
			   13,		// imu
			   321.5,	// tpi
			   {5, 0},	// tracker linear offset
			   45 );	// tracker angular offset

dom::Options move_options = {.exit = 1.0,		// inches
							 .timeout = 0,	// ms
							 .speed = 85,		// %
							 .accel = 50,		// %/s
							 .lin_PID = dom::Gains{10, 0, 0},	// linear pid gains
							 .ang_PID = dom::Gains{50, 0, 0} };	// angular pid gains

dom::Options turn_options = {.exit = 2.0,		// degrees
							 .timeout = 5000,	// ms
							 .speed = 50,		// %
							 .accel = 50,		// %/s
							 .ang_PID = dom::Gains{0, 0, 0} };	// angular pid gains

dom::Chassis bot({-10, -9, 8, 3, -1},	// left motors
				 {17, 19, -18, -12, 11},// right motors
				 odom,			 		// odom
				 move_options,	 		// default move options
				 turn_options ); 		// default turn options

void initialize() {
	odom.start();
	bot.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// odom.set(45, 10, 90);
	// bot.move({45, 80}, {.speed = 100, .lin_PID = dom::Gains{5, 1, 0}, .async = true});
	// // do something while driving
	// bot.wait();
	// bot.move({45, 20});

	// bot.turn({20, 20});
	// bot.move({20, 20}, {.speed = 80});
	// bot.move(-10, {.speed = 20});
}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	printf("opcontrol started\n");

	while (true) {
		bot.arcade(master);
		// odom.debug();
		pros::delay(10);
	}
}