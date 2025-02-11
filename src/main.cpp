#include "main.h"

dom::Odom odom(3, 1, 5, 321.5, {2, 0}, 45);

// dom::Options move_options = {.exit = 1.0,	// inches
// 						.settle = 100,		// ms
// 						.timeout = 10000,	// ms
// 						.speed = 85,		// %
// 						.accel = 50,		// %/s
// 						.lin_PID = Gains{1, 1, 1},
// 						.ang_PID = Gains{1, 1, 1}
// 					};


// dom::Options turn_options = {.exit = 2.0,	// degrees
// 						.settle = 100,		// ms
// 						.timeout = 5000,	// ms
// 						.speed = 50,		// %
// 						.accel = 50,		// %/s
// 						.ang_PID = Gains{1, 1, 1}
// 					};


// dom::Chassis bot({1, 2, 3, 4},
// 			{5, 6, 7, 8},
// 			odom,
// 			move_options,
// 			turn_options
// 		);

void initialize() {
	// odom.start();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// odom.reset(45, 10, 90);
	// bot.move({45, 80}, {.speed = 100, .lin_PID = Gains{5, 1, 0}, .async = true});
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
		// bot.arcade(master);
		// odom.debug();
		pros::delay(10);
	}
}