#include "main.h"

appa::Odom odom({2, 3}, // x tracker port
                {2, 1}, // y tracker port
                13,     // imu port
                321.5,  // tracker encoder ticks per inch
                {5, 0}, // tracker linear offset (inches)
                45);    // tracker angular offset (degrees)

appa::MoveConfig move_config(1.0,         // exit (inches)
                             85,          // speed (%)
                             0.5,         // lead (%)
                             6,           // lookahead (inches)
                             {10, 0, 0},  // linear pid gains
                             {50, 0, 0}); // angular pid gains

appa::TurnConfig turn_config(2.0,         // exit (degrees)
                             50,          // speed (%)
                             {10, 0, 0}); // angular pid gains

appa::Options default_options = {.timeout = 5000, // ms
                                 .accel = 50};    // %/s

appa::Chassis bot({-10, -9, 8, 3, -1},    // left motors
                  {17, 19, -18, -12, 11}, // right motors
                  odom,                   // odom
                  move_config,            // move configuration
                  turn_config,            // turn configuration
                  default_options);       // default options

void initialize() {
    // start odometry
    odom.start();
}

void disabled() {}

void competition_initialize() {}

appa::Options thru = {.exit = 4, .thru = true};
appa::Options fast = {.speed = 100, .accel = 0};
appa::Options precise = {.speed = 50, .accel = 20, .lin_PID = appa::Gains{5, 0, 0}};

void autonomous() {
    printf("autonomous started\n");
    bot.set_brake_mode(MOTOR_BRAKE_HOLD);
    odom.set(0, 0, 90);

    bot.move({24, 12});                            // move with default options
    bot.move({50, 0}, fast << thru);               // move with thru and fast options
    bot.move({10, 0, 90}, precise, {.lead = 0.3}); // move with precise options plus different lead
}

void opcontrol() {
    pros::Controller master(CONTROLLER_MASTER);
    bot.set_brake_mode(MOTOR_BRAKE_COAST);
    printf("opcontrol started\n");

    while (true) {
        if (master.get_digital_new_press(DIGITAL_A)) {
            autonomous();
            bot.set_brake_mode(MOTOR_BRAKE_HOLD);
        }

        bot.arcade(master);
        // odom.debug();
        pros::delay(10);
    }
}