#include "main.h"

appa::Odom odom({2, 3},  // x tracker port
                {2, 1},  // y tracker port
                {13, 5}, // imu port(s)
                321.5,   // encoder ticks per unit (inches)
                {2, 0},  // tracker linear offset (inches)
                45);     // tracker angular offset (degrees)

appa::Config config(100,             // speed (%)
                    400,             // accel (%/s)
                    {10, 0, 0},      // linear PID
                    {10, 0, 0},      // angular PID
                    0.5,             // lead (%)
                    6.0,             // lookahead (in)
                    1.0,             // linear exit (in)
                    2.0,             // angular exit (deg)
                    2.0,             // angular deadzone (in)
                    {0.1, 0.5, 250}, // exit speed (in, deg, ms)
                    0,               // settle (ms)
                    0);              // timeout (ms)

appa::Chassis bot({-10, -9, 8, 3, -1},    // left motors
                  {17, 19, -18, -12, 11}, // right motors
                  odom,                   // odom
                  config);                // configuration

void initialize() {
    // start odometry with debugging
    odom.start();
    odom.debug = true;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    printf("autonomous started\n");
    odom.set(0, 0, 0);

    printf("running...");
    bot.move({24, 0}, {.speed = 50});
    printf("done\n");
    pros::delay(500);
}

void opcontrol() {
    pros::Controller master(CONTROLLER_MASTER);
    bot.set_brake_mode(MOTOR_BRAKE_COAST);
    printf("opcontrol started\n");

    while (true) {
        if (master.get_digital_new_press(DIGITAL_A)) {
            autonomous();
            bot.set_brake_mode(MOTOR_BRAKE_COAST);
        }

        bot.arcade(master);
        pros::delay(10);
    }
}