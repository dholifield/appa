#include "main.h"
#include "appa/utils.h"

pros::Controller master(CONTROLLER_MASTER);

appa::TwoWheelIMU tracker({2, 3},  // x encoder
                          {2, 1},  // y encoder
                          {13, 5}, // imus
                          321.5);  // tpu (ticks per inch)

appa::Odom odom(tracker, // tracker
                {2, 0},  // linear offset (inches)
                45);     // angular offset (degrees)

appa::Config config(100,              // speed (%)
                    500,              // accel (%/s)
                    {12, 0, 1},       // linear PID
                    {120, 5, 10},     // angular PID
                    0.5,              // lead (%)
                    6.0,              // lookahead (in)
                    0.5,              // linear exit (in)
                    1.0,              // angular exit (deg)
                    6.0,              // angular deadzone (in)
                    {0.05, 0.1, 250}, // exit speed (in, deg, ms)
                    50,               // settle (ms)
                    0);               // timeout (ms)

appa::Chassis bot({-10, -9, 8, 3, -1},    // left motors
                  {17, 19, -18, -12, 11}, // right motors
                  odom,                   // localization
                  config);                // configuration

void initialize() {
    // start odometry
    odom.start();
    // odom.debug = true;
    // bot.debug = true;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    printf("autonomous started\n");
    // odom.set(0, 0, 0);

    bot.move({24, 24}, {.speed = 50});
    master.rumble("-");
    pros::delay(500);
}

void opcontrol() {
    bot.set_brake_mode(MOTOR_BRAKE_COAST);
    printf("opcontrol started\n");
    master.rumble(".");

    while (true) {
        if (master.get_digital_new_press(DIGITAL_A)) {
            bot.set_brake_mode(MOTOR_BRAKE_HOLD);
            autonomous();
            bot.set_brake_mode(MOTOR_BRAKE_COAST);
        }

        bot.arcade(master);
        pros::delay(10);
    }
}