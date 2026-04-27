#include "main.h"

pros::Controller master(CONTROLLER_MASTER);

appa::TwoWheelIMU tracker({2, 3, 321.5}, // x encoder wheel
                          {2, 1, 321.5}, // y encoder wheel
                          {13, 5});      // imus

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
    tracker.calibrate();
    odom.start();
    // odom.debug = true;
    // bot.debug = true;
}

void disabled() {}

void competition_initialize() {}

const appa::Options goal_opts = {.dir = appa::FORWARD, .offset = 10};
const appa::Target goal_1 = {24, 72, goal_opts};
const appa::Target goal_2 = {48, 72, goal_opts};

void autonomous() {
    printf("autonomous started\n");
    odom.set(0, 0, 0);

    // claw.open();
    bot.move(goal_1, {.speed = 100, .lin_exit = 10});
    // claw.close();
    bot.move(goal_1, {.speed = 100});

    bot.turn(180, {.turn = appa::CCW, .relative = true});

    bot.move({24, 24}, {.speed = 50});
    master.rumble("-");
    pros::delay(500);
}

void opcontrol() {
    bot.set_brake_mode(MOTOR_BRAKE_COAST);
    printf("opcontrol started\n");
    master.rumble(".");

    while (true) {
        if (!pros::competition::is_connected() && master.get_digital_new_press(DIGITAL_A)) {
            bot.set_brake_mode(MOTOR_BRAKE_HOLD);
            autonomous();
            bot.set_brake_mode(MOTOR_BRAKE_COAST);
        }

        bot.arcade(master);
        pros::delay(10);
    }
}