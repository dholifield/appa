#include "main.h"

dom::Odom odom({2, 3}, // x tracker
               {2, 1}, // y tracker
               13,     // imu
               321.5,  // tpi
               {5, 0}, // tracker linear offset
               45);    // tracker angular offset

dom::Options move_options = {.exit = 1.0,            // inches
                             .timeout = 5000,        // ms
                             .speed = 85,            // %
                             .accel = 50,            // %/s
                             .lin_PID = (10, 0, 0),  // linear pid gains
                             .ang_PID = (50, 0, 0)}; // angular pid gains

dom::Options turn_options = {.exit = 2.0,           // degrees
                             .timeout = 5000,       // ms
                             .speed = 50,           // %
                             .accel = 50,           // %/s
                             .ang_PID = (0, 0, 0)}; // angular pid gains

dom::Chassis bot({-10, -9, 8, 3, -1},    // left motors
                 {17, 19, -18, -12, 11}, // right motors
                 odom,                   // odom
                 move_options,           // default move options
                 turn_options);          // default turn options

void initialize() {
    // start odometry
    odom.start();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    printf("autonomous started\n");
    bot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    odom.set(0, 0, 90);
    bot.move((0, 24), {.speed = 100});

    bot.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    bot.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    printf("opcontrol started\n");

    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) autonomous();
        bot.arcade(master);
        // odom.debug();
        pros::delay(10);
    }
}