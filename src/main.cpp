#include "main.h"

dom::Odom odom({2, 3}, // x tracker
               {2, 1}, // y tracker
               13,     // imu
               321.5,  // tpi
               {5, 0}, // tracker linear offset
               45);    // tracker angular offset

dom::MoveConfig move_config(1.0,         // exit (inches)
                            5000,        // timeout (ms)
                            85,          // speed (%)
                            50,          // accel (%/s)
                            {10, 0, 0},  // linear pid gains
                            {50, 0, 0}); // angular pid gains

dom::TurnConfig turn_config(2.0,        // exit (degrees)
                            5000,       // timeout (ms)
                            50,         // speed (%)
                            50,         // accel (%/s)
                            {0, 0, 0}); // angular pid gains

dom::Chassis bot({-10, -9, 8, 3, -1},    // left motors
                 {17, 19, -18, -12, 11}, // right motors
                 odom,                   // odom
                 move_config,            // move configuration
                 turn_config);           // turn configuration

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