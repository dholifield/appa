# dom
A simple and intuitive VEX chassis movement library for [PROS](https://pros.cs.purdue.edu/). 

The intent of this library it to make chassis movement both intuitive to use and understand. Only recommended configurations are supported, but it is designed to be easy to build upon for custom implementations. 

# Installation

1. Download the most recent [template](https://github.com/dholifield/dom/releases) zip file (once released lol)

2. Run this command in the terminal `pros c fetch dom@<version>.zip`

3. Apply the library to your project with `pros c apply dom`

4. Add `#include "dom/dom.h"` in your `main.h`


# Usage
This library is comprised of two main components: odometry for position tracking and a chassis for robot movement.

## Odometry
[Odometry](https://wiki.purduesigbots.com/software/odometry) is used for keeping track of the robots position at all times. The supported configuration is for two tracker wheels, one in the x (forward) and one in the y (left) direction, in combination with an IMU. Here is how to create an instance of odom:
```c++
dom::Odom odom({11, 1}, // x tracker port
               {11, 3}, // y tracker port
               12,      // imu port
               300,     // tracker encoder ticks per inch (tpi)
               {5, 0},  // tracker linear offset (inches)
               45);     // tracker angular offset (degrees)
```

- The tracker ports can be `port` or `{expander port, port}`
- TPI should be experimentally determined by moving the robot a known distance and recording the encoder output `ticks / distance`
- The linear offset is `{x, y}` which is the distance from your tracking center to your center of mass
- The angular offset can be used for [angled tracker wheel](https://www.youtube.com/watch?v=TqMNuXfKgMc) configurations, as long as the two wheels are perpendicular

To start odometry, simply call `odom.start()`, usually during initialization.

```c++
void initialize() {
    odom.start();
}
```

## Chassis
The chassis is whats used to actually control the robot. Only differential drive robots are supported in this library. Most holonomic drives will work fine, but won't take advantage of its capabilities. The chassis contains configurations for the different movement types as well as optional options. Information on tuning a PID can be found [here](https://wiki.purduesigbots.com/software/control-algorithms/pid-controller). Here is how to make a chassis:

```c++
dom::MoveConfig move_config(1.0,        // exit (inches)
                            85,         // speed (%)
                            {1, 1, 1},  // linear pid gains
                            {1, 1, 1}); // angular pid gains

dom::TurnConfig turn_config(2.0,        // exit (degrees)
                            50,         // speed (%)
                            {1, 1, 1}); // angular pid gains

dom::Options default_options = {.timeout = 5000, // ms
                                .accel = 50};    // %/s

dom::Chassis bot({1, 2, 3, 4, 5},       // left motors
                 {-6, -7, -8, -9, -10}, // right motors
                 odom,                  // odom
                 move_config,           // move configuration
                 turn_config,           // turn configuration
                 default_options);      // default options
```
- Left and Right motors are provided in a `list`. Negative reverses the motor direction
- Move and turn configurations set default parameters for those movements
- Default options will be shared by all movements and used if nothing is specified when a movement is called. Options and are described below
### Options
Options contain different parameters for a chassis movement. Options, as the name implies, are optional and can be used to change the default options or configurations. Options are as follows:

- `Direction dir` the direction of the movement. defaults to `AUTO`
    - can be `FORWARD`/`CCW`, `AUTO`, or `REVERSE`/`CW`
- `double exit` the maximum error to be considered at target. defaults to `config.exit`*
- `int timeout` the maximum allowed time for a movement. defaults to `0` or ignore timeout
- `double speed` the maximum speed of a movement. defaults to `config.speed`*
- `double accel` the maximum acceleration of a movement. defaults to `0` or ignore acceleration limits
- `Gains lin_PID` PID gains for linear movement. defaults to `config.lin_PID`*
- `Gains ang_PID` PID gains for angular movement and turns. defaults to `config.ang_PID`*
- `bool async` true for asynchronous movements. defaults to `false`
- `bool thru` true for through movements. defaults to `false`
- `bool relative` true for relative movements. defaults to `false`

_*these options will be ignored in the default_options as the configurations take precedence_

### Movements
Currently, there are 2 different motion commands: `move(Point/Distance, Options)`, and `turn(Point/Angle, Options)`. This makes it very easy to control the chassis. The movement parameters will automatically default to configurations or default options for those not specified. Movements can be done like:

```c++
// move to point (24, 0)
bot.move((24, 0));
// move to point (24, 24) at max speed
bot.move((24, 24), {.speed = 100});
// turn to face (0, 0)
bot.turn((0, 0));
// turn 270 degrees clockwise
bot.turn(270, {.relative = true, .dir = CW});
```

Options also make it very easy to tune specific types of motions and use them throughout your autonomous

```c++
// options for a fast movement
dom::Options fast = {.speed = 100, .accel = 0, .exit = 5, .thru = true};
// options for a precise movement
dom::Options precise = {.speed = 50, .accel = 20, .exit = 0.5, .lin_PID = (5, 0, 0)};

bot.move((50, 0), fast);
bot.move((10, 0), precise);
bot.turn(90, fast);
```

For operator control, tank and arcade controls exist. You can pass in the controller for ease of use, or simply use numbers for custom curves

```c++
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while(true) {
        // arcade controls
        bot.arcade(master);
        // tank controls
        bot.tank(master);
    }
}
```
