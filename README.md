# appa
appa is a simple and intuitive VEX chassis movement library for [PROS](https://pros.cs.purdue.edu/). 

The intent of this library it to make chassis movement both intuitive to use and understand. Only recommended configurations are supported, but it is designed to be easy to build upon for custom implementations. 

# Installation

1. Download the most recent version with `pros c add-depot appa https://odom.tech/appa.json` (once released)

2. Apply the library to your project with `pros c apply appa`

3. Add `#include "appa/appa.h"` in your `main.h`


# Usage
This library is comprised of two main components: odometry for position tracking and a chassis for robot movement.

## Odometry
[Odometry](https://wiki.purduesigbots.com/software/odometry) is used for keeping track of the robots position at all times. The supported configuration is for two tracker wheels, one in the x (forward) and one in the y (left) direction, in combination with an IMU. Here is how to create an instance of odom:
```c++
appa::Odom odom({11, 1}, // x tracker port
                {11, 3}, // y tracker port
                12,      // imu port
                300,     // tracker encoder ticks per inch (tpi)
                {5, 0},  // tracker linear offset (inches)
                45);     // tracker angular offset (degrees)
```

- The tracker ports can be `port` or `{expander port, port}`
- TPI should be experimentally determined by moving the robot a known distance and recording the encoder output `ticks / distance`
- The linear offset is `{x, y}` which is the distance from your tracking center to your center of mass
- The angular offset can be used for [angled tracker wheel](https://youtu.be/TqMNuXfKgMc?si=iwc8nQkSW-A0ZFeG&t=36) configurations, as long as the two wheels are perpendicular

To start odometry, simply call `odom.start()`, usually during initialization.

```c++
void initialize() {
    odom.start();
}
```

Odometry will do its work in the background after you start it, and should be passed into a chassis to use it. Here are some useful commands:

```c++
// set the robot's pose
odom.set(24, 12, 90);
// get the robot's pose
odom.get();
// set the tracking offset (if COG changes)
// note: this doesn't change tracking, just the .get() function (used in movements)
odom.set_offset({5, 0});
```

## Chassis
The chassis is whats used to actually control the robot. Only differential drive robots are supported in this library. Most holonomic drives will work fine, but won't take advantage of its capabilities. The chassis contains configurations for the different movement types as well as optional options. Information on tuning a PID can be found [here](https://wiki.purduesigbots.com/software/control-algorithms/pid-controller). Here is how to make a chassis:

```c++
appa::MoveConfig move_config(1.0,        // exit (inches)
                             85,         // speed (%)
                             {1, 1, 1},  // linear pid gains
                             {1, 1, 1}); // angular pid gains

appa::TurnConfig turn_config(2.0,        // exit (degrees)
                             50,         // speed (%)
                             {1, 1, 1}); // angular pid gains

appa::Options default_options = {.timeout = 5000, // ms
                                 .accel = 50};    // %/s

appa::Chassis bot({1, 2, 3, 4, 5},       // left motors
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
Options contain different parameters for a chassis movement. The purpose of options are to make it very easy and readable to modify any chassis movement for highly customizable autonomous programs. They are, as the name implies, optional and can be used to change the default options and configurations.

They can be set by simply putting the variable name and value in brackets like `{.speed = 100}`. You can set as many options as you want separated by commas, but they must be in the same order as the list below. Options can also easily be combined using the `<<` and `>>` operators. For example, `opts1 << opts2` will override options in `opts1` with any set in `opts2`. If `opts2` is empty, the the resulting options will be equal to `opts1`. Options are as follows:

| Option | Units | Default | Description |
| - | - | - | - |
| `Direction dir` | `AUTO`, `FORWARD`, or `REVERSE` | `AUTO` for move, `FORWARD` for turns and follow | the direction of the movement |
| `Direction turn` | `AUTO`, `CCW`, `CW` | `AUTO` | the rotating direction of a turn |
| `double exit` | inches for movements, degrees for turns | `config.exit` | the maximum error to be considered at target |
| `int timeout` | milliseconds | `0` or ignore timeout | the maximum allowed time for a movement |
| `double speed` | % of max voltage | `config.speed` | the maximum speed of a movement |
| `double accel` | speed (%) per second | `0` or ignore acceleration limits | the maximum acceleration of a movement |
| `double lead` | decimal % of distance to target | `config.lead` | the lead percentage for boomerang movements |
| `double lookahead` | inches | `config.lookahead` | the lookahead distance for pure pursuit movements |
| `Gains lin_PID` | | `config.lin_PID` | PID gains for linear movement |
| `Gains ang_PID` | | `config.ang_PID` | PID gains for angular movement and turns |
| `bool async` | | `false` | true for asynchronous movements |
| `bool thru` | | `false` | true for through movements |
| `bool relative` | | `false` | true for relative movements |

>Options that default to configurations will ignore those set in the default_options in the constructor. Any other options that were set with default_options will override the defaults above. Options that don't apply to a movement will be ignored.

### Movements
Currently, there are 3 different motion commands: `move(target, options, override)`, `turn(target, options, override)`, and `follow(path, options, override)`. This makes it very easy to control the chassis. `move` targets can be a single number for a relative straight movement, a point to drive to, or a target pose which uses the boomerang controller. `turn` targets can be a single number for a target heading, or a point to face towards. Options will be set as `options << override` for the purpose of allowing the user to use a set of predefined options, and also manually set others for a specific movement. The movement parameters will automatically default to configurations or default options for those not specified. Movements can be done like:

```c++
std::vector<Point> path1 = {{0, 0}, {24, 0}, {24, 24}, {0, 24}}; // path with 4 points

bot.move(-10);                                     // move backwards 10 inches
bot.move({24, 0, 90});                             // move to pose {24, 0, 90}
bot.move({24, 24}, {.speed = 100, .async = true}); // move to point {24, 24} asynchronously at max speed
bot.turn({0, 0}, {.dir = REVERSE});                // turn so the rear faces {0, 0}
bot.turn(180, {.turn = CCW, .relative = true});    // turn 180 degrees CCW
bot.follow(path1, {.lookahead = 6});               // follow path1 with a lookahead distance of 6in
```

Options also make it very easy to tune specific types of motions and use them throughout your autonomous

```c++
// preset and tuned options
appa::Options thru = {.exit = 4, .thru = true};
appa::Options fast = {.speed = 100, .accel = 0};
appa::Options precise = {.speed = 50, .accel = 20, .lin_PID = appa::Gains{5, 0, 1}};

bot.move({24, 12});                            // move with default options
bot.move({50, 0}, fast << thru);               // move with thru and fast options
bot.move({10, 0, 90}, precise, {.lead = 0.7}); // move with precise options plus different lead
bot.turn(90, fast);                            // turn with fast options
```

For operator control, tank and arcade controls exist. You can pass in the controller for ease of use, or simply use numbers for custom curves

```c++
void opcontrol() {
    pros::Controller master(CONTROLLER_MASTER);

    while(true) {
        bot.arcade(master); // arcade controls
        bot.tank(master);   // tank controls
    }
}
```
Please go through the header file `include/appa/appa.h` to see all available functions. Other useful chassis commands include:
```c++
bot.wait();                           // wait for async movement to stop
bot.set_brake_mode(MOTOR_BRAKE_HOLD); // set the brake mode
bot.stop();                           // stop moving
```