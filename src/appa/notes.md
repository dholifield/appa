# To Do
- [ ] maybe make path struct. calculate headings in constructor and store path length
- [ ] maybe add radius to turn so you can do arc movements
- [ ] add new op control setting for fancy curves and scaling and deadzone

# Testing
- [ ] back to back movements
- [ ] test all motions
    - [ ] move distance
    - [ ] move to point
    - [ ] move to pose
    - [ ] turn to heading
    - [ ] turn to point
    - [ ] follow path
 test all options
    - [ ] dir
        - [ ] move point
        - [ ] move pose
        - [ ] turn heading
        - [ ] turn point
        - [ ] negative distance for move
    - [ ] turn
        - [ ] turn CW
        - [ ] turn CCW
    - [ ] thru
        - [ ] move point
        - [ ] move pose
        - [ ] turn any
    - [ ] relative
        - [ ] move point
        - [ ] move pose
        - [ ] turn heading
        - [ ] turn point
    - [ ] async
        - [ ] async then wait
        - [ ] async then cancel with new movement
        - [ ] async then cancel with stop
    - [ ] speed
        - [ ] any
    - [ ] accel
        - [ ] any
    - [ ] lin_PID
        - [ ] move (any)
    - [ ] ang_PID
        - [ ] move (any)
        - [ ] turn (any)
    - [ ] lead
        - [ ] move pose
    - [ ] lookahead
        - [ ] follow
    - [ ] offset
        - [ ] move point
        - [ ] move pose
    - [ ] lin_exit
        - [ ] move (any)
    - [ ] ang_exit
        - [ ] move pose
        - [ ] turn (any)
    - [ ] ang_dz
        - [ ] move point
        - [ ] move pose
    - [ ] exit_speed
        - [ ] any
    - [ ] settle
        - [ ] move point
        - [ ] move pose
        - [ ] turn (any)
    - [ ] timeout
        - [ ] any
    - [ ] exit_fn
        - [ ] move (any)
        - [ ] turn (any)
        - [ ] follow

# Port to FreeRTOS

## Chassis
- motor groups
    - tank/arcade
    - brake mode
- task
- mutex
- millis, task_delay_until

## Odom
- task
- mutex
- custom trackers
- millis, task_delay_until

## Utils
- remove imu and encoder

