# Motion Control
This ROS package contains simple yet mandatory scripts to move a differential mobile robot.

## Encoders
The encoders script interfaces with quadrature encoders, calculates distances covered by each wheel and the velocities of each wheel

Distance are published on the topics:
```
rwheel_dist_rob1
lwheel_dist_rob1
```
Velocities are published on the topics:

```
rwheel_spd_rob1
lwheel_spd_rob1
```

## Motor Driver
The motor driver script uses 6 wire, 2 PWM mode to drive two motors through an H-bridge. It subscribes to two power motor topics:
```
rmotor_pwr_rob1
lmotor_pwr_rob1
```
The topics need to have a value from -100,100 as a duty cycle for the motor. The script handles the negative input as a reversing direction

## Odom
The odometry script subscribes to wheel velocities and distances, and then publishes the current position of the robot on:
```
pose_rob1
```

## Velocity PID
The velocity pid script is a simple pid loop. When running, the output is published to the motor power topics, to which the motor driver is suscribed to

## Twist To Motors
The script subscribes to a twist message on the following topic
```
cmd_vel
```
It then transforms the twist message into required wheel velocities by using the base width as w in the beginning of the script
