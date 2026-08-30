# Decision 0002: Jetson / ESP32 actuation and encoder boundary

- Status: provisional baseline
- Date: 2026-08-30

## Decision

Use the Jetson for perception, race-state logic, planning, and generation of a desired vehicle speed and steering angle. Use an ESP32-class MCU for the time-sensitive actuator and safety layer:

```text
Jetson (/drive: desired speed + steering angle)
              |
              v
     ESP32 command watchdog
       |                  |
       v                  v
 ESC command output   steering-servo PWM
       |
       v
   ESC -> BLDC motor

left/right driven-wheel encoders -> ESP32 speed estimate / control
                                      |
                                      v
                     Jetson (/wheel_states + health)
```

The ESP32 does not directly commutate the final BLDC in the baseline design. A compatible ESC performs motor commutation; the ESP32 sends the ESC's required command signal. Direct field-oriented control is a different hardware/software project and is not assumed.

Fit one quadrature encoder to each driven rear wheel if packaging permits. A single motor-shaft encoder can regulate motor speed but cannot observe left/right wheel difference, wheel slip after the drivetrain, or a mechanically disconnected wheel. If encoders are mounted before a gearbox, the configured counts per wheel revolution must include the gear ratio.

Keep the cross-computer ROS contract independent of the selected transport:

- `/drive` (`ackermann_msgs/AckermannDriveStamped`): desired longitudinal speed and steering angle;
- `/wheel_states` (`sensor_msgs/JointState`): left/right driven-wheel angle and angular velocity;
- `/wheel_encoder_ticks` (`std_msgs/Int64MultiArray`): cumulative `[rear_left, rear_right]` counts for bring-up and diagnostics.

USB serial, CAN, and micro-ROS transport remain open choices. The final link must include sequence/age checking, an ESP32-side timeout that commands a safe stop, explicit arming state, and an independently reachable emergency stop.

## Why an encoder is in the baseline

Open-loop ESC throttle is sufficient to make the car move, but commanded throttle is not vehicle speed. Battery voltage, tire load, surface friction, motor temperature, gearing, and collisions change the achieved speed. Encoder feedback materially improves repeatable low-speed motion, stopping distance, launch consistency, and fault detection. It does not replace IMU or visual/lidar localization because wheel slip remains possible.

## Simulation treatment

Gazebo already calculates ideal wheel-joint angle and velocity. Algorithms do not consume that ground truth directly. `sim_wheel_encoder` converts it into the same `/wheel_states` interface intended for the real ESP32 adapter and can model:

- encoder counts per revolution;
- finite sample rate;
- communication / processing latency;
- optional sample dropout.

The first provisional values are 2048 ticks per wheel revolution, 100 Hz sampling, 2 ms latency, and no dropout. They are test parameters, not a purchasing specification.

## Revisit when known

- motor, ESC, command protocol, braking and reverse behavior;
- encoder mounting point, electrical interface, counts per revolution, and maximum edge rate;
- drivetrain gear ratio and differential / spool layout;
- exact ESP32 board and available timers, pulse counters, buses, and isolated power;
- Jetson-to-ESP32 transport and message framing;
- electrical emergency stop and loss-of-command behavior.
