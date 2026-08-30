# Project context

Last updated: 2026-08-30

## Purpose

Prepare a simulation-first software stack for the GIST-ISAAC-Robotics entry in the 2026 IT ARENA autonomous-driving competition, approximately three months away. Hardware is still being designed, so geometry and sensor placement in `src/arena_description/config/vehicle.yaml` must remain configurable while software experiments begin now.

## Confirmed team and event constraints

- The reported vehicle footprint limit is 20 cm x 15 cm. No height rule has yet been supplied.
- Roughly five cars are expected to start together after a qualifying-based grid order.
- The organizer is expected to provide a battery, an as-yet-unidentified power-related board, a Jetson Orin Nano, and a steering servo.
- BLDC propulsion is permitted. An ESP32-class MCU and driven-wheel encoders are now the provisional baseline, but the exact ESC, motor, gearing, encoder, transport, and power architecture are not confirmed.
- The team owns a RealSense D435i and intends to use it by default.
- A rotating 2D LiDAR and optional ToF sensors are candidates, mainly for surrounding-vehicle safety.
- The vehicle must observe the start traffic light with a camera.
- ArUco markers identify course regions / branch opportunities. Route choice must be supported.
- Overtaking is not a first milestone; reliable slowing, stopping, and collision avoidance are.

## Machine state observed on 2026-08-30

- Windows host GPU: NVIDIA GeForce RTX 5070 Laptop GPU, driver 610.74, 8 GB VRAM.
- WSL: Ubuntu 24.04.4 LTS on WSL2, with WSLg and NVIDIA GPU forwarding available.
- ROS 2 and Gazebo were not installed at first inspection. ROS 2 Jazzy Desktop, Gazebo Harmonic / `ros_gz`, and the supporting development packages are now installed and pass `scripts/doctor.sh`.
- Docker was not installed on Windows. It is not required for the first milestone.
- GitHub CLI is authenticated as `leejinh0225`.
- Remote repository: `GIST-ISAAC-Robotics/2026-IT-ARENA` (public and initially empty).

## Architecture decision

- PC simulation: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
- Jetson deployment: preserve JetPack 6.2.1 / Ubuntu 22.04. Do not reflash merely to match the simulator.
- Keep algorithm packages source-compatible with ROS 2 Humble and Jazzy where practical. Confine simulator integration to dedicated packages.
- Vehicle-facing interfaces remain stable across environments; only adapters change.
- Gazebo ground-truth pose may be used for controller validation, but competition autonomy must never depend on it.

See `docs/decisions/0001-simulation-platform.md` for rationale.

The provisional real-vehicle boundary assigns perception/planning to the Jetson and actuator timing, command timeout, ESC/servo output, and encoder acquisition to an ESP32. See `docs/decisions/0002-jetson-esp32-actuation-boundary.md`.

## Track source

- Original: `assets/track/original/it_arena_track_final.zip`
- SHA-256: `DE448BA10C614E0F635D44B2F36BAB29EBF455C323DE442562DD01A8296758E4`
- Extracted copy: `assets/track/source/it_arena_track/`
- Actual generated output and the included README disagree. Always read `docs/track/TRACK_AUDIT.md` first.

## Planned software boundary

```text
Gazebo world and simulated sensors
              |
              v
simulation adapters -> stable ROS topics <- real hardware adapters
                              |
                              v
 perception -> localization -> route/race state -> planning -> control
                              |
                              v
                          /drive command
```

Gazebo wheel-joint ground truth is exposed only as `/sim/joint_states_raw`. The simulated encoder adapter publishes quantized `/wheel_states` and `/wheel_encoder_ticks`, matching the planned ESP32-facing feedback contract.

Traffic-light and ArUco algorithms must consume rendered RGB images. Direct simulator state is allowed only in test assertions and visualization.

## Verified implementation baseline

- The preserved organizer output is converted into a strict-SDF runtime world without modifying the source archive. Repetitive static geometry is merged from 1,090 links to 10 so startup is practical in WSL.
- A configurable 0.18 m x 0.12 m provisional Ackermann vehicle spawns in any of six supplied grid slots.
- `/drive` commands are adapted to Gazebo with speed/steering limits and a 0.5 s command-loss watchdog.
- `/odom`, `/tf`, D435i-like RGB, depth, point cloud, and IMU outputs are bridged to ROS 2.
- Ideal Gazebo joint data stays under `/sim/joint_states_raw`. Configurable rear-wheel encoder feedback is published as `/wheel_states` and `/wheel_encoder_ticks`.
- Headless and WSLg GUI launches both succeed and shut down without leaving Gazebo / ROS child processes.
- Environment doctor, strict SDF validation, track provenance validation, four Python interface tests, straight/turn motion, camera/depth, IMU, and encoder data have been exercised successfully.

## Milestones

1. Reproducible environment check and dependency setup.
2. Load the supplied track headlessly and in the Gazebo GUI.
3. Spawn a parameterized Ackermann car and verify manual forward/steering control.
4. Follow the supplied centerline using ground-truth pose.
5. Add curvature-based speed control and complete one solo lap.
6. Add camera/IMU, then 2D LiDAR and optional ToF models behind stable interfaces.
7. Render and recognize the traffic light and ArUco markers from camera pixels.
8. Replace ground truth with realistic localization inputs.
9. Add another vehicle, safe following/stopping, and controlled avoidance.
10. Run multi-car, noise, latency, dropout, and parameter-sweep tests.
11. Bring the same autonomy nodes onto the Jetson through real sensor / actuator adapters.

## Current blockers and risks

- Final vehicle wheelbase, track width, wheel radius, steering range, mass, CG, motor/ESC response, and sensor poses are unknown.
- The actual 12 cm-wide branches are narrower than the maximum permitted 15 cm vehicle width and leave no tolerance for many plausible chassis designs.
- The generated track's general minimum-radius check is false (`0.2987 m` versus the README's older `0.45 m` claim).
- Track README and `output_final` disagree on lap length, wall height, marker set, branch geometry, and source-of-truth status.
- The supplied traffic-light visual has all three lamps emissive and is not animated by the UDP script.
- Camera visibility of the lamp at approximately 1.08 m height must be tested from every grid slot before fixing the D435i bracket.
- A single 2D LiDAR scan plane can miss lower opponent bodywork. Final coverage requires opponent-height evidence and simulated occlusion tests.
- The first simulated D435i uses a combined RGB-D camera with RGB-like horizontal FOV. It is an interface and visibility baseline, not yet a faithful model of the separate RGB/depth intrinsics, stereo holes, lighting sensitivity, or D435i IMU noise.

## Next actions

1. Ask the organizer / hardware team the questions in `docs/HARDWARE_AND_RULE_QUESTIONS.md`; branch width and supplied power hardware are the most urgent.
2. Implement ground-truth centerline following and curvature-based speed control, then record a reproducible solo lap.
3. Add configurable 2D LiDAR and simple opponent models for sensor-height / occlusion experiments before purchasing sensors.
4. Add rendered traffic-light state control and camera-only traffic-light / ArUco perception tests.
5. Select the ESC, encoder, exact ESP32 board, and Jetson transport; then implement the real adapter behind the existing topic contract.
