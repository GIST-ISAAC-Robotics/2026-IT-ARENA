# Hardware and rule questions

These answers are not required to start the first simulation, but they progressively replace provisional values in `src/arena_description/config/vehicle.yaml`.

## Ask the organizer first

1. Is 20 cm x 15 cm a maximum outer envelope, and do wheels, bumpers, wiring, and sensors count?
2. Is there truly no height limit, minimum ground clearance, or forbidden sensor overhang?
3. Is the current `output_final` track the competition baseline? Which file/version/date is authoritative?
4. Are both branches really 0.12 m wide? If so, what maximum vehicle width is expected to use them?
5. What are the final wall height, wall thickness, surface material, friction, bump geometry, and tolerances?
6. What are the traffic-light position, lamp diameter, height, color order, sequence, timing, and allowed viewing region?
7. What are the final ArUco dictionary, physical size, IDs, poses, mounting height, and decoy-marker policy?
8. Are deliberate contact, blocking, reversing, lane crossing, and passing restricted? How are collisions penalized?
9. Can a stopped vehicle remain on course, and must competitors safely handle it?
10. What exact battery and power-related board are supplied? The forgotten name may be a PDB, DC-DC/power board, motor driver, or another board; the exact part number matters.
11. What radio / external-compute / preloaded-map restrictions apply during a race?

## Ask the hardware team next

1. Wheelbase: rear axle center to front kingpin/axle center.
2. Front and rear track widths.
3. Overall body length and width, including wheels and protection.
4. Wheel diameter, width, tire material, and expected deformation.
5. Steering linkage type, neutral PWM, end-stop PWM, maximum inner/outer wheel angle, backlash, and full-step response time.
6. Driven axle, motor model, ESC model, gear ratio, differential/spool/free-wheel arrangement, and brake/reverse behavior.
7. Total mass, approximate center of gravity, battery mass/position, and expected payload range.
8. Available regulated rails, current limits, connectors, grounding plan, emergency stop, and Jetson power mode.
9. Exact ESP32 board, logic voltage, available pins/timers, and power source.
10. ESC command protocol (servo PWM, DShot, CAN, UART, or vendor-specific), arming sequence, update rate, telemetry, braking, and reverse behavior.
11. Encoder mounting point (motor shaft, gearbox output, or wheel), channels per driven wheel, quadrature/index support, counts per revolution, gear ratio, maximum edge rate, and voltage level.
12. Jetson-to-ESP32 transport (USB serial, CAN, UART, or Ethernet), connector retention, grounding, message timeout, and restart behavior.
13. Which layer owns the speed loop, steering calibration, command watchdog, arming state, and emergency stop. The provisional design assigns these real-time/safety duties to the ESP32.
14. D435i mount height/pitch and unobstructed field of view.
15. Feasible LiDAR height and whether its 360-degree scan is blocked by the Jetson, battery, mast, wheels, or bodywork.
16. Bumper heights of our car and likely opponents; this determines whether a single scan plane can see them.

## Sensor purchase gate

Do not purchase a LiDAR or an array of ToF sensors solely from intuition. First simulate and measure blind regions using:

- the confirmed chassis envelope;
- at least three opponent-height profiles (low, equal, tall);
- front, side, rear, and close-corner approaches;
- self-occlusion from wheels/body/electronics;
- D435i minimum-range and field-of-view limits;
- required stopping distance at the chosen maximum speed.

The initial likely architecture is D435i RGB/depth + a low or mid-height 2D LiDAR, with ToF only for demonstrated near-field blind spots.
