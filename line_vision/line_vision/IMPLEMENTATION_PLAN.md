# RoboCupJunior Rescue Line 2026 camera-vision plan

This plan is based on the attached *RCJ Rescue Line 2026 draft*, dated 2026-01-28. It focuses on the line and evacuation-zone rules. Check the official forum and the event's local rules for later corrections before competition.

## What the robot must recognize

- A black line 1-2 cm wide on a white floor. Straight-line gaps are at most 20 cm, with at least 5 cm of line before each gap. Tiles are 30 cm square; seams, steps up to 3 mm, ramps, and uneven surfaces can affect the image.
- Intersections are perpendicular, with three or four branches. A green marker is 25 mm square and is placed just before the intersection to signal the intended branch. No marker means continue straight. Two markers, one on either side of the line, mean a dead end and the robot must turn around. Confirm marker-to-turn interpretation against the rule illustrations and event clarifications before fixing it in code.
- Ramps can reach 25 degrees; lines there can also have gaps, intersections, speed bumps, obstacles, and debris. Seesaws have a straight, otherwise unmarked line.
- The evacuation zone is 120 x 90 cm, bounded by walls. Its entrance is marked by a 25 x 250 mm reflective silver floor strip. The line stops at the entrance and resumes at the exit, which has a 25 x 250 mm black strip. The zone contains two silver live victims, one black dead victim, and red and green triangular evacuation points. Victims can be anywhere; fake victims or images may also appear. White wall LEDs are allowed.
- The goal is marked by a 25 x 300 mm red strip perpendicular to the incoming line. Red must therefore be interpreted in context, not treated as an intersection marker or an evacuation point by itself.
- The course is not disclosed in advance. Make decisions from current observations; do not encode a known tile sequence or route.

## Recommended system layout

Use the Pi 5 as the vision and navigation computer, with a downward-facing camera and a motor controller handling fast wheel control. Keep motor PID, encoder sampling, and emergency stop outside the image-processing loop. Send the motor controller a speed and turn request plus a stop/watchdog signal. Use encoders and an IMU if available for controlled turns and motion through brief visual ambiguity; they supplement vision and must not become a pre-mapped route.

The Raspberry Pi Camera Module v1.3 is the OV5647 camera. On a Pi 5, use the correct 15-to-22-pin camera cable and the current libcamera/Picamera2 stack. Start with a fixed camera exposure/white balance after startup calibration, a rigid mount, and a broad enough field of view to see the line near the wheels and several centimetres ahead. Keep the lens protected from floor contact and vibration. Use active cooling for sustained Pi 5 operation.

Start with a 640 x 480 or similarly modest image region at a stable 30 fps. Raise frame rate or resolution only if testing shows a need. Use grayscale for line processing and HSV/color masks only for green, red, and silver cues. This is a small, deterministic OpenCV-style vision pipeline; a neural network is unnecessary for the first implementation and would make timing and field-light robustness harder.

## Per-frame vision pipeline

1. Capture timestamped frames and reject stale frames. Calibrate camera exposure and white balance on representative white floor, black tape, green markers, and silver tape. Use modest blur and thresholding; avoid aggressive auto settings that make black/white classification drift.
2. Apply a floor region of interest. Optionally rectify it with a ground-plane homography once the mount is fixed. Keep a raw-image fallback because ramps change the camera-to-floor geometry.
3. Make a black-line mask from grayscale/brightness, with a small adaptive threshold or locally normalized brightness to tolerate shadows. Remove isolated speckles, but do not erode away the 1 cm line. Reject image regions outside the expected floor area.
4. Divide the near field into horizontal bands. In each, measure line pixels, connected components, centroid, width, and confidence. Fit a line/curve to centroids and estimate lateral error and heading error relative to the robot. Use near-field error to steer and farther-field shape to anticipate turns. If the line is forked, retain all plausible branches instead of averaging them into a false centre.
5. Separately measure green, red, and silver masks in calibrated color space. Require plausible size, shape, position, and temporal consistency. Use image context and state to disambiguate signals: green floor markers near an intersection, red goal stripe versus red evacuation triangle, and silver entrance strip versus silver victim.
6. Publish a compact observation with line error, heading error, candidate branches, marker detections, gap/line-loss confidence, special-strip candidates, and frame age. Log images and decisions during development.

## Navigation and intersection handling

Use a finite-state machine, not one monolithic vision rule:

- **FOLLOW_LINE:** Track the line with a speed-scheduled PD/PID steering command. Slow when curvature, heading error, or uncertainty rises. Use a wider look-ahead at speed and a near-field measurement at low speed.
- **APPROACH_INTERSECTION:** A branch/width increase in the forward ROI or a green marker starts an event. Reduce speed, preserve the robot's incoming heading, and gather several frames before committing. Distinguish a real junction from a wide tape patch, seam, or shadow using connected components and persistence.
- **CHOOSE_BRANCH:** Apply the marker rule: one marker selects the indicated turn, no marker selects straight, and two side markers select a 180-degree reversal. The exact one-marker geometry must match the official diagrams. Stop treating branch appearance as a route map; choose only from the current marker and observed branches.
- **EXECUTE_TURN:** Use a closed-loop pivot/arc with encoder or gyro feedback if available, while checking camera evidence. Reacquire the outgoing line near the expected direction, then return to FOLLOW_LINE. Do not use a fixed timed turn as the sole control.
- **DEAD_END_TURN:** Confirm both side markers and the terminating line pattern, then rotate 180 degrees and reacquire the incoming line. Avoid interpreting a single marker or temporary occlusion as a dead end.
- **GAP_RECOVERY:** When the line disappears, keep the last reliable heading and move forward slowly while searching a bounded ROI for up to the allowed gap distance. A 20 cm gap is the rule maximum, not a good blind-drive target; stop/search or use odometry to cap travel and prevent driving off a raised tile. Resume only after a stable line is detected. If it cannot be reacquired by the next tile, stop and allow the referee's Lack of Progress process rather than wandering.
- **RAMP/SEESAW:** Detect sustained pitch/forward-flow change where possible, reduce speed, and keep the line tracker active. Do not assume line appearance alone means level ground. Use traction-aware acceleration and avoid steering oscillations.
- **GOAL:** Confirm the red transverse goal strip with the line ending and its expected geometry. Stop fully and hold for more than five seconds to qualify for the exit bonus.

For all states, use confidence thresholds and hysteresis: several consistent frames to enter a special state, clear exit conditions to leave it, and a safe stop if camera frames stall. Record an event only once per traversal so the robot does not loop repeatedly on a marker.

## Evacuation-zone behavior

Treat the zone as a separate mission state, entered only after the silver entrance strip is confirmed at the end of the incoming line. Do not follow silver reflections as if they were line. Use the camera to estimate the zone boundary, victims, and evacuation triangles; use wheel odometry for short local moves and continually correct against visual observations.

1. **Enter and localize:** Cross the silver strip slowly. Detect the walls and the red/green 30 x 30 cm corner triangles; build a local top-down or image-relative map. The entrance and exit corners are occupied by the zone openings, so the safe points are in the other corners and can be in either arrangement. Re-localize after movement or a victim rolls.
2. **Find and classify victims:** Detect spherical candidates by size, roundness, and floor contact. Black is the dead victim; reflective silver is a live victim. Silver appearance changes with lighting, so combine color/brightness, specular behavior over multiple views, and shape. Ignore flat images and inconsistent candidates. The rules also state silver victims are conductive: a small contact conductivity sensor is a useful independent confirmation if practical, but vision remains the search and approach method. White wall LEDs can create bright false detections; reject candidates attached to the wall or without spherical geometry.
3. **Prioritize:** Rescue both live victims first, then the dead victim. Carry each gently and release it completely inside the matching triangle: live to green, dead to red. Confirm the victim is wholly within the triangle and no longer touching the robot before leaving. This sequence is required for the dead-victim multiplier.
4. **Manipulate and verify:** Add a low-force capture/transport mechanism with enough retention for the victims' off-centre mass. Recheck camera view after pickup and placement; a sphere can roll after release. Make placement actions slow and verify the victim remains in the safe point without robot contact.
5. **Exit:** Find the black exit strip and reacquire the line on the far side. Do not mistake the zone boundary or a victim for the outgoing path. Resume line-following until the goal strip.

The zone rules allow Lack of Progress to leave victims where they are or reposition held victims. Keep state based on fresh observations after any referee restart; do not assume a carried victim or a previously observed victim is still in the same place.

## Development sequence and acceptance tests

1. **Camera and logging:** Verify OV5647 capture on the Pi 5, frame rate, latency, exposure stability, focus, and temperature. Record representative images under bright, dim, warm, cool, and flash-like lighting.
2. **Line tracking:** Get stable following on straight, curved, narrow (1 cm), and 2 cm tape on white surfaces. Test tile seams, texture, shadows, and up-to-3 mm steps. Measure steering latency and line-loss rate.
3. **Events:** Test every 3/4-way junction, each marker direction, no-marker straight, double-marker reversal, and lookalikes. Test gaps from short to 20 cm with varying approach angles. Add ramps, seesaw, speed bumps, and obstacle detours without assuming their locations.
4. **Rescue zone:** Test both arrangements of red/green triangles, random victim positions, rolling victims, false/image victims, wall LEDs, entrance/exit strips, and bright reflections. Validate pickup, correct order, release verification, and exit-line reacquisition.
5. **End-to-end runs:** Run randomized courses under the full 8-minute clock. Track completion time, false junctions, missed markers, gap recovery, successful rescues, camera latency, and temperature. Prefer conservative speed that reliably reaches checkpoints; score reductions apply to later attempts.

Keep saved debug overlays for masks, line fit, branch candidates, marker/victim detections, state, and confidence. Add a physical camera view/diagnostics mode for bench testing, but do not depend on a laptop or external field information during a run.

## Rule-driven priorities

The robot must autonomously recognize an unknown course, and the course can change during a round. Prioritize safe line reacquisition and correct intersections over speed. Losing the line without reacquiring it by the next tile is a Lack of Progress; an incorrect-sequence line is also a Lack of Progress. Hazards score when the robot reaches the next tile, so reliable completion matters. Gaps, intersections/dead ends, ramps, speed bumps, obstacles, and seesaws can score once per intended direction; evacuation-zone hazards do not score. The 8-minute limit includes calibration and the final stop. The draft's ramp scoring is inconsistent: Section 3.7 says each individual ramp tile, while Section 5.6 says per ramp. Ask the event/referee which interpretation applies.

This plan covers the line and rescue-zone behavior. Before implementation, check the official intersection illustrations and any current forum clarification, and confirm the competition's local rule version.
