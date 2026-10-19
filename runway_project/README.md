# Runway Detection and Tracking for Landing Assistance (MATLAB R2019a + Simulink)

Vision-based landing cue: find the two runway edges in an approach image, track
them with a Kalman filter, and turn them into lateral deviation (m) and yaw error (deg).
Needs MATLAB, Simulink and Stateflow only (Stateflow supplies the MATLAB Function block).
Computer Vision Toolbox is optional (only for the live Video Viewer).

## Run it

1. Put all files in one folder and make it the current folder in MATLAB.
2. `test_runway_standalone` runs the algorithm without Simulink. Do this first.
3. `run_runway_demo` builds `runway_landing_assist.slx`, simulates 20 s and plots results.
   `run_runway_demo(0.4)` runs the same thing in haze.
4. `runway_visibility_sweep` gives the haze robustness table and plot.

## Simulink model

    Clock ---+--> Scene ---> Detector ---> Tracker ---> Guidance
    Visibility                  |             |             |
    NoiseSigma                  +-------------+-------------+--> Logger --> RunLog + 3 Scopes
                  Scene --> Overlay (red = Hough, green = tracked) --> Video Viewer

| Block | File | Job |
|---|---|---|
| Scene | runway_scene.m | Synthetic approach video: runway, dashed centre-line, taxiway clutter, haze, noise. Outputs truth. |
| Detector | runway_detect.m | Sobel gradient, threshold, Hough transform in a left and a right angle window, sub-bin peak refinement. |
| Tracker | runway_tracker.m | 4 constant-velocity Kalman filters (edge position at top and bottom of the ROI), innovation gating, lock flag. |
| Guidance | runway_guidance.m | Lateral deviation from edge geometry, yaw from the vanishing point. |
| Logger / Overlay | runway_logger.m, runway_overlay.m | Logging, scopes, annotated video. |

All tunable numbers (image size, thresholds, Kalman noise, gates) live in `runway_params.m`.
The blocks only call the `.m` files, so you can edit the algorithm without opening Simulink.
To use real footage, replace the Scene block with a From Multimedia File block plus
RGB-to-grey and resize to 120 x 160, and feed the grey image (0..1, double) to the Detector.

## Results (algorithm run in GNU Octave, 20 s, 10 fps, noise sigma 0.03)

| Visibility | Raw detection correct | Tracker locked | Centre error raw | Centre error tracked | Lateral dev. error | Yaw error |
|---|---|---|---|---|---|---|
| 1.00 | 100 % | 99 % | 0.81 px | 0.72 px | 0.37 m | 0.20 deg |
| 0.60 | 100 % | 99 % | 0.83 px | 0.70 px | 0.36 m | 0.19 deg |
| 0.45 | 100 % | 99 % | 0.81 px | 0.55 px | 0.29 m | 0.13 deg |
| 0.35 | 100 % | 99 % | 0.86 px | 0.49 px | 0.26 m | 0.11 deg |
| 0.25 | 37 % | 84 % | 14.0 px | 0.83 px | 0.36 m | 0.14 deg |
| 0.18 | 3 % | 3.5 % | 14.9 px | 295 px | n/a | n/a |

Figures are in `results/`: `tracking_clear_vs_haze.png`, `visibility_sweep.png`, `sample_frames.png`.

How to read this honestly:
- The tracker adds little in clear conditions (0.81 to 0.72 px). Most of that residual is a
  steady bias of about 0.6 px in the measurement, which filtering cannot remove.
- Its value is robustness: at visibility 0.25 only 37 % of raw detections are correct, yet gating
  keeps the track at 0.83 px.
- Below about 0.2 the system fails. The lock flag correctly drops, which is the integrity
  signal a real system would use to declare the cue unavailable.
- Tracker tuning: sigmaMeas = 1.0, sigmaAccel = 8 gave the lowest error over visibility 1.0 to 0.35
  among 16 combinations tried. The original 2.0 / 2.0 lagged (1.23 px).

## Limits of this model (say these in your viva)

- Pinhole camera, flat ground, no roll or pitch, camera axis parallel to the runway except a small yaw wobble.
- The deviation formula assumes that alignment; with real roll and pitch you would fuse IMU data (an EKF is the natural next step).
- The scene is synthetic. Validate on real approach video before claiming real-world performance.

## What is and is not verified

- Verified: all algorithm files ran end to end in GNU Octave and produced the results above.
- Not verified here: `build_runway_model.m`, the Simulink wiring and MATLAB Function block code generation on R2019a,
  because no MATLAB or Simulink was available. If the builder errors, the most likely causes are the Video Viewer
  library path (the script falls back to a Terminator, connect the viewer by hand) or a port-count mismatch on a MATLAB Function block.
  `test_runway_standalone` is unaffected either way.

## Ideas to extend (placement-level extras)

- Add an EKF that fuses the vision cue with IMU roll and pitch.
- Replace the Scene block with real or FlightGear video.
- Generate HDL for the Sobel stage with Vision HDL Toolbox.
- Add a failure monitor that disables the cue when lock is lost for more than 1 s.
