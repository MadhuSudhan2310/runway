# Runway Detection and Tracking for Landing Assistance

Vision-based landing aid built in **MATLAB R2019a + Simulink**. A synthetic approach camera image is processed to find the two runway edges, which are tracked with a Kalman filter and converted into **lateral deviation (m)** and **yaw error (deg)** cues.

## Pipeline

```
Scene -> Detector (Sobel + Hough) -> Tracker (Kalman + gating) -> Guidance -> Logger / Scopes / Video
```

| Block | File | Function |
|---|---|---|
| Scene | `runway_scene.m` | Synthetic approach video with clutter, haze and noise |
| Detector | `runway_detect.m` | Sobel gradient, threshold, Hough transform |
| Tracker | `runway_tracker.m` | Kalman filter per edge point, innovation gating, lock flag |
| Guidance | `runway_guidance.m` | Lateral deviation and yaw from the tracked edges |
| Logger / Overlay | `runway_logger.m`, `runway_overlay.m` | Logging and annotated video |
| Parameters | `runway_params.m` | All constants in one place |

---

## 1. Theory and formulas

### 1.1 Camera model (pinhole)

A ground point $(X, Y, Z)$ projects to the image as

$$u = f\,\frac{X}{Z}, \qquad v = f\,\frac{Y}{Z}$$

The focal length in pixels follows from the horizontal field of view $\mathrm{HFOV}$ and the optical-axis column $c_x$:

$$f = \frac{c_x}{\tan(\mathrm{HFOV}/2)}$$

### 1.2 Runway edges in the image

Runway edges are parallel lines on the ground, so they become straight lines in the image that meet at the **vanishing point** $(x_{vp}, y_h)$, where $y_h$ is the horizon row. For image row $y$, with $H$ the number of image rows:

$$x_{L,R}(y) = x_{vp} + \bigl(x_{L,R}^{b} - x_{vp}\bigr)\,\frac{y - y_h}{H - y_h}$$

Here $x^{b}$ is the edge column at the bottom row. The runway centre and half-width at the bottom row are

$$c_b = \frac{x_L^{b} + x_R^{b}}{2}, \qquad w = \frac{x_R^{b} - x_L^{b}}{2}$$

### 1.3 Sobel edge detection

Image gradients are computed with the Sobel kernels:

$$G_x = \begin{bmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{bmatrix} * I, \qquad G_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{bmatrix} * I$$

$$G = \sqrt{G_x^2 + G_y^2}$$

A pixel is an edge pixel (inside the region of interest below the horizon) if

$$G > T, \qquad T = \max\bigl(\alpha \cdot \max G,\; T_{\min}\bigr)$$

with $\alpha = 0.35$ and $T_{\min} = 0.12$.

### 1.4 Hough transform

Each edge pixel $(x, y)$ votes for all lines through it, written in normal form

$$\rho = x\cos\theta + y\sin\theta$$

Votes are accumulated in a $(\rho, \theta)$ array. The left and right runway edges lean in opposite directions, so two separate angle windows are searched:

$$\theta_L \in [5^\circ, 60^\circ], \qquad \theta_R \in [-60^\circ, -5^\circ]$$

This rejects horizontal clutter such as taxiways, whose normals lie near $90^\circ$. The accumulator is smoothed with a $3\times 3$ box sum, the peak is picked, and its position is refined to sub-bin accuracy with a centroid of the raw votes in the $3 \times 3$ neighbourhood. A line is accepted only if its smoothed peak height exceeds `minVotes` (40).

The detected line is converted to column positions at the bottom row $y_b$ and at the top of the search region $y_t$:

$$x(y) = \frac{\rho - y\sin\theta}{\cos\theta}$$

giving the measurement vector

$$\mathbf{z} = \begin{bmatrix} x_L(y_b) & x_L(y_t) & x_R(y_b) & x_R(y_t) \end{bmatrix}^T$$

A sanity check discards the weaker edge if the detected runway is narrower than 20 px at the bottom row.

### 1.5 Kalman filter tracking

Each of the four channels of $\mathbf{z}$ is tracked by an independent constant-velocity filter with state $\mathbf{s} = [x,\ \dot{x}]^T$ and frame period $\Delta t$.

**Model**

$$\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \qquad \mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}, \qquad \mathbf{Q} = \sigma_a^2 \begin{bmatrix} \Delta t^4/4 & \Delta t^3/2 \\ \Delta t^3/2 & \Delta t^2 \end{bmatrix}, \qquad R = \sigma_m^2$$

**Predict**

$$\hat{\mathbf{s}}^- = \mathbf{F}\hat{\mathbf{s}}, \qquad \mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$$

**Update**

$$\nu = z - \mathbf{H}\hat{\mathbf{s}}^-, \quad S = \mathbf{H}\mathbf{P}^-\mathbf{H}^T + R, \quad \mathbf{K} = \mathbf{P}^-\mathbf{H}^T S^{-1}$$

$$\hat{\mathbf{s}} = \hat{\mathbf{s}}^- + \mathbf{K}\nu, \qquad \mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$$

**Gating.** A runway edge measurement (both of its points) is accepted only if every innovation satisfies

$$|\nu| \le \max\bigl(g_{\min},\ k\sqrt{S}\bigr), \qquad g_{\min} = 12\ \text{px},\ k = 4$$

If a measurement is rejected or missing, the filter coasts on its prediction. The covariance grows during coasting, so the gate widens and the filter can re-acquire the edge.

**Lock flag.** Tracking is declared locked when both edges have at least 3 accepted updates and no more than 5 consecutive rejections.

Tuned values: $\sigma_a = 8\ \text{px/s}^2$, $\sigma_m = 1\ \text{px}$, $\Delta t = 0.1\ \text{s}$.

### 1.6 Guidance cues

**Lateral deviation.** At the bottom image row, the ratio of the pixel offset of the runway centre to the pixel half-width equals the ratio of the metric offset to the runway half-width (similar triangles). With runway width $W_{rw} = 45$ m:

$$d = \frac{c_x - c_b}{w}\cdot\frac{W_{rw}}{2}$$

where $d > 0$ means the aircraft is to the **right** of the centre-line. This assumes the camera axis is aligned with the runway axis.

**Yaw cue.** The two tracked edge lines intersect at the vanishing point. Let $s_L$, $s_R$ be the edge slopes in columns per row going up the image:

$$u = \frac{x_R^b - x_L^b}{s_L - s_R}, \qquad x_{vp} = x_L^b + s_L u, \qquad y_{vp} = H - u$$

$$\psi = \arctan\!\left(\frac{x_{vp} - c_x}{f}\right)$$

**Range cue.** The half-width $w$ grows as the aircraft approaches, so it acts as a crude range indicator.

### 1.7 Performance metrics

$$\mathrm{RMSE} = \sqrt{\frac{1}{N}\sum_{k=1}^{N}\bigl(\hat{c}_{b,k} - c_{b,k}^{\text{true}}\bigr)^2}$$

computed after a 1 s start-up transient. A raw detection counts as *correct* if its centre error is below 5 px and its half-width error is below 8 px.

---

## 2. Results

Run on the synthetic scene: 20 s, 10 fps, sensor noise $\sigma = 0.03$. Visibility is a contrast scale (1.0 = clear).

| Visibility | Raw detection correct | Tracker locked | Centre error raw | Centre error tracked | Lateral dev. error | Yaw error |
|---|---|---|---|---|---|---|
| 1.00 | 100 % | 99 % | 0.81 px | 0.72 px | 0.37 m | 0.20 deg |
| 0.60 | 100 % | 99 % | 0.83 px | 0.70 px | 0.36 m | 0.19 deg |
| 0.45 | 100 % | 99 % | 0.81 px | 0.55 px | 0.29 m | 0.13 deg |
| 0.35 | 100 % | 99 % | 0.86 px | 0.49 px | 0.26 m | 0.11 deg |
| 0.25 | 37 % | 84 % | 14.0 px | 0.83 px | 0.36 m | 0.14 deg |
| 0.18 | 3 % | 3.5 % | 14.9 px | 295 px (diverged) | n/a | n/a |

Findings:
- In clear conditions the Kalman filter gives only a small gain. About 0.6 px of steady measurement bias remains.
- Its main benefit is robustness: at visibility 0.25 only 37 % of raw detections are correct, yet gating keeps the track at 0.83 px.
- Below about 0.2 the system fails. The lock flag drops, which is the integrity signal to declare the cue unavailable.

![frames](results/sample_frames.png)
![sweep](results/visibility_sweep.png)

---

## 3. How to run

1. Put all files in one folder and make it the current folder in MATLAB.
2. `test_runway_standalone` runs the algorithm without Simulink (run this first).
3. `run_runway_demo` builds `runway_landing_assist.slx`, simulates 20 s and plots results. `run_runway_demo(0.4)` runs in haze.
4. `runway_visibility_sweep` produces the haze robustness table and plot.

Requires MATLAB, Simulink and Stateflow. Computer Vision Toolbox is optional (live Video Viewer only).

## 4. Assumptions and limitations

- Pinhole camera, flat ground, no roll or pitch, camera axis parallel to the runway except a small yaw wobble.
- Synthetic images only. Validate on real approach video before claiming real-world performance.
- The algorithm was verified in GNU Octave. The Simulink model builder has not been verified on R2019a.
- Possible extensions: EKF fusion with IMU roll and pitch, real or FlightGear video input, HDL generation for the Sobel stage.
