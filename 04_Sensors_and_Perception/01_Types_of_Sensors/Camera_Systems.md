---
title: Camera Systems
description: Imaging sensors that record incident light onto a 2D pixel array, producing the raw input for nearly all visual perception. The pinhole model, intrinsic and extrinsic calibration, distortion, stereo, RGB-D, and event cameras — what every roboticist needs to know about cameras as measuring instruments.
tags:
  - sensors
  - camera
  - vision
  - perception
  - calibration
  - stereo
  - rgb-d
type: Sensor
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /camera_systems/
related:
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[Computer_Vision]]"
  - "[[RGB-D_Sensor]]"
  - "[[LIDAR]]"
  - "[[Sensor_Calibration_Techniques]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM]]"
  - "[[IMU_Sensors]]"
---

# Camera Systems

A **camera** is a sensor that records the intensity of light arriving from each direction in front of it onto a 2D pixel array. In robotics it is the cheapest, densest, and most semantically rich exteroceptive sensor available — and also the trickiest one to use as a *measuring instrument*, because the mapping from a 3D world point to a 2D pixel involves a calibrated geometric model that has to be respected at every step.

> **Etymology.** *Camera* — Latin for "vault, chamber, room." The full original term was *camera obscura* ("dark room"), the room-sized predecessor of the photographic camera, in which a small aperture in one wall projected an inverted image of the outside scene onto the opposite wall. Documented use of the camera obscura goes back to Mozi (墨子) in 4th-century-BC China and Ibn al-Haytham's *Book of Optics* (~1021 AD). The modern handheld camera is just a miniaturized chamber with a lens to focus the light and a sensor where the wall used to be.

---

## The pinhole camera model

The geometric foundation of every camera in robotics is the **pinhole model**: an idealization in which all light rays pass through a single point (the optical center) and project onto an image plane behind it.

A 3D point $\mathbf{P} = (X, Y, Z)$ in the camera frame projects to a pixel $(u, v)$ via:

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} \sim K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}, \qquad
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

where:

- $f_x, f_y$ are the **focal lengths** in pixel units (one per axis because pixels may not be square).
- $c_x, c_y$ are the **principal point** — the pixel coordinates of the optical center.
- $\sim$ means "equal up to scale" — the right-hand side is a homogeneous coordinate that needs to be divided by its $Z$-component.

$K$ is the **intrinsic matrix**. It is a property of the camera itself (sensor + lens) and does not change as the camera moves.

For a 3D point $\mathbf{P}_w$ in *world* coordinates, you first transform to the camera frame using the **extrinsic** $[R \mid \mathbf{t}]$:

$$
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix} = R \mathbf{P}_w + \mathbf{t}
$$

The full projection is then $u, v = \pi(K [R \mid \mathbf{t}] \mathbf{P}_w)$. Intrinsics describe *the camera*; extrinsics describe *where the camera is*. Calibration recovers both.

---

## Distortion — what real lenses actually do

Real lenses bend light slightly differently from the pinhole ideal. The two dominant distortions:

- **Radial distortion** — straight lines bow outward (barrel) or inward (pincushion). Modeled as a polynomial $r' = r (1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$, where $r$ is the distance from the principal point.
- **Tangential distortion** — caused by the lens not being perfectly parallel to the sensor. Two parameters $p_1, p_2$.

Five distortion coefficients $(k_1, k_2, p_1, p_2, k_3)$ ("OpenCV model") plus the four intrinsic parameters $(f_x, f_y, c_x, c_y)$ make up the **9-parameter calibration** used by virtually every robotics codebase. Wide-angle and fisheye lenses use richer models (Kannala-Brandt, equidistant) with up to 8 distortion coefficients.

**Calibration removes distortion** — undistorting the image gives you the pinhole-model approximation that all the math above assumes. Skip this step and your 3D measurements will be wrong, with errors growing toward the edges of the image. See [[Sensor_Calibration_Techniques]] and Zhang's 2000 chessboard method, which is what you're using when you run `cv2.calibrateCamera` or the ROS `camera_calibration` GUI.

---

## Sensor technology — CCD vs CMOS, rolling vs global shutter

A camera's silicon sensor turns photons into electrons into a digital number. Two big technology partitions matter for robotics:

| Partition | Options | Why it matters |
|---|---|---|
| **Photodiode tech** | CCD (charge-coupled device) vs CMOS | CCD: lower noise, slower readout, more expensive. CMOS: dominant today (90%+ of consumer + robotics). |
| **Shutter** | Global (whole frame at once) vs rolling (one row at a time) | Rolling shutter creates motion warping ("Jell-O effect") on fast-moving cameras and ruins SLAM/VIO geometry |

For any robot doing visual SLAM or visual-inertial odometry, **buy a global-shutter camera**. The rolling-shutter math exists and works, but the constant-headache margin is not worth saving $50.

A few more sensor choices that show up in datasheets:

- **Bit depth** — 8 / 10 / 12 / 16 bits per pixel. More bits = larger dynamic range, important for outdoor scenes that mix shadow and sunlight.
- **Pixel size** — 2–6 µm typical. Bigger pixels collect more light per exposure → less noise in low light.
- **Quantum efficiency** — fraction of photons that become measurable electrons. 60–90% for backside-illuminated sensors.
- **Read noise** — electrons of noise per pixel per readout. 1–5 e⁻ for good sensors, dominates in low light.

---

## The robotics camera zoo

| Type | What it adds | Examples |
|---|---|---|
| **Monocular RGB** | Standard color camera | Logitech, FLIR, Basler ace |
| **Global-shutter monocular** | No rolling-shutter artifacts | FLIR Blackfly, Basler ace, Allied Vision |
| **Stereo pair** | Two cameras → depth via disparity | ZED, Bumblebee, OAK-D |
| **RGB-D ([[RGB-D_Sensor]])** | RGB + per-pixel depth | RealSense D435/D455, Azure Kinect, Orbbec |
| **Wide-angle / fisheye** | 180°+ FoV | GoPro, Entaniya, T265 |
| **Omnidirectional / 360°** | Full-sphere FoV | Insta360, Ricoh Theta |
| **Thermal / IR** | Heat instead of visible light | FLIR Lepton, Boson |
| **Multispectral / hyperspectral** | Many wavelength bands | Agriculture, mining |
| **Event cameras** | Per-pixel asynchronous brightness *changes* | Prophesee, iniVation DAVIS |
| **Plenoptic / light-field** | Angular + spatial info per pixel | Lytro (deprecated), Raytrix |

Event cameras (Lichtsteiner et al. 2008) deserve a special note: instead of frames at fixed rate, they output a stream of events `(x, y, t, polarity)` whenever any individual pixel sees a brightness change. Latency is sub-microsecond, dynamic range is 120+ dB, and they handle rapid motion gracefully — but standard CV algorithms don't apply, and they remain a research-grade tool in 2025.

---

## Stereo geometry — depth from two cameras

Two cameras separated by a known **baseline** $b$ see the same scene from slightly different angles. A point in 3D projects to slightly different image coordinates in the two cameras — the **disparity** $d$. Depth follows from:

$$
Z = \frac{f \cdot b}{d}
$$

where $f$ is the focal length (in pixels), $b$ is the baseline (in metres), $d$ is the disparity (in pixels), $Z$ is the depth (in metres).

So:

- A larger baseline gives more depth precision but a smaller overlap region (shorter minimum range).
- Depth precision degrades quadratically with distance: $\sigma_Z \propto Z^2 / (f b)$.
- A 12 cm baseline at 700 px focal length and 0.5 px disparity precision gives $\sigma_Z \approx 0.06$ m at 3 m and $\sigma_Z \approx 0.6$ m at 10 m.

That last fact is why stereo-only depth perception works well at desk distances and starts to fail at car-park distances. LIDAR doesn't have this scaling issue.

The hard problem is the **stereo correspondence problem** — finding the same scene point in both images. Block matching, semi-global matching (SGM), and learned methods (RAFT-Stereo, HITNet) all attack this; modern stereo cameras like the OAK-D ship the matching pre-computed on an internal DSP/NPU.

---

## Camera + robot — the geometric stack

Every visual sensor on a robot lives inside a chain of transforms:

```
world → robot base → camera mount → camera optical center → image plane → pixel
                    [extrinsic]   [URDF]               [intrinsic K]
```

Get any link wrong and downstream numbers are wrong. Standard practice:

1. **Intrinsic calibration** — chessboard or AprilTag board, OpenCV / Kalibr / ROS `camera_calibration`.
2. **Hand-eye / camera-to-robot calibration** — `tsai`, Park-Martin, or ROS `easy_handeye` packages. Recovers the rigid transform from camera to robot base or end-effector.
3. **Camera-IMU calibration** (for VIO) — Kalibr's IMU-camera tool. Recovers rotation, translation, and time offset between camera frames and IMU samples.
4. **Time synchronization** — hardware sync via PTP/IEEE-1588, or trigger lines that strobe both sensors.

Skip any of these and your visual SLAM will produce poses that drift in unexpected directions. The work is tedious but it is *the* difference between perception code that works in simulation and perception code that works on the robot.

---

## Worked example — pixel to 3D ray

A pixel $(u, v) = (640, 360)$ on a 1280×720 image with calibration $f_x = f_y = 800, c_x = 640, c_y = 360$ corresponds to:

$$
\mathbf{r}_{\text{cam}} = K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix} (u - c_x)/f_x \\ (v - c_y)/f_y \\ 1 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

i.e. the pixel at the principal point looks straight along the optical axis. The 3D point itself is *not* recoverable from a single pixel — only the *direction*. To get $(X, Y, Z)$ you need an additional constraint: a depth measurement (RGB-D or LIDAR), a second view (stereo or motion), or a known scene model (planar floor, known object size).

This single fact — a monocular pixel gives you a ray, not a point — is why stereo, depth sensors, and visual-inertial fusion exist.

---

## Tooling

| Tool | Use |
|---|---|
| **OpenCV** | The default CV library. Calibration, feature detection, stereo, SfM, etc. |
| **Kalibr** | The gold standard for camera + IMU + multi-camera calibration |
| **ROS 2 `image_pipeline`** | Standard ROS image topics, rectification, undistortion |
| **ROS 2 `camera_calibration`** | Interactive intrinsic + stereo calibration GUI |
| **`vision_msgs`** | Standardized perception output messages (detections, poses) |
| **GStreamer / libcamera** | Lower-level video pipeline plumbing on Linux / RPi |
| **ZED SDK / RealSense SDK / OAK API** | Vendor SDKs for stereo + RGB-D cameras |
| **Foxglove Studio / RViz** | Image visualization with overlays |

---

## Recommended reading

- Hartley & Zisserman, *Multiple View Geometry in Computer Vision* (2nd ed., 2003) — *the* reference; pinhole, stereo, fundamental and essential matrices
- Szeliski, *Computer Vision: Algorithms and Applications* (2nd ed., 2022) — broad and readable (free PDF)
- Zhang (2000), *A Flexible New Technique for Camera Calibration* — the chessboard-calibration paper everyone implements
- Scaramuzza & Fraundorfer (2011), *Visual Odometry: Tutorial Part I/II* — clean introduction to camera-as-sensor for SLAM
- Forsyth & Ponce, *Computer Vision: A Modern Approach* (2nd ed., 2011) — alternative to Szeliski

---

## Dataview

```dataview
LIST FROM #camera OR #sensors WHERE contains(file.outlinks, [[Camera_Systems]])
```
