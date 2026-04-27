---
title: IMU Sensors (Inertial Measurement Unit)
description: A self-contained sensor that measures linear acceleration and angular velocity along three axes, optionally augmented with a magnetometer for magnetic heading. The proprioceptive workhorse of every modern robot, drone, smartphone, and autonomous vehicle.
tags:
  - sensors
  - imu
  - accelerometer
  - gyroscope
  - magnetometer
  - inertial-navigation
  - state-estimation
type: Sensor
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /imu_sensors/
related:
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[Sensor_Fusion]]"
  - "[[Kalman_Filter]]"
  - "[[Localization]]"
  - "[[Sensor_Calibration_Techniques]]"
  - "[[GPS]]"
  - "[[SLAM]]"
---

# IMU Sensors (Inertial Measurement Unit)

An **Inertial Measurement Unit (IMU)** is a small sensor package that reports two things, six times per second to a few thousand times per second: the **linear acceleration** of the unit (3 axes, m/s²) and the **angular velocity** of the unit (3 axes, rad/s). A 9-axis IMU adds a **magnetometer** (3 axes, µT) for absolute magnetic-heading. From those numbers, in principle, one can integrate to recover the unit's full pose — orientation and position — over time.

> **Etymology.** *Inertial* — from Latin *inertia*, "idleness, sluggishness, lack of skill," from *iners* (*in-* "without" + *ars* "art, skill"). Originally meant "lacking skill or ability"; Kepler and then Newton repurposed it as the technical term for "the resistance of a body to change in its state of motion." *Measurement* — Latin *mensura*, from *metiri*, "to measure." *Unit* — from Latin *unus*, "one." Put together: an Inertial Measurement Unit is "a single device that measures inertial effects." The acronym IMU dates to early aerospace engineering in the 1950s; before MEMS, it referred to a fridge-sized assembly of mechanical gyroscopes used for missile guidance.

---

## Why every robot has one

A camera or LIDAR runs at 10–60 Hz, with significant per-frame latency. An IMU runs at 100–1000 Hz with sub-millisecond latency, costs $3 to $300, and works in the dark, in fog, and in featureless environments. Its weakness — it can only measure motion *relative to itself*, never absolute position — is exactly the strength of GPS, vision, and LIDAR. So the dominant pattern is fusion: the IMU provides high-rate, low-latency motion estimates between slow, absolute updates from other sensors.

Without an IMU, a robot can only act on stale, low-rate world measurements. With one, the controller can run at kHz rates while the perception stack catches up.

---

## What's inside

### Accelerometer

Measures **specific force**: linear acceleration *minus* gravity, expressed in the sensor frame. A stationary accelerometer on a flat table reads $(0, 0, +g)$ in m/s² because gravity is pushing the proof mass *down*, which is mechanically indistinguishable from accelerating *up*.

The MEMS implementation is a tiny silicon proof mass on flexures inside a vacuum cavity. Linear acceleration deflects the mass; capacitive electrodes measure the deflection; closed-loop force-feedback restores the mass to center while reporting the required force. Range: typically ±2 g, ±4 g, ±8 g, ±16 g (selectable). Bandwidth: 100 Hz – several kHz.

### Gyroscope

Measures **angular velocity** about each of three axes, in rad/s. MEMS gyroscopes exploit the **Coriolis effect**: a vibrating proof mass deflects perpendicular to the rotation axis when the package rotates. The deflection is proportional to angular rate. Range: ±125°/s up to ±2000°/s. Bandwidth: similar to accelerometer.

Higher-grade gyros (fiber-optic, ring-laser) use the Sagnac effect — light traveling around a closed loop arrives slightly earlier or later depending on the loop's rotation. Used in aerospace and ships.

### Magnetometer (in 9-axis IMUs)

Measures the **local magnetic field vector** in 3 axes. In a clean magnetic environment, this is dominated by the Earth's field (~50 µT, pointing roughly toward magnetic north and downward), letting the sensor estimate absolute heading.

But "clean magnetic environment" rarely exists on a robot — motors, ferrous metal, and high-current cables all distort the field nearby (**hard-iron** and **soft-iron** distortions). Magnetometer calibration on the assembled robot is mandatory and the measurements are still noisy.

---

## What you can compute from an IMU

Given specific force $\mathbf{a}_s$ and angular velocity $\boldsymbol{\omega}_s$ in the sensor frame, and starting from a known initial orientation $R_0$, position $\mathbf{p}_0$, velocity $\mathbf{v}_0$:

$$
R_{k+1} = R_k \exp([\boldsymbol{\omega}_s] \Delta t), \qquad
\mathbf{v}_{k+1} = \mathbf{v}_k + (R_k \mathbf{a}_s + \mathbf{g}) \Delta t, \qquad
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t
$$

This is the **strapdown inertial navigation equations**. In a perfect world, you integrate from $t = 0$ and recover full pose forever. In practice, errors compound:

- **Orientation drift** ∝ $t$ (gyro bias × time).
- **Velocity drift** ∝ $t^2$ (accelerometer bias × $t$, plus cross-coupling from orientation drift).
- **Position drift** ∝ $t^3$.

So an IMU alone is useless for position over more than a few seconds. The whole game in inertial navigation is to inject corrections from external sensors before the cubic blow-up wins.

---

## The error model

For each axis of each sensor, the canonical model is:

$$
z = (1 + S) \cdot x + b(t) + \eta(t)
$$

with:

- **$S$ — scale factor error.** Typically tens of ppm on quality units. Calibrated once.
- **$b(t)$ — bias.** A constant-plus-slow-drift term:
  - *Bias instability* (also called *in-run bias drift*) — random walk during a single run. The hardest error to deal with.
  - *Turn-on-to-turn-on bias* — different value every time you power on.
  - *Temperature bias* — function of temperature.
- **$\eta(t)$ — white noise.** Modeled as Gaussian, characterized by *Velocity Random Walk (VRW)* for accelerometers and *Angular Random Walk (ARW)* for gyros, both in units like (m/s)/√h or °/√h.

The full characterization tool is the **Allan variance plot** — log-log of variance vs averaging time. Different slopes correspond to different noise types: $-1/2$ slope = white noise; $0$ slope = bias instability; $+1/2$ slope = rate random walk.

This is why every state estimator that uses an IMU includes **gyro bias** and often **accelerometer bias** as states to be estimated alongside the pose. The biases drift slowly; the estimator catches them as long as there's some external evidence (GPS, vision, ZUPT) to anchor against.

---

## IMU grades — by far the widest cost-quality range of any robotics sensor

| Grade | Cost | Bias instability (gyro) | Use |
|---|---|---|---|
| **Consumer MEMS** | $1–10 | 100°/h to 10°/s | Smartphones, hobby drones (MPU-6050, BMI270) |
| **Industrial MEMS** | $50–500 | 5–50°/h | Mobile robots, prosumer drones (Bosch BMI088, ADIS16475) |
| **Tactical-grade** | $1k–20k | 0.5–5°/h | Defense, surveying (HG1700, KVH 1750) |
| **Navigation-grade** | $20k–200k | 0.01–0.1°/h | Naval/aerospace inertial navigation (ring-laser, fiber-optic) |
| **Strategic-grade** | $100k+ | <0.01°/h | ICBM guidance (mechanical platform IMUs) |

The 1000× span is real. A $5 MPU-6050 and a $50,000 fiber-optic gyro do *the same job* — they just drift much faster vs much slower. The choice depends on how often you can correct with another sensor. A drone with GPS at 10 Hz tolerates a consumer IMU; an underwater AUV without GPS may need a tactical or even navigation-grade unit.

---

## Calibration — what you actually do before flight

For a typical industrial-grade IMU:

1. **Static six-position test.** Rotate the IMU through 6 orthogonal orientations, hold each for ~30 s. Compute accelerometer biases (each axis sees ±1 g and 0 g) and gyro biases (zero rotation should give zero rate).
2. **Magnetometer figure-8 / wave.** Rotate through all orientations to map the hard-iron and soft-iron distortion ellipsoid, fit a 9-parameter model.
3. **Temperature sweep.** Run the sensor through its operating temperature range and fit bias-vs-temperature curves.
4. **Online estimation.** Most state estimators continuously refine biases as part of the EKF/UKF state. See [[Kalman_Filter]].

Tools: ROS `imu_tools`, Kalibr (for camera + IMU joint calibration), Allan-variance toolbox in Python.

---

## Worked example — gyro drift in 1 second

A consumer MPU-6050 has ~20°/s peak bias and ~0.005°/s/√Hz noise density.

- After 1 second of pure integration: orientation error ≈ 20° (bias) + ~0.5° (noise) = ~20°.
- Removing the calibrated bias (say, residual 0.5°/s): orientation error ≈ 0.5° (residual bias) + 0.5° (noise) = ~1°.
- After 60 seconds with residual 0.5°/s bias: ~30° error → useless for long-term tracking.

Conclusion: even after careful calibration, a consumer IMU drifts by tens of degrees per minute. The fusion partner (GPS, magnetometer, vision) must arrive at least once per second to keep orientation usable. That's why visual-inertial odometry runs at camera rate and inserts the IMU between frames, not the other way around.

---

## Common production IMUs

| Unit | Grade | Notable specs | Where you'll see it |
|---|---|---|---|
| **InvenSense MPU-6050** | Consumer | 6-axis, ±2 g/±250°/s, $3 | Quadcopter teaching projects |
| **InvenSense MPU-9250 / ICM-20948** | Consumer | 9-axis, $5–10 | DIY robotics, Arduino |
| **Bosch BMI088** | Industrial | 6-axis, low noise, robust to vibration | Most autopilot boards (Pixhawk) |
| **Analog Devices ADIS16475** | Industrial | 6-axis, factory-calibrated, SPI | Industrial AGVs, surveying |
| **VectorNav VN-100** | Tactical-ish | 9-axis with fused output, $$$ | Drones, UGVs |
| **Xsens MTi-G-710** | Tactical | INS + GNSS, $$$ | Surveying, AV development |
| **Honeywell HG1700** | Tactical | Ring-laser, $$$$ | Defense, missile, ship |

For most ground-robot work in 2025, an industrial-grade BMI088 or ADIS16475 is the right choice: high enough quality that fusion converges reliably, cheap enough to afford on a research robot.

---

## Tooling

| Tool | Use |
|---|---|
| **ROS 2 `sensor_msgs/Imu`** | Standard IMU message |
| **`imu_tools`** | Filtering, complementary, Madgwick, Mahony |
| **Kalibr** | Camera-IMU joint calibration |
| **`allan_variance_ros`** | Allan-variance characterization |
| **`robot_localization`** | EKF/UKF that fuses IMU with other sensors |
| **GTSAM, Ceres** | IMU preintegration in factor graphs (Forster et al. 2015) |

---

## Recommended reading

- Titterton & Weston, *Strapdown Inertial Navigation Technology* (2nd ed., 2004) — *the* reference for the math
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems* (2nd ed., 2013) — deep, authoritative
- Forster, Carlone, Dellaert, Scaramuzza (2015), *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* — the modern factor-graph IMU model
- Solà (2017), *Quaternion kinematics for the error-state Kalman filter* (arXiv 1711.02508) — clean treatment of IMU integration on $SO(3)$
- Allan-variance tutorial: IEEE Std 952-1997 ("IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros")

---

## Dataview

```dataview
LIST FROM #imu OR #sensors WHERE contains(file.outlinks, [[IMU_Sensors]])
```
