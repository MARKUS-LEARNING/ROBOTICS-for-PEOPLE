---
title: Sensors
description: Devices that convert physical phenomena (light, motion, force, temperature) into measurable signals. The "feelers" through which a robot acquires any information at all about its body or its environment.
tags:
  - robotics
  - sensors
  - measurement
  - transduction
  - perception
  - mechatronics
type: Device
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /sensors/
related:
  - "[[Perception]]"
  - "[[Sensor_Fusion]]"
  - "[[Camera_Systems]]"
  - "[[LIDAR]]"
  - "[[IMU_Sensors]]"
  - "[[Encoders]]"
  - "[[GPS]]"
  - "[[Sensor_Calibration_Techniques]]"
  - "[[Mechatronics]]"
  - "[[Feedback_Control]]"
---

# Sensors

A **sensor** is a device that converts a physical phenomenon — light, motion, force, temperature, sound, magnetic field, chemical concentration — into an electrical signal that a computer can read. Sensors are the *only* channel through which a robot acquires information about the world, including information about its own body. Take the sensors away and the entire perception, planning, and control stack has nothing to operate on.

> **Etymology.** From Latin *sentire*, "to feel, perceive, sense" — the same root as *sentient*, *sentiment*, and *consensus* (literally, "feeling together"). A sensor is a *feeler*. The English word *sensor* dates only to the 1920s; before that, *transducer* (Latin *trans-* "across" + *ducere* "to lead" → "to lead across") was the common technical term. They mean almost the same thing — a sensor is just a transducer specialized for measurement.

---

## What every sensor does — the universal recipe

Strip away the physics and every sensor follows the same four-stage pipeline:

1. **Sensitive element.** A physical structure that *physically responds* to the measurand. Piezoelectric crystal squeezed by a force; photodiode struck by a photon; MEMS proof-mass deflected by acceleration.
2. **Transduction.** The physical response becomes an electrical quantity — voltage, charge, capacitance, resistance, frequency.
3. **Conditioning.** Amplifiers, filters, and analog-to-digital converters (ADCs) turn the raw electrical quantity into a clean digital number.
4. **Communication.** The digital number is shipped over a bus (I²C, SPI, UART, CAN, USB, Ethernet) to the host computer.

The first stage is governed by physics; the last three are governed by signal processing and electronics. A *good* sensor gets all four right — a great sensitive element ruined by noisy conditioning is no better than a bad sensor.

---

## The two big partitions

### Proprioceptive vs exteroceptive

Greek *proprio* (one's own) vs *extero* (outside).

- **Proprioceptive** — measures the robot's *own* state. Joint encoders, IMU, motor current, battery voltage, internal temperature. Same sense in which humans have *proprioception* — the awareness of where your limbs are without looking.
- **Exteroceptive** — measures *the world*. Camera, LIDAR, ultrasonic, GPS, microphone, force-torque at the wrist (touching something external).

A useful first-question for any sensor: "is it telling me about *me* or about *the world*?"

### Active vs passive

- **Passive** — receives energy that already exists. Camera, microphone, magnetometer, thermometer.
- **Active** — *emits* energy and measures the return. LIDAR (laser pulse), radar (radio pulse), ultrasonic (sound pulse), structured-light depth (projected pattern). Active sensors have power budgets and interfere with each other; passive sensors don't.

---

## What gets specified on a datasheet

Every sensor has a datasheet, and every datasheet pins down the same characteristics. Knowing them lets you compare sensors on equal footing.

| Specification | What it means | Why it matters |
|---|---|---|
| **Range** | Min/max measurable value | Outside the range, output saturates |
| **Resolution** | Smallest distinguishable change | The least-significant bit you can trust |
| **Accuracy** | Closeness to the *true* value | A trustworthy reading vs a precise lie |
| **Precision (repeatability)** | Spread of repeated readings of the same input | Independent of accuracy |
| **Bias / offset** | Constant error at zero input | Removable by calibration |
| **Drift** | Slow time- or temperature-dependent change in bias | Can't be calibrated away once and forgotten |
| **Bandwidth** | Highest input frequency the sensor tracks | Aliasing waits for the unwary |
| **Sample rate** | How often you get a reading (Hz) | Sets the controller-loop ceiling |
| **Latency** | Delay from event to digital output | Critical for closed-loop control |
| **Noise floor** | RMS noise with constant input | Below this, you measure your own electronics |
| **Cross-axis sensitivity** | Response to inputs you didn't ask about | A z-accelerometer that reports x-acceleration |

The interplay between *accuracy* (how right) and *precision* (how repeatable) is the most-confused pair. A sniper rifle that puts five shots inside a 1 cm circle 10 cm above the bullseye is *precise but inaccurate*. A shotgun that puts five shots in a 30 cm spread centered on the bullseye is *accurate but imprecise*. Robot perception generally cares more about precision than absolute accuracy — bias can be calibrated out, noise cannot.

---

## The error model — every measurement is a number plus a story

For a scalar sensor measuring quantity $x$, the model robotics uses is:

$$
z = S \cdot x + b + \eta
$$

where:

- $z$ is the reading
- $S$ is the **scale factor** (units conversion, ideally 1 in correct units)
- $b$ is the **bias** (constant offset)
- $\eta$ is **noise** — usually modeled as zero-mean Gaussian with variance $\sigma^2$

**Calibration** means estimating $S$ and $b$. **Filtering** (and sensor fusion) means coping with $\eta$.

For vector sensors (IMU, magnetometer, force-torque) $S$ becomes a matrix — typically diagonal-plus-small-cross-terms — and bias becomes a vector. For cameras the model expands to include lens distortion. For range sensors it expands to include range-dependent scale errors. The recipe is always: figure out which parameters are constant (calibrate them once), which drift slowly (re-calibrate periodically), and which are pure noise (filter them).

See [[Sensor_Calibration_Techniques]] for the practical procedure on each common sensor type.

---

## The robotics sensor catalog

| Type | Example sensors | What it measures | Section |
|---|---|---|---|
| **Position (joint)** | [[Encoders]], potentiometers, [[Resolvers]] | Joint angle / linear displacement | Proprio |
| **Orientation** | [[IMU_Sensors]] (gyro), magnetometer | Angular rate, magnetic heading | Proprio |
| **Acceleration** | IMU (accelerometer) | Linear acceleration + gravity | Proprio |
| **Vision** | [[Camera_Systems]], event cameras | Light intensity / color | Extero |
| **Depth** | [[LIDAR]], [[RGB-D_Sensor]], stereo, ToF cameras | 3D structure of the scene | Extero |
| **Range (1D)** | [[Ultrasonic_Sensors]], IR rangers | Distance to a single point | Extero |
| **Force/torque** | Strain-gauge load cells, FT wrist sensors | Forces on the end-effector | Extero |
| **Tactile** | Capacitive, piezoresistive arrays, [[Piezoelectricity\|piezoelectric]] | Contact pressure distribution | Extero |
| **Global pose** | [[GPS]], RTK-GPS, motion capture | World-frame position | Extero |
| **Current/torque (motor)** | Shunt resistors, Hall sensors | Motor current → torque estimate | Proprio |

A typical mobile manipulator (UR5 arm + Husky base + RealSense + Velodyne) carries 10+ sensors of 6+ types running at 100 Hz to 200 kHz, all reporting into a single state estimator. The job of [[Sensor_Fusion]] is to make that mess speak with one voice.

---

## Worked example — interpreting an IMU datasheet

The InvenSense **MPU-6050** (a ~$3 6-axis IMU) lists:

- **Accelerometer:** ±2 g full-scale, 16-bit, RMS noise 400 µg/√Hz, bias ~50 mg.
- **Gyroscope:** ±250°/s full-scale, 16-bit, RMS noise 0.005°/s/√Hz, bias up to ~20°/s, 1.6%/°C temperature drift.

Read this as:

- **Resolution:** 16 bits over ±2 g → 4 g range / 2¹⁶ ≈ 61 µg per LSB. Noise floor is 400 µg, so the bottom several bits are dominated by noise; only the top ~9 bits carry trustworthy signal.
- **Bias of 20°/s on the gyro** is enormous — integrated over 1 second it produces 20° of drift. Without calibration the gyro is unusable for orientation tracking past a fraction of a second.
- **Temperature drift of 1.6%/°C** means the bias *changes* as the chip heats up. A one-time bias calibration won't hold; runtime estimation is required (this is what the gyroscope-bias state of an [[Kalman_Filter|EKF]] is for).

This pattern — cheap sensor with manageable noise but unmanageable bias — is exactly why sensor fusion exists. A $3 IMU + a slow but accurate magnetometer + a periodic GPS fix together give better orientation than any one of them alone.

---

## Hardware integration — the parts every robot needs

| Stage | What goes wrong if you skip it |
|---|---|
| **Mechanical mounting** | Vibration, misalignment → cross-axis errors and signal-to-noise loss |
| **Power filtering** | Switching-supply ripple shows up as noise on every analog reading |
| **Cable shielding** | Long unshielded I²C runs pick up motor-PWM EMI |
| **Time synchronization** | Multi-sensor fusion needs $<\!1$ ms timestamp alignment; use PTP or hardware triggers |
| **Calibration archive** | Calibration parameters per-unit, per-temperature; lose them and the sensor is just a number generator |

Time synchronization deserves special emphasis — it is the most-overlooked source of bugs in multi-sensor robots. A camera frame and an IMU sample taken 30 ms apart but timestamped identically will produce a 30 ms × 100°/s = 3° rotation error in any visual-inertial fusion. Fix it once with a PTP-synchronized network and stop chasing ghosts.

---

## Recommended reading

- Siciliano, Sciavicco, Villani, Oriolo, *Robotics: Modelling, Planning and Control*, Ch. 5 — the textbook treatment of robot sensing
- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005), Ch. 6 — sensor models from a Bayesian-filtering perspective
- Borenstein, Everett, Feng, *Where Am I? Sensors and Methods for Mobile Robot Positioning* (1996) — old but still the canonical taxonomy
- Fraden, *Handbook of Modern Sensors* (5th ed., 2016) — physics-of-transduction encyclopedia
- Texas Instruments / Analog Devices application notes — the *real* education in sensor electronics is in chip-vendor app notes

---

## Dataview

```dataview
LIST FROM #sensors OR #robotics WHERE contains(file.outlinks, [[Sensors]])
```
