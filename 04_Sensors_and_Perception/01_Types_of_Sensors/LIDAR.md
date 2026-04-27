---
title: LIDAR (Light Detection and Ranging)
description: An active range sensor that emits laser pulses and times their return. Produces dense 3D point clouds of the surrounding scene at tens of thousands to millions of points per second — the workhorse exteroceptive sensor for autonomous vehicles, mobile robots, and aerial mapping.
tags:
  - sensors
  - lidar
  - range-sensor
  - perception
  - autonomous-driving
  - mobile-robot
  - point-cloud
type: Sensor
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /lidar/
related:
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[Range_Sensor]]"
  - "[[Camera_Systems]]"
  - "[[RGB-D_Sensor]]"
  - "[[SLAM]]"
  - "[[Mapping]]"
  - "[[Sensor_Fusion]]"
  - "[[IMU_Sensors]]"
---

# LIDAR (Light Detection and Ranging)

A **LIDAR** is an active range sensor that fires laser pulses and times their return to compute distance. By repeating this process across many directions — mechanically by spinning, or electronically by steering the beam — a LIDAR builds a 3D **point cloud** of every solid surface in its field of view.

> **Etymology.** *LIDAR* is an acronym, **LI**ght **D**etection **A**nd **R**anging, modeled deliberately on the older *RADAR* (**RA**dio **D**etection **A**nd **R**anging, 1940). The term emerged at Hughes Aircraft Laboratories in the early 1960s, shortly after Theodore Maiman's first working laser in 1960. It is *not* a portmanteau of "light" and "radar" despite the appearance — the second half of the acronym is the same as RADAR's. Sometimes spelled "LiDAR" with mixed case, especially in earth-sciences literature, but the all-caps form is older and dominant in robotics.

---

## How a LIDAR measures distance

The core physics is the **time of flight (ToF)** of light:

$$
d = \frac{c \cdot t_{\text{round-trip}}}{2}
$$

where $c \approx 3 \times 10^8$ m/s is the speed of light and $t_{\text{round-trip}}$ is the time for the pulse to leave the sensor, hit a target, and return.

The factor of 2 accounts for the round trip. The challenge is that for $d = 10$ m, $t = 67$ ns. To get 1 cm range resolution requires timing precision of $\sim\!67$ ps. Modern LIDARs use either:

1. **Pulsed time-of-flight** (most common) — fire a short laser pulse (a few ns), record the echo arrival time with a time-to-digital converter (TDC) running at GHz rates.
2. **Phase-shift continuous-wave (CW)** — modulate a continuous laser at MHz, measure the phase shift of the returning light. Used in indoor / short-range LIDARs (e.g., Hokuyo URG).
3. **Frequency-modulated continuous-wave (FMCW)** — sweep the laser frequency, measure beat frequency between transmitted and received light. Recovers both *range* and *velocity* (Doppler) at every point — the basis of newer "4D LIDAR" (Aeva, Mobileye Chronos).

Each measurement returns a $(\theta, \phi, r, I)$ tuple — azimuth, elevation, range, intensity — which gets converted to a Cartesian point $(x, y, z)$ in the sensor frame.

---

## How the beam gets steered

The 3D coverage problem — point a laser pulse at every direction in the FoV, fast — has multiple solutions:

| Steering | Mechanism | Examples | Trade-off |
|---|---|---|---|
| **Mechanical spinning** | Motor spins the entire optical assembly | Velodyne HDL-64, Ouster OS-1, Robosense RS-LiDAR-32 | 360° FoV, moving parts wear out |
| **Mechanical galvo / polygon mirror** | Mirror oscillates / rotates inside a stationary housing | Sick MRS, Hesai Pandar | Compact, narrower FoV |
| **MEMS micro-mirror** | Tiny silicon mirrors steer the beam electrostatically | Innoviz, Luminar Hydra | Solid-state, limited FoV |
| **Optical phased array (OPA)** | Phase-shift many emitters to electronically steer | Quanergy S3 (early), research-grade | No moving parts, immature |
| **Flash LIDAR** | Illuminate the *entire* scene with one pulse, image with a 2D detector array | Continental, ASC | Range-limited but very fast |
| **Frequency-steered (FMCW + grating)** | Wavelength tuning steers the beam through a dispersive element | Aurora, Aeva | Solid-state + Doppler |

Mechanical spinning LIDARs ruled the autonomous-vehicle world from ~2010 (Velodyne HDL-64 on Stanley/Junior) to ~2020. The industry has been migrating toward solid-state designs ever since for cost, robustness, and form factor.

---

## What ends up in the point cloud

A single sweep of a Velodyne VLP-16 (16 laser channels, 10 Hz spin) produces ~300,000 points per second. A point cloud frame is typically a list of:

```
(x, y, z, intensity, ring_id, timestamp)
```

- **Position $(x, y, z)$** — the geometric measurement, in the sensor frame.
- **Intensity** — return strength; depends on surface reflectivity and incidence angle. Intensity images are surprisingly camera-like and useful for object recognition.
- **Ring ID** — which laser channel produced the point (vertical row in the scan).
- **Timestamp** — per-point time, *crucial* for motion-compensation when the sensor is moving.

The motion-compensation point matters: a spinning LIDAR takes 100 ms to complete one sweep. If the vehicle is moving at 20 m/s, the points at the start and end of the sweep are taken from positions 2 m apart. Motion-compensation un-warps the cloud using IMU + odometry — see [[Sensor_Fusion]] for the canonical recipe.

---

## Specifications worth checking

| Spec | Typical range | What it controls |
|---|---|---|
| **Channels** | 1, 4, 16, 32, 64, 128 | Vertical resolution |
| **Range** | 30 m (indoor) — 300 m (automotive) | How far you can see |
| **Range accuracy** | 1–5 cm | Geometric noise per point |
| **Angular resolution** | 0.1°–0.4° horizontal, 0.5°–2° vertical | Spatial sampling density |
| **Field of view** | 360° hor / 30°–45° vert (spinning); ~60°×30° (solid-state) | Coverage |
| **Frame rate** | 5–20 Hz | How often the cloud refreshes |
| **Wavelength** | 905 nm (most), 1550 nm (longer-range, eye-safe) | Range × eye safety |
| **Points/sec** | 100k–4M | Compute load downstream |

Wavelength is a subtle but important choice. **905 nm** lasers are cheap silicon technology but limited in maximum-permitted-exposure (MPE) for eye safety. **1550 nm** (Luminar, Aeva) sits in a band absorbed by the human cornea before reaching the retina, allowing 40× higher pulse energy → longer range, but needs more expensive InGaAs detectors. Most consumer-robotics LIDARs are 905 nm; long-range automotive LIDARs are increasingly 1550 nm.

---

## LIDAR vs camera vs radar — when to use which

| | **LIDAR** | **Camera** | **Radar** |
|---|---|---|---|
| Active/passive | Active (laser) | Passive (ambient light) | Active (radio) |
| Output | 3D point cloud | 2D image | Sparse range/velocity returns |
| Direct depth? | Yes | No (need stereo or learning) | Yes |
| Resolution | Sparse but accurate | Dense pixels, no depth | Very sparse |
| Range | 30–300 m | Limited by resolution | Up to 200+ m, sees through fog |
| Lighting independence | Works in pitch dark | Fails in low light | Indifferent to light |
| Weather | Rain/fog/snow degrade | Rain/fog degrade | Excellent through weather |
| Color/semantics | None (intensity only) | Rich color and texture | None |
| Cost | $$$$ — historically very expensive | $ — cheap | $$ — moderate |

The dominant pattern in modern robots is **fusion**: LIDAR for geometry, camera for semantics, IMU for short-term motion, radar for long-range or weather-resistant tracking. See [[Sensor_Fusion]] for the canonical recipes.

The "vision-only" debate (Tesla, Wayve, comma.ai) and the "LIDAR-or-bust" camp (Waymo, Cruise, most academic robotics) have been arguing for a decade about whether expensive depth sensing is worth its weight. The answer in 2025 is "it depends on the operating envelope" — robotaxis in well-mapped cities can survive on vision; mining trucks in dust-storm conditions cannot.

---

## What you do with a point cloud — the algorithm catalog

| Task | Algorithm | Library |
|---|---|---|
| **Ground segmentation** | RANSAC plane fit, GraSPP, Patchwork | PCL, `pyransac3d` |
| **Voxel downsampling** | Grid-based averaging | PCL `VoxelGrid`, Open3D |
| **Clustering** | Euclidean clustering, DBSCAN | PCL, scikit-learn |
| **Registration** (align two clouds) | ICP (Iterative Closest Point), GICP, NDT | PCL, Open3D, GICP-SLAM |
| **Object detection** | PointPillars, CenterPoint, VoxelNet | OpenPCDet, MMDetection3D |
| **Mapping (odometry + map)** | LOAM, LeGO-LOAM, FAST-LIO, KISS-ICP | Open-source ROS packages |
| **Semantic segmentation** | RandLA-Net, Cylinder3D, SPVNAS | OpenPCDet |

For SLAM and mapping with LIDAR, see [[SLAM]] and [[Mapping]].

---

## Worked example — measuring a wall

A LIDAR is pointed at a flat wall 5 m away. The vertical FoV is 30°, with 16 channels. Each channel hits the wall at a slightly different angle, producing 16 points along a vertical line.

- Time of flight: $t = 2 \times 5 / 3 \times 10^8 = 33.3$ ns. The TDC must time this to ~30 ps for 1 cm accuracy.
- Per-pulse range error of 2 cm (typical for a VLP-16) means each point is at $5.00 \pm 0.02$ m.
- Fitting a plane through 16 points reduces the noise: $\sigma_{\text{plane}} = \sigma_{\text{point}} / \sqrt{N} \approx 0.5$ cm.
- Multiple sweeps and a low-pass filter further reduce noise to mm-level.

This is why LIDAR-based SLAM works so well: each individual point is noisy, but fitting *geometric primitives* (planes, edges) through many points averages the noise away. See [[SLAM]].

---

## Common production LIDARs

| Model | Channels | Range | Notable use |
|---|---|---|---|
| **Hokuyo URG-04LX / UTM-30LX** | 1 (2D plane) | 5–30 m | Indoor mobile robots, TurtleBot |
| **Velodyne VLP-16 ("Puck")** | 16 | 100 m | Research robots, KITTI dataset |
| **Velodyne HDL-64E** | 64 | 120 m | DARPA Urban Challenge, early Waymo |
| **Ouster OS-1 / OS-2** | 32, 64, 128 | 200 m | Modern AV stacks, digital LIDAR |
| **Livox Mid-360 / Avia** | non-repetitive scan | 200 m | Drones, low-cost SLAM |
| **Luminar Iris** | (1550 nm FMCW) | 250 m | Volvo EX90, automotive |
| **Robosense Helios / Bpearl** | 16–128 | 100–230 m | Chinese AV market |

---

## Tooling

| Tool | Use |
|---|---|
| **PCL (Point Cloud Library)** | Classical point-cloud processing, ICP, segmentation |
| **Open3D** | Modern Python-first PCL replacement |
| **ROS 2 `sensor_msgs/PointCloud2`** | Standard point-cloud message |
| **`pcl_ros` / `point_cloud_proc`** | ROS-side filters and visualization |
| **RViz / Foxglove Studio** | 3D point-cloud visualization |
| **OpenPCDet** | LIDAR object-detection framework (PointPillars, CenterPoint, etc.) |
| **KISS-ICP / FAST-LIO / LIO-SAM** | Modern open-source LIDAR(-inertial) odometry |

---

## Recommended reading

- Thrun, Burgard, Fox, *Probabilistic Robotics*, Ch. 6.3 — beam-based sensor models for LIDAR
- Wang & Wang (2020), *3D LiDAR and Stereo Fusion using Stereo Matching Network with Conditional Cost Volume Normalization* — modern fusion treatment
- Zhang & Singh (2014), *LOAM: Lidar Odometry and Mapping in Real-time* — the paper that defined LIDAR SLAM
- Vizzo et al. (2023), *KISS-ICP: In Defense of Point-to-Point ICP* — a startlingly simple modern baseline
- Velodyne / Ouster / Luminar manuals — vendor-specific details on point-cloud formats and calibration

---

## Dataview

```dataview
LIST FROM #lidar OR #sensors WHERE contains(file.outlinks, [[LIDAR]])
```
