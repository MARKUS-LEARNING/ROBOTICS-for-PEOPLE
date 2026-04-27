---
title: Mapping
description: The process of building a representation of the environment from sensor data so a robot can plan, navigate, and reason about it. The taxonomy of map types — occupancy grids, point clouds, feature maps, topological graphs, semantic and HD maps — and the algorithms that build each.
tags:
  - robotics
  - mapping
  - perception
  - slam
  - occupancy-grid
  - point-cloud
  - mobile-robot
layout: default
category: robotics
author: Jordan_Smith
date: 2025-04-29
permalink: /mapping/
related:
  - "[[Localization]]"
  - "[[SLAM]]"
  - "[[Sensors]]"
  - "[[Perception]]"
  - "[[LIDAR]]"
  - "[[RGB-D_Sensor]]"
  - "[[Sensor_Fusion]]"
  - "[[GPS]]"
  - "[[Path_Planning]]"
  - "[[Configuration_Space]]"
---

# Mapping

**Mapping** is the process of converting a stream of sensor measurements into a persistent representation of the environment that a robot can plan, navigate, and reason against. A map answers questions like: *what space is free, what space is occupied, where are the doorways, where are the lanes, where is the chair I was looking at five minutes ago?* Different planning and perception tasks need different answers, which is why robotics has not converged on a single map type but maintains a small zoo of representations, each with its own algorithms.

> **Etymology.** From Latin *mappa*, "napkin, cloth, signal cloth," likely of Punic/Phoenician origin. The Romans drew sketches of properties on cloth (*mappa mundi* = "cloth of the world"), and the term migrated from the cloth itself to the drawing on it. The word arrives in English in the 16th century via French *mappemonde*. The robotics sense — a digital representation of the environment — was first formalized in Hans Moravec's PhD work at Stanford (1980) and crystallized as **occupancy grids** by Alberto Elfes (1989).

---

## What a map is for

A map serves four downstream purposes, and the right map type depends on which one matters most:

1. **Navigation** — "find me a path from here to there." Needs free-space representation; collision-checking efficiency dominates the design.
2. **Localization** — "where am I?" Needs sensor-comparable structure (the same kind of features the robot can re-observe).
3. **Reasoning / task planning** — "where is the kitchen?" Needs semantic labels (rooms, objects, affordances).
4. **Inspection / digital twin** — "show me what the robot has seen." Needs photorealism or geometric fidelity.

A single robot stack often runs *multiple* maps simultaneously — a 2D occupancy grid for local planning, a 3D point cloud for collision-avoidance, a topological graph for high-level routing. They are different views of the same world.

---

## The five canonical map types

### 1. Occupancy grid (Elfes 1989)

Discretize the world into a grid of cells. Each cell holds a single number: the **probability that the cell is occupied** by an obstacle.

- **Update rule.** For each cell $c$ and each measurement $z$, update the log-odds:

$$
\ell_t(c) = \ell_{t-1}(c) + \log \frac{p(c \mid z_t)}{1 - p(c \mid z_t)} - \log \frac{p(c)}{1 - p(c)}
$$

Log-odds keeps the math additive (no division), saturate-clamping prevents overconfidence.

- **Strengths.** Simple, fast, well-suited to 2D LIDAR. Direct interface to grid-based path planners (A*, D*).
- **Weaknesses.** Memory grows with $1/r^2$ in 2D, $1/r^3$ in 3D. Doesn't handle fine geometry or unbounded environments.
- **3D variant — Octree (OctoMap).** Adaptive resolution: empty regions stay coarse, occupied regions subdivide.

This is the default map type for indoor mobile robots with 2D LIDAR (TurtleBot, Roomba research platforms) and the foundation of the ROS `nav_msgs/OccupancyGrid` message.

### 2. Point cloud

A list of 3D points $(x, y, z)$, optionally with color, intensity, or normals. Direct output of LIDAR and RGB-D depth.

- **Strengths.** Lossless representation of what the sensor saw. Well-supported by libraries (PCL, Open3D, CloudCompare).
- **Weaknesses.** No notion of free space. Memory grows with the number of measurements unless downsampled. No topology.
- **Compressed form — voxel grid / Octree.** Group points into voxels for downsampling and storage.
- **Mesh form — triangulated surface.** Good for visualization and collision-checking; expensive to maintain online.

The dominant representation for autonomous driving (HD maps stored as compressed point clouds) and high-fidelity 3D reconstruction.

### 3. Feature / landmark map

A list of distinct re-observable landmarks — corners, line segments, AprilTags, ORB-feature points — with their estimated positions.

- **Strengths.** Sparse → cheap memory, fast association. Mathematically clean for [[SLAM]] back-ends.
- **Weaknesses.** Only useful where features can be reliably re-detected; throws away geometry between features.
- **Used by.** Most early SLAM (Davison MonoSLAM 2003, EKF-SLAM, FastSLAM), AprilTag-based fiducial localization.

### 4. Topological map

A graph where nodes are *places* (rooms, corridors, intersections) and edges are *connections* between them. No metric coordinates required at the high level.

- **Strengths.** Compact, scales to entire buildings or cities. Natural for high-level reasoning ("go to room 312 via the elevator").
- **Weaknesses.** No metric → can't do trajectory optimization or geometric collision-checking inside it.
- **Combined form — hybrid metric-topological.** Each node holds a small local metric map; edges hold transition rules. Used by service robots and museum-tour robots.

### 5. Semantic / HD map

Map augmented with object labels, lane geometry, traffic-control elements, drivable-area polygons. Built once and refreshed periodically; the robot localizes against it.

- **Strengths.** Tied to the planning task — lane lines for driving, doorways for service robots, shelf positions for warehouse robots.
- **Weaknesses.** Expensive to build; requires labeling; needs continual refresh as the world changes.
- **Used by.** Waymo, Cruise, Mobileye REM, Ouster Velodyne TomTom OpenLR — every modern AV stack depends on HD maps.

A 2025-vintage AV stack uses *all five* simultaneously: HD map for lane geometry, semantic objects for traffic actors, occupancy grid for local cost, point cloud for unmapped obstacles, topological graph for route planning.

---

## How the map gets built — three regimes

### Mapping with known poses

The robot knows exactly where it is at every step (motion-capture, RTK-GPS, instrumented test track). All sensor measurements get transformed into the world frame and aggregated into the map. *Easy* — but this is rarely the case outside a lab.

### SLAM (Simultaneous Localization and Mapping)

The robot builds the map *and* localizes itself in it concurrently. Both pose and map are unknown; both must be estimated jointly. This is the dominant case in real robotics. See [[SLAM]] for the algorithms (EKF-SLAM, FastSLAM, GraphSLAM, ORB-SLAM3, Cartographer, FAST-LIO).

The deep insight: pose and map are coupled — uncertainty in one causes uncertainty in the other. The mathematical machinery (factor graphs, bundle adjustment) is exactly the right tool for joint inference.

### Mapping with prior map (relocalization + update)

The robot has a prior map (built earlier or by another robot) and updates it with new observations. Detect changes, age out stale features, register new objects. This is what fleet-mapped AVs and warehouse robot fleets do.

---

## Active mapping and exploration

A robot tasked with *mapping an unknown space* needs to choose where to go to see new ground. Two algorithm families:

- **Frontier exploration (Yamauchi 1997)** — drive toward the boundary between known-free and unknown space.
- **Information-theoretic exploration** — pick the next viewpoint to maximize expected information gain about the map.

Both close the loop between perception and planning: the perception system reports its uncertainty; the planner chooses motions to reduce it. Modern systems (Compsmate, Voxblox-NBV, Voxfield) use 3D octrees and information gain in real time.

---

## Map-building gotchas

1. **Scan registration drift.** Errors compound when registering each new scan to the previous frame. Loop closure (recognizing a previously visited place) corrects long-term drift. Without loop closure, even good odometry produces a banana-shaped trajectory.
2. **Dynamic objects.** People, cars, doors. Either filter them out before mapping (semantic mask) or run a *static-world* prior in the update rule (decay factor on free cells).
3. **Sensor-specific biases.** A 905 nm LIDAR sees mirrors as transparent, glass as either transparent or specular. Cameras struggle in low light. Each sensor needs its own map sanity check.
4. **Map size.** A 200 m × 200 m × 5 m occupancy grid at 5 cm resolution is 80M cells — about 80 MB at 1 byte/cell. OctoMap and Voxblox compress this 10–100×. NeRFs and Gaussian splats compress further at the cost of differentiability.
5. **Coordinate-frame consistency.** A multi-session map built across multiple drives needs a consistent frame, typically anchored by GPS or a fiducial. ROS `tf2` plus `map_server` handles the bookkeeping.

---

## Worked example — building a 2D occupancy grid with AMCL-style sweep

A TurtleBot with a 2D LIDAR drives through a hallway:

1. **Initialization.** Empty grid, all cells at prior $p = 0.5$ (log-odds 0).
2. **Per-scan update.** For each LIDAR ray ending at distance $r$ in direction $\theta$:
   - Cells along the ray *before* $r$ are likely free → decrement log-odds (e.g., by 0.4).
   - Cell at $r$ is likely occupied → increment log-odds (e.g., by 0.85).
3. **Visualization.** Convert log-odds back to probability via $p = 1 / (1 + e^{-\ell})$. Render: black for occupied, white for free, gray for unknown.

After ~5 minutes of driving the corridor, the map is recognizable as the floor plan. Run `slam_toolbox` and you get this for free; `gmapping` was the older predecessor.

---

## Modern frontier — implicit maps

Since 2020 a new family of map representations has emerged from neural rendering:

- **NeRF (Mildenhall et al. 2020).** A 3D scene is encoded in the weights of a small MLP that maps $(x, y, z, \text{view})$ to color and density. Photorealistic novel-view synthesis from sparse images. Slow to train, slow to query.
- **Gaussian Splatting (Kerbl et al. 2023).** Represent the scene as a set of 3D Gaussians with color and opacity. Fast to render (rasterization), differentiable, plays well with SLAM (e.g., SplaTAM, MonoGS).
- **Implicit signed-distance fields (SDF).** Each query point returns the signed distance to the nearest surface. Useful for direct collision-checking and trajectory optimization.

These representations are differentiable end-to-end with the rest of the perception stack, opening the door to systems where the *map itself* is a learned component co-optimized with localization, planning, and control.

---

## Tooling

| Tool | Use |
|---|---|
| **`slam_toolbox`** | Modern 2D LIDAR SLAM + localization (ROS 2 default) |
| **Cartographer** | Google's 2D/3D real-time SLAM |
| **OctoMap, UFOMap, Voxblox** | 3D occupancy mapping |
| **PCL, Open3D** | Point-cloud processing libraries |
| **`map_server` (Nav2)** | Save and load 2D occupancy grids |
| **RTAB-Map** | Visual+LIDAR SLAM with appearance-based loop closure |
| **NeRFStudio, Splatfacto** | Modern neural mapping / Gaussian splatting toolchains |

---

## Recommended reading

- Elfes, A. (1989), *Using Occupancy Grids for Mobile Robot Perception and Navigation* — the original occupancy-grid paper
- Thrun, Burgard, Fox, *Probabilistic Robotics* (2005), Ch. 9-13 — exhaustive treatment of mapping algorithms
- Hornung et al. (2013), *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees*
- Cadena, Carlone, Carrillo, Latif, Scaramuzza, Neira, Reid, Leonard (2016), *Past, Present, and Future of Simultaneous Localization And Mapping: Toward the Robust-Perception Age* — survey
- Mildenhall et al. (2020), *NeRF: Representing Scenes as Neural Radiance Fields* — the implicit-mapping shift

---

## Dataview

```dataview
LIST FROM #mapping OR #robotics WHERE contains(file.outlinks, [[Mapping]])
```
