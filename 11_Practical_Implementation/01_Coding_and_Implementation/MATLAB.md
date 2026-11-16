---
title: MATLAB
description: "MATLAB and Simulink in robotics: numerical computing, control design, modelling and simulation, the Robotics System Toolbox and ROS Toolbox, code generation for embedded targets, and where MATLAB fits next to Python and ROS 2 in 2026. Includes a worked homogeneous-transform example and pointers to the Python equivalents."
aliases:
  - Simulink
  - MATLAB Robotics System Toolbox
tags:
  - matlab
  - simulation
  - control
type: tool
level: beginner
subfield:
  - software
  - control
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Linear_Algebra_for_Robotics]]"
related:
  - "[[Python_for_Robotics]]"
  - "[[Control_Systems]]"
  - "[[Homogeneous_Transformation]]"
key_takeaways:
  - "MATLAB is a matrix-oriented numerical environment; Simulink adds block-diagram modelling and simulation, strongest for control design and model-based design with code generation."
  - "Robotics System Toolbox and ROS Toolbox provide kinematics, planning and ROS 2 connectivity; Peter Corke's Robotics Toolbox exists for both MATLAB and Python."
  - "Industry uses MATLAB/Simulink heavily for control and automotive work; robot learning and ROS 2 development have largely moved to Python and C++."
sources:
  - "MathWorks (2026). Robotics System Toolbox. https://www.mathworks.com/products/robotics.html"
  - "MathWorks (2026). ROS Toolbox. https://www.mathworks.com/products/ros.html"
  - "Corke, P. (2023). Robotics, Vision and Control: Fundamental Algorithms in Python, 3rd edition. https://link.springer.com/book/10.1007/978-3-031-06469-2"
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition (MATLAB and Python examples). https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
---

# MATLAB

**MATLAB** is a commercial numerical computing environment built around matrices; **Simulink** is its companion for modelling and simulating dynamic systems as block diagrams. Both have long been standard tools in control engineering, and remain widely used in industry and teaching.

> **Etymology.** *MATLAB* = **MAT**rix **LAB**oratory. Cleve Moler wrote the first version in the late 1970s so students could use linear-algebra libraries without Fortran; MathWorks was founded in 1984.

## What it is good at

- **Linear algebra and numerics** with concise syntax.
- **Control design**: transfer functions, state space, Bode plots, LQR, MPC (Control System Toolbox, Model Predictive Control Toolbox).
- **Simulink**: physical modelling (Simscape), controller design, and **code generation** to C/C++ for embedded targets, used extensively in automotive and aerospace.
- **Robotics System Toolbox**: rigid-body trees, inverse kinematics, motion planning, trajectory generation.
- **ROS Toolbox**: connects MATLAB and Simulink to ROS 2 networks and generates ROS 2 nodes.

## Worked example: a homogeneous transform

```matlab
T = trvec2tform([0.5 0 0.2]) * eul2tform([pi/2 0 0], "ZYX");   % translate, then yaw 90°
p_tool = [0.1; 0; 0; 1];                                         % point in tool frame
p_base = T * p_tool                                              % → [0.5; 0.1; 0.2; 1]
```

The tool point 0.1 m along the tool's x axis appears 0.1 m along the base's y axis, because the tool frame is yawed by 90°, offset by the translation (0.5, 0, 0.2). The same calculation in Python with NumPy or the spatialmath library is equally short (see [[Python_for_Robotics]], [[Homogeneous_Transformation]]).

## MATLAB versus Python in 2026

| | MATLAB / Simulink | Python |
|---|---|---|
| Cost | Commercial licences (campus licences common) | Free and open source |
| Strengths | Control design, Simulink model-based design, certified code generation, polished toolboxes | Machine learning, ROS 2 tooling, simulation for learning, huge open ecosystem |
| Typical users | Automotive, aerospace, control and industrial teams; teaching | Robot-learning research, startups, ROS developers |

Many teams use both: design and verify controllers in Simulink, generate embedded code, and run learning and high-level robot software in Python and ROS 2.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[MATLAB]])
```
