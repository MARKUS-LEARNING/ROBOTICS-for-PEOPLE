---
title: Sim2Real Experiments
description: "A hands-on lab you can run on a laptop: tune a cart-pole controller on a 'nominal' MuJoCo model, test it against a perturbed 'reality' (heavier pole, actuator delay), then choose it under randomisation and measure the difference. Ends with a protocol and record sheet for real-hardware sim-to-real experiments."
aliases:
  - Sim2Real Lab
  - Sim-to-Sim Experiment
tags:
  - sim-to-real
  - simulation
  - mujoco
  - experiment
  - robot-learning
type: lab
level: intermediate
subfield:
  - robot-learning
  - software
  - control
status: reviewed
author: Jordan_Smith
date: 2025-04-29
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Sim-to-Real_Transfer]]"
  - "[[MuJoCo]]"
  - "[[Linear_Quadratic_Regulator_(LQR)]]"
related:
  - "[[Reinforcement_Learning_for_Robots]]"
  - "[[Robot_Simulation]]"
  - "[[GPU-Parallel_Simulation]]"
  - "[[Labs_Projects_and_Tutorials]]"
key_takeaways:
  - "A controller tuned only on a nominal model can be excellent there and fail badly when reality differs a little (here: 30-60 ms of actuator delay)."
  - "Choosing the controller across randomised conditions raised success from 38% to 83% on 200 random 'realities', at the cost of the most extreme cases."
  - "Sim-to-sim testing (train in one model, test in a deliberately different one) is a cheap rehearsal for sim-to-real."
sources:
  - "Tobin, J. et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. https://arxiv.org/abs/1703.06907"
  - "Peng, X. B. et al. (2018). Sim-to-Real Transfer of Robotic Control with Dynamics Randomization. https://arxiv.org/abs/1710.06537"
  - "Google DeepMind (2026). MuJoCo documentation: Python bindings. https://mujoco.readthedocs.io/en/stable/python.html"
  - "Zhao, W., Peña Queralta, J. and Westerlund, T. (2020). Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey. https://arxiv.org/abs/2009.13303"
---

# Sim2Real Experiments

This lab makes the [[Sim-to-Real_Transfer|sim-to-real gap]] something you can see and measure on a laptop. You will tune a controller on a "nominal" simulation, test it on a deliberately different simulation standing in for reality, and then use randomisation to make it robust. No robot is needed. The second half gives a protocol for doing the same with real hardware.

> **Etymology.** *Experiment* is from Latin *experimentum*, "a trial, test", from *experiri*, "to try". The same root gives *experience* and *expert*: someone who has tried things.

## Part 1: the sim-to-sim gap on a cart-pole

**Setup.** Python 3.11+ with `pip install mujoco numpy scipy`. The experiment runs in about 15 seconds.

**The system.** A cart on a rail with a pole hinged on top, pushed by a motor limited to ±8 N. The controller is a [[Linear_Quadratic_Regulator_(LQR)|linear-quadratic regulator]] computed from MuJoCo's own linearisation (`mjd_transitionFD`). Weights $q_\theta = 100,\ r = 0.01$ ask for fast, aggressive balancing.

**"Reality".** The same system with a heavier pole (0.1–1.5 kg instead of 0.1) and an actuator delay (0–120 ms) that the controller does not know about. Success means the pole stays within 0.5 rad and the cart within 1.8 m for 5 s, starting from a 0.15 rad tilt.

```python
import numpy as np, mujoco, scipy.linalg

XML = """
<mujoco model="cartpole">
  <option timestep="0.002"/>
  <worldbody>
    <body name="cart">
      <joint name="slider" type="slide" axis="1 0 0" damping="0.1"/>
      <geom type="box" size="0.1 0.05 0.05" mass="1.0"/>
      <body name="pole">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="0.01"/>
        <geom name="pole" type="capsule" fromto="0 0 0 0 0 0.6" size="0.02" mass="0.1"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor joint="slider" ctrlrange="-8 8"/></actuator>
</mujoco>"""

def make(pole_mass=0.1):
    m = mujoco.MjModel.from_xml_string(XML)
    pole = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pole")
    scale = pole_mass / m.body_mass[pole]
    m.body_mass[pole] *= scale; m.body_inertia[pole] *= scale
    return m

def lqr_gains(m, q_angle=100.0, r=0.01):
    """Linearise around upright with finite differences, solve discrete-time LQR."""
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    nx = 2 * m.nv; A = np.zeros((nx, nx)); B = np.zeros((nx, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, True, A, B, None, None)
    Q = np.diag([1.0, q_angle, 0.1, 0.1]); R = np.eye(m.nu) * r      # state: x, theta, x_dot, theta_dot
    P = scipy.linalg.solve_discrete_are(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)

def run(m, K, delay_ms=0.0, theta0=0.15, seconds=5.0):
    d = mujoco.MjData(m); d.qpos[1] = theta0
    lag = int(round(delay_ms / 1000 / m.opt.timestep)); queue = [0.0] * (lag + 1)
    for _ in range(int(seconds / m.opt.timestep)):
        x = np.concatenate([d.qpos, d.qvel])
        queue.append(float(np.clip(-(K @ x)[0], -8, 8)))
        d.ctrl[0] = queue.pop(0)                                  # the command arrives delay_ms late
        mujoco.mj_step(m, d)
        if abs(d.qpos[1]) > 0.5 or abs(d.qpos[0]) > 1.8 or not np.isfinite(d.qpos).all():
            return False
    return True

nominal = make(0.1)
K_nominal = lqr_gains(nominal)

# randomisation: choose the LQR weights that succeed most often across random "realities"
rng = np.random.default_rng(0)
sample = lambda n: [(rng.uniform(0.1, 1.5), rng.uniform(0, 120)) for _ in range(n)]
train, test = sample(60), sample(200)
score = lambda K, conds: np.mean([run(make(pm), K, dl) for pm, dl in conds])
candidates = [(q, r) for q in (1, 3, 10, 30, 100) for r in (0.1, 1, 10, 100)]
best = max(candidates, key=lambda qr: score(lqr_gains(nominal, *qr), train))
K_rand = lqr_gains(nominal, *best)
print(best, score(K_nominal, test), score(K_rand, test))
```

### Results (MuJoCo 4.0.1, seed 0)

Controller tuned on the nominal model (rows: pole mass in kg; columns: actuator delay in ms):

| Pole mass | 0 | 30 | 60 | 90 | 120 |
|---|---|---|---|---|---|
| 0.1 (nominal) | ✓ | ✓ | ✗ | ✗ | ✗ |
| 0.3 | ✓ | ✓ | ✗ | ✗ | ✗ |
| 0.6 | ✓ | ✓ | ✗ | ✗ | ✗ |
| 1.0 | ✓ | ✓ | ✗ | ✗ | ✗ |
| 1.5 | ✓ | ✓ | ✗ | ✗ | ✗ |

Controller whose weights were chosen under randomisation ($q_\theta = 100,\ r = 10$):

| Pole mass | 0 | 30 | 60 | 90 | 120 |
|---|---|---|---|---|---|
| 0.1 (nominal) | ✓ | ✓ | ✓ | ✓ | ✓ |
| 0.3 | ✓ | ✓ | ✓ | ✓ | ✓ |
| 0.6 | ✓ | ✓ | ✓ | ✓ | ✓ |
| 1.0 | ✓ | ✓ | ✓ | ✓ | ✗ |
| 1.5 | ✗ | ✗ | ✗ | ✗ | ✗ |

| | Nominal-tuned | Chosen under randomisation |
|---|---|---|
| Success on the nominal model | ✓ | ✓ |
| Success on 200 random "realities" | **38%** | **83%** |

### What the numbers say

1. **The nominal controller was perfect in its own world and fragile outside it.** Mass barely mattered, but any delay of 60 ms or more broke it: aggressive gains plus a delay make the loop unstable. On a real robot, delay comes from sensing, communication and motor drivers, and is easy to underestimate.
2. **Randomisation bought robustness by giving up some performance.** The chosen controller is gentler (a larger control penalty $r$). It survives every delay up to the 1 kg pole, but it can no longer catch the heaviest 1.5 kg pole, which the aggressive controller handled when there was no delay. Randomisation optimises the *average* over the range you randomise; what you randomise, and over what range, is a design decision.
3. **The fix was found without touching "reality".** Only the training conditions changed. That is the whole idea of domain randomisation, applied here to choosing two weights instead of training a neural network.

### Extensions

- Add **friction** randomisation (`m.dof_damping`) and **sensor noise** on `qpos`, and see which failures appear.
- Replace the LQR with a neural-network policy trained by PPO in a GPU simulator such as MuJoCo Playground or [[Isaac_Lab]], with the same randomised parameters (see [[GPU-Parallel_Simulation]]).
- **System identification:** measure the "true" delay by logging command and response, put it in the nominal model, and compare against randomisation.

## Part 2: protocol for a real sim-to-real experiment

1. **Freeze the setup**: robot, firmware, control rate, task definition, success criterion, number of trials (20+ per condition).
2. **Measure before you randomise**: masses, friction where possible, latency from command to motion, sensor noise. Calibrate the simulator first ([[Sim-to-Real_Transfer]]).
3. **Train or tune in simulation** with randomisation ranges based on those measurements.
4. **Rehearse sim-to-sim**: test in a second simulator, or in the same one with different parameters, before touching hardware.
5. **Deploy with limits**: velocity and torque limits, an emergency stop, someone watching.
6. **Evaluate several policies or checkpoints in both simulation and reality**, and check whether simulation ranks them correctly.
7. **Attribute every failure** to one of the four gaps (looks, physics, actuation, task), fix the simulator, repeat.

### Record sheet

| Field | Example |
|---|---|
| Task and success criterion | Pole within 0.5 rad for 5 s from 0.15 rad |
| Robot, firmware, control rate | — |
| Simulator and version | MuJoCo 4.0.1, 2 ms step |
| Randomised parameters and ranges | Pole mass 0.1–1.5 kg; delay 0–120 ms |
| Policies or checkpoints compared | Nominal LQR; randomisation-chosen LQR |
| Trials per condition | 20 |
| Success in simulation / reality | — / — |
| Failures and attributed gap | e.g. oscillation → actuation (delay) |
| What was changed next | — |

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Sim2Real_Experiments]])
```
