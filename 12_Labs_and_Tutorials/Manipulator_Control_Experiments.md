---
title: Manipulator Control Experiments
description: "Hands-on control experiments for robot arms that run on a laptop in MuJoCo, plus how to document them. Includes a complete, tested two-link-arm lab comparing PD control with and without gravity compensation (5.2° sag versus zero), follow-on experiments (gain sweeps, computed torque, impedance, payload changes, learned policies), and a write-up structure for reproducible results."
aliases:
  - Arm Control Lab
  - Manipulator Experiments
tags:
  - control
  - manipulation
  - mujoco
  - simulation
type: lab
level: intermediate
subfield:
  - control
  - manipulation
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[PID_Control]]"
  - "[[Robot_Control]]"
  - "[[MuJoCo]]"
related:
  - "[[Computed_Torque_Control]]"
  - "[[Compliance]]"
  - "[[Robotics_Lab_Template]]"
  - "[[Sim2Real_Experiments]]"
key_takeaways:
  - "A two-link arm in MuJoCo is enough to see the core ideas of manipulator control with nothing but pip install mujoco."
  - "In the tested lab, PD control alone left the shoulder 5.2° below target under gravity; adding MuJoCo's own gravity and Coriolis terms (qfrc_bias) removed the error."
  - "Change one thing at a time, record everything (code version, gains, model), and report numbers, not impressions."
sources:
  - "Google DeepMind (2026). MuJoCo 4.0.0 release notes. https://github.com/google-deepmind/mujoco/releases/tag/4.0.0"
  - "Google DeepMind (2026). MuJoCo documentation: Computation. https://mujoco.readthedocs.io/en/stable/computation/index.html"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
---

# Manipulator Control Experiments

Control theory becomes real when you watch an arm sag under gravity, oscillate with too much gain, or snap to its target once the model is right. These experiments run in [[MuJoCo]] on an ordinary laptop, so anyone can do them before touching hardware.

> **Etymology.** *Experiment* is from Latin *experiri*, "to try out".

## Lab 1: gravity compensation (tested)

**Question.** How much does a PD controller sag under gravity, and does adding the model's gravity torque fix it?

**Setup.** Install with `pip install mujoco numpy`. A two-link arm moves in a vertical plane (upper link 0.3 m, 1.0 kg; lower link 0.25 m, 0.6 kg). Torque limits are ±30 N·m, and the gains are $K_p = 40$ N·m/rad and $K_d = 4$ N·m·s/rad.

```python
import mujoco, numpy as np

XML = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="upper">
      <joint name="shoulder" type="hinge" axis="0 1 0" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1.0"/>
      <body name="lower" pos="0.3 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.025" mass="0.6"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="shoulder" ctrlrange="-30 30"/>
    <motor joint="elbow" ctrlrange="-30 30"/>
  </actuator>
</mujoco>
"""

def run(gravity_comp, kp=40.0, kd=4.0, seconds=3.0):
    m = mujoco.MjModel.from_xml_string(XML); d = mujoco.MjData(m)
    target = np.array([-0.5, 0.8])                 # rad
    for _ in range(int(seconds / m.opt.timestep)):
        tau = kp * (target - d.qpos) - kd * d.qvel
        if gravity_comp:
            tau += d.qfrc_bias                     # gravity + Coriolis from the model
        d.ctrl[:] = np.clip(tau, -30, 30)
        mujoco.mj_step(m, d)
    return np.degrees(target - d.qpos)

for gc in (False, True):
    print(gc, run(gc))
```

**Results** (MuJoCo 4.0.1, after 3 s):

| Controller | Shoulder error | Elbow error |
|---|---|---|
| PD only | −5.22° | −0.97° |
| PD + gravity compensation | 0.00° | 0.00° |

**Interpretation.** Without compensation, each joint settles where its stiffness balances the gravity torque, $K_p e = \tau_g$, so the arm sags. The shoulder carries both links and sags most. Adding `qfrc_bias`, MuJoCo's computed gravity and Coriolis forces, cancels the load exactly, because the model is perfect in simulation. On hardware the model would be imperfect; see Lab 3.

## Follow-on experiments

1. **Gain sweep.** Vary $K_p$ from 10 to 200 and $K_d$ from 0.5 to 20. Plot steady-state sag and overshoot. Expect sag roughly proportional to $1/K_p$, and oscillation when $K_d$ is too low.
2. **Computed torque.** Track a sinusoid using the full model (`mj_inverse` or the mass matrix from `mj_fullM`). Compare tracking error with PD alone (see [[Computed_Torque_Control]]).
3. **Wrong model.** Scale the controller's masses by 0.8 while keeping the simulated arm's true masses. Measure how much error returns, and try integral action or adaptation (see [[Adaptive_Control]]).
4. **Payload.** Add a 0.5 kg box at the tip without telling the controller.
5. **Impedance.** Command a target inside a wall and measure the contact force for different stiffnesses (see [[Compliance]]).
6. **Learned policy.** Train a small RL or imitation policy to reach targets and compare robustness to payload changes.

## Writing it up

Use the [[Robotics_Lab_Template]]: state the question, the exact setup (software versions, model, gains, random seeds), the procedure, results in tables or plots with numbers, interpretation, and what you would test next. Commit the code and parameters (see [[Git_Integration_and_Version_Control]]).

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Manipulator_Control_Experiments]])
```
