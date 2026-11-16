---
title: Policy (Robot Control Policy)
description: "A policy is the rule a robot uses to choose its next action from what it currently observes. Hand-written controllers, reinforcement learning, imitation learning and vision-language-action models are all different ways of producing one."
aliases:
  - Control Policy
  - Robot Policy
tags:
  - policy
  - robot-learning
  - reinforcement-learning
  - imitation-learning
  - control
type: concept
level: beginner
subfield:
  - robot-learning
  - control
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Open_Loop_vs_Closed_Loop]]"
related:
  - "[[Markov_Decision_Process]]"
  - "[[Reinforcement_Learning]]"
  - "[[Imitation_Learning]]"
  - "[[Vision-Language-Action_Models]]"
  - "[[Feedback_Control]]"
key_takeaways:
  - "A policy maps what the robot observes to what it does next: a = π(o)."
  - "Classical controllers, learned neural networks and large vision-language-action models are all policies; they differ in how the mapping is obtained."
  - "Modern learned policies usually output a short chunk of future actions, not one action, and run at 10-50 Hz on top of faster low-level controllers."
sources:
  - "Sutton, R. and Barto, A. (2018). Reinforcement Learning: An Introduction (2nd ed.). http://incompleteideas.net/book/the-book-2nd.html"
  - "Zhao, T. et al. (2023). Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware. https://arxiv.org/abs/2304.13705"
  - "Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137"
  - "Black, K. et al. (2024). π0: A Vision-Language-Action Flow Model for General Robot Control. https://arxiv.org/abs/2410.24164"
---

# Policy (Robot Control Policy)

A **policy** is the rule a robot uses to decide what to do next. It takes in what the robot currently knows (joint angles, camera images, a spoken instruction) and returns an action (joint targets, wheel speeds, a gripper command). Written as a function:

$$a_t = \pi(o_t)$$

where $o_t$ is the observation at time $t$ and $a_t$ the action. Every robot that acts in a [[Open_Loop_vs_Closed_Loop|closed loop]] runs a policy, whether or not anyone calls it that.

> **Etymology.** *Policy* comes from Greek *politeía* (citizenship, the conduct of a city's affairs) through Latin *politia* and Old French *policie*, meaning a course of action adopted by an authority. Richard Bellman borrowed the word for his 1950s work on dynamic programming, where a policy is the rule for choosing a decision in every possible state. Reinforcement learning inherited it from there, and the symbol $\pi$ stuck.

## One idea, many ways to build it

The word unifies what used to look like separate fields. All of these are policies; they differ in *where the mapping comes from*.

| How the policy is made | What it looks like | Typical example | Note |
|---|---|---|---|
| **Written by an engineer** | Equations or rules | PID loop, state machine, MPC | [[PID_Control]], [[Model_Predictive_Control_(MPC)]] |
| **Optimised for a reward** | Neural network trained by trial and error | Legged locomotion trained in simulation | [[Reinforcement_Learning]] |
| **Copied from demonstrations** | Neural network trained on recorded human control | Bimanual folding with ACT | [[Imitation_Learning]] |
| **Pretrained on many robots, then adapted** | Large vision-language-action model | π0.x, GR00T, Gemini Robotics | [[Vision-Language-Action_Models]] |

## Deterministic and stochastic policies

- A **deterministic** policy returns one action: $a = \pi(s)$. Classical controllers are deterministic.
- A **stochastic** policy returns a probability distribution over actions: $a \sim \pi(a \mid s)$. Reinforcement learning usually trains stochastic policies, because randomness is how the robot explores, and gradient methods need a smooth distribution to differentiate.

Stochasticity matters for demonstrations too. If half your demonstrations go left around an obstacle and half go right, a policy that predicts the *average* action drives straight into it. Generative policies such as [[Diffusion_Policy]] model the full distribution of plausible actions and pick one mode, which is a large part of why they work so well on real demonstration data.

## What goes in: observations, not states

Theory is usually written with the true **state** $s$ (everything needed to predict the future). Robots only get **observations** $o$: noisy, partial views. A camera cannot see behind the box; an encoder has quantisation error. Two practical consequences:

- Policies often take a short **history** of observations, or keep an internal memory, to recover what one frame cannot show.
- Learned policies are trained on the *observation* the real robot will have. A policy trained in simulation with perfect state but deployed with camera input will fail. Simulation-trained controllers therefore often use a "teacher-student" setup: a teacher learns with privileged state, and a student learns to imitate it from realistic observations (see [[Reinforcement_Learning_for_Robots]]).

## What comes out: action spaces

| Action space | Meaning | Who uses it |
|---|---|---|
| Joint torques | Direct motor effort | Research on dynamic locomotion; needs very fast control |
| Joint position targets | Setpoints for low-level PD controllers | Most RL locomotion policies (50 Hz targets, 1 kHz PD) |
| End-effector pose | Where the gripper should be | Teleoperation-trained manipulation policies |
| Action chunks | The next $H$ actions at once | ACT, Diffusion Policy, most VLAs |

**Action chunking** deserves a word. Instead of predicting one action per step, the policy predicts the next $H$ steps (for example 50 actions covering one second) and executes several before predicting again. Zhao et al. introduced it with ACT in 2023 to reduce compounding errors in imitation learning; it is now standard in [[Imitation_Learning]], [[Diffusion_Policy]] and [[Vision-Language-Action_Models]].

## How fast a policy runs

Learned policies rarely command motors directly. A common split:

```
camera / joints ──► learned policy (10–50 Hz) ──► targets ──► PD / impedance controller (500–1000 Hz) ──► motors
```

The fast inner loop handles stiffness and safety; the policy decides *what* to do. Large models such as VLAs may run even slower and stream action chunks so the robot never waits.

## Worked example: a gripper that closes on contact

A hand-written policy for a parallel gripper:

```python
def policy(obs):
    if obs.object_between_fingers and obs.grip_force < 5.0:   # newtons
        return {"gripper": "close"}
    if obs.grip_force >= 5.0:
        return {"gripper": "hold"}
    return {"gripper": "open"}
```

A learned policy replaces those `if` statements with a neural network that maps camera pixels and finger readings to the same three commands. It can handle cases the programmer did not foresee, such as transparent or deformable objects, but only if those cases appear in its training data. Choosing between the two is the oldest design decision in robot learning.

## State of the art (as of November 2026)

- **Generalist policies.** A single pretrained policy now controls many robots and tasks: Physical Intelligence's π0.7 (April 2026), NVIDIA's GR00T N1.7 (April 2026) and Google DeepMind's Gemini Robotics 2 family (July 2026). See [[Vision-Language-Action_Models]].
- **Steerable policies.** π0.7 is prompted not only with a task but with metadata such as desired speed and quality, and with subgoal images; the policy is conditioned on *how* to act, not just *what* to do.
- **World action models.** A newer family predicts future video and actions together (NVIDIA's DreamZero and GR00T 2); see [[World_Models]].

## Reading list

- Sutton and Barto, *Reinforcement Learning: An Introduction*, chapter 3: the formal definition of a policy.
- Zhao et al. 2023 (ACT): action chunking.
- Chi et al. 2023 (Diffusion Policy): why multimodal action distributions matter.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Policy]])
```
