---
title: Markov Decision Process (MDP)
description: "The mathematical model behind reinforcement learning: states, actions, transition probabilities, rewards and a discount factor. Solving an MDP means finding the policy that maximises expected discounted reward."
aliases:
  - MDP
  - Markov Decision Process
  - Markov Decision Process (MDP)
  - POMDP
tags:
  - mdp
  - reinforcement-learning
  - probability
  - decision-making
  - optimal-control
type: concept
level: intermediate
subfield:
  - robot-learning
  - mathematics
status: keystone
author: Jordan_Smith
date: 2026-11-15
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Policy]]"
  - "[[Probability_and_Statistics_for_Robotics]]"
related:
  - "[[Reinforcement_Learning]]"
  - "[[Optimal_Control]]"
  - "[[Hamilton-Jacobi-Bellman_Equation]]"
  - "[[Bayesian_Inference]]"
key_takeaways:
  - "An MDP is (S, A, P, R, γ): states, actions, transition probabilities, rewards, discount."
  - "The Bellman equation says a state's value is the immediate reward plus the discounted value of where you land; almost every RL algorithm is a way of solving it approximately."
  - "Robots only observe part of the state, so the honest model is a POMDP; memory, observation history and state estimation are how practice copes."
sources:
  - "Sutton, R. and Barto, A. (2018). Reinforcement Learning: An Introduction (2nd ed.). http://incompleteideas.net/book/the-book-2nd.html"
  - "Mnih, V. et al. (2013). Playing Atari with Deep Reinforcement Learning. https://arxiv.org/abs/1312.5602"
  - "Schulman, J. et al. (2017). Proximal Policy Optimization Algorithms. https://arxiv.org/abs/1707.06347"
---

# Markov Decision Process (MDP)

A **Markov decision process** is the model of a decision-maker acting in an uncertain world, one step at a time. At each step the agent sees a **state**, picks an **action**, receives a **reward**, and the world moves to a new state according to fixed probabilities. [[Reinforcement_Learning]] is the study of how to act well in an MDP when you do not know those probabilities in advance.

> **Etymology.** Named after **Andrey Markov** (1856–1922), the Russian mathematician who studied chains of random events where the next event depends only on the current one: the *Markov property*. Richard Bellman added *decisions* and rewards in his 1957 work on dynamic programming, and Ronald Howard's 1960 book *Dynamic Programming and Markov Processes* gave the MDP its modern form.

## The five ingredients

An MDP is the tuple $(\mathcal{S}, \mathcal{A}, P, R, \gamma)$:

| Symbol | Name | Robot example |
|---|---|---|
| $\mathcal{S}$ | States | Joint angles and velocities, object pose |
| $\mathcal{A}$ | Actions | Joint position targets |
| $P(s' \mid s, a)$ | Transition probabilities | Physics, plus noise and unmodelled effects |
| $R(s, a)$ | Reward | +1 when the object is lifted, minus a small cost for energy |
| $\gamma \in [0, 1)$ | Discount factor | 0.99: future rewards count slightly less than present ones |

**The Markov property** says the future depends only on the present state and action, not on how you got there:

$$P(s_{t+1} \mid s_t, a_t, s_{t-1}, a_{t-1}, \dots) = P(s_{t+1} \mid s_t, a_t)$$

That is a modelling choice, not a law of nature. Joint angles alone are not Markov (you also need velocities); add them and the system becomes Markov enough to work with.

## Value, and the Bellman equation

A [[Policy]] $\pi$ chooses actions. Its **return** from time $t$ is the discounted sum of rewards $G_t = \sum_{k=0}^{\infty} \gamma^k r_{t+k}$. Two functions measure how good a policy is:

- **State value** $V^\pi(s) = \mathbb{E}_\pi[G_t \mid s_t = s]$: expected return starting in $s$.
- **Action value** $Q^\pi(s, a) = \mathbb{E}_\pi[G_t \mid s_t = s, a_t = a]$: expected return after taking $a$ in $s$, then following $\pi$.

The **Bellman equation** expresses value recursively: what a state is worth equals what you get now plus the discounted worth of where you end up.

$$V^\pi(s) = \sum_a \pi(a \mid s) \Big[ R(s,a) + \gamma \sum_{s'} P(s' \mid s, a)\, V^\pi(s') \Big]$$

For the optimal policy, the average over actions becomes a maximum:

$$V^*(s) = \max_a \Big[ R(s,a) + \gamma \sum_{s'} P(s' \mid s,a)\, V^*(s') \Big]$$

Almost every reinforcement learning algorithm is a way of solving this equation approximately: **value iteration** applies it repeatedly; **Q-learning** and DQN learn $Q^*$ from samples; **policy-gradient** methods such as PPO improve $\pi$ directly using value estimates as a baseline.

## Worked example: a three-cell corridor

A robot in a corridor of three cells, `[A][B][Goal]`. Actions: *left* or *right*. Reaching *Goal* gives reward +1 and ends the episode; every other step gives 0. Let $\gamma = 0.9$ and suppose moves always succeed.

- From **B**, *right* reaches the goal: $V^*(B) = 1$.
- From **A**, *right* reaches B: $V^*(A) = 0 + 0.9 \cdot V^*(B) = 0.9$.

Now make the floor slippery: *right* succeeds with probability 0.8 and leaves the robot in place otherwise. Then $V^*(B) = 0.8 \cdot 1 + 0.2 \cdot 0.9\, V^*(B)$, so $V^*(B) = 0.8 / 0.82 \approx 0.976$. The discount makes the robot prefer reaching the goal *sooner*, and uncertainty lowers the value of every state. Real robot problems are this, with millions of continuous states.

## POMDPs: what robots actually face

Real robots do not observe the state; they get noisy, partial **observations** $o \sim O(o \mid s)$. The honest model is a **partially observable MDP (POMDP)**. Solving POMDPs exactly is intractable, so practice uses three shortcuts:

1. **Estimate the state** with a filter ([[Kalman_Filter]], [[SLAM]]) and act as if the estimate were true.
2. **Feed the policy a history** of observations, or give it memory (recurrent or transformer networks).
3. **Train with privileged information** in simulation (the true state), then distil into a policy that only sees observations (see [[Reinforcement_Learning_for_Robots]]).

## Continuous time and control

Control theorists meet the same object in continuous time. The Bellman equation becomes the [[Hamilton-Jacobi-Bellman_Equation]], and the linear-quadratic case has a closed-form answer: the [[Linear_Quadratic_Regulator_(LQR)|LQR]]. [[Optimal_Control]] and reinforcement learning are two traditions solving the same problem: one usually knows the model, the other usually learns from samples.

## Where MDPs show up in robotics (as of November 2026)

- **Locomotion and manipulation by RL in simulation.** The MDP's transition function *is* the simulator; GPU simulators such as Isaac Lab 3.0 and MuJoCo Playground run thousands of copies at once.
- **Fine-tuning large policies with RL.** Pretrained [[Vision-Language-Action_Models]] are increasingly refined with reward signals after imitation pretraining; the MDP framing is how that refinement is defined.
- **Learned models of $P$.** [[World_Models]] learn the transition function itself from video and interaction data, so an agent can plan or train inside an imagined MDP.

## Reading list

- Sutton and Barto, *Reinforcement Learning: An Introduction* (2nd ed.), chapters 3–4: MDPs, value functions, dynamic programming. Free online.
- Mnih et al. 2013: the first deep network to learn Q-values from pixels.
- Schulman et al. 2017: PPO, the policy-gradient method used in most robot RL today.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Markov_Decision_Process]])
```
