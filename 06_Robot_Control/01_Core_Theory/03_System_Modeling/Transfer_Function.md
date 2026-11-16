---
title: Transfer Function
description: "A transfer function G(s) = Y(s)/U(s) describes a linear system's input–output behaviour in the Laplace domain. Covers how it is derived, poles and zeros, a worked mass-spring-damper example including its frequency response at resonance (computed), block-diagram algebra for feedback loops, the link to Bode plots and margins, and where transfer functions are still used in robotics in 2026."
aliases:
  - Transfer Functions
  - Laplace Transform
tags:
  - control
  - mathematics
  - stability
type: concept
level: intermediate
subfield:
  - control
  - mathematics
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Control_Theory]]"
  - "[[Calculus_for_Robotics]]"
related:
  - "[[State_Space_Representation]]"
  - "[[PID_Control]]"
  - "[[Control_Theory_and_Principles]]"
key_takeaways:
  - "A transfer function is the ratio of output to input in the Laplace domain for a linear, time-invariant system with zero initial conditions; its denominator's roots are the poles."
  - "Evaluating G(jω) gives the frequency response: the mass-spring-damper 1/(s² + 1.2s + 4) amplifies a 2 rad/s input by 0.417 (versus 0.25 at DC) and lags it by 90°."
  - "Transfer functions remain the everyday tool for single-loop tuning (motor and joint loops, filters, drone rate loops); multi-input robot dynamics use state space."
sources:
  - "Åström, K. J. and Murray, R. M. (2021). Feedback Systems, 2nd edition, chapter 9: Transfer Functions. https://fbswiki.org/wiki/index.php/Feedback_Systems:_An_Introduction_for_Scientists_and_Engineers"
  - "Åström, K. J. and Murray, R. M. (2020). Feedback Systems, public PDF edition. https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-public_24Jul2020.pdf"
  - "Lynch, K. M. and Park, F. C. (2017). Modern Robotics, chapter 11: Robot Control. https://hades.mech.northwestern.edu/index.php/Modern_Robotics"
---

# Transfer Function

A **transfer function** describes how a linear system turns inputs into outputs. It is the ratio of the output's Laplace transform to the input's, assuming the system starts at rest:

$$G(s) = \frac{Y(s)}{U(s)}$$

The Laplace transform turns differential equations into algebra: differentiation becomes multiplication by $s$. Combining systems in series, parallel and feedback then becomes multiplying and dividing polynomials.

> **Etymology.** Pierre-Simon Laplace introduced the transform in the late 18th century. Oliver Heaviside's operational calculus (1880s–90s) and the work of Nyquist (1932) and Bode (1940s) made transfer functions the basis of classical control.

## Deriving one

For a mass-spring-damper, $m\ddot y + b\dot y + k y = u$. Take Laplace transforms (zero initial conditions):

$$(ms^2 + bs + k)\,Y(s) = U(s) \quad\Rightarrow\quad G(s) = \frac{1}{ms^2 + bs + k}$$

- **Poles** (roots of the denominator) set stability and the natural response.
- **Zeros** (roots of the numerator) shape the response; zeros in the right half-plane cause an initial move in the wrong direction.

## Worked example

With $m = 1$, $b = 1.2$, $k = 4$: $G(s) = 1/(s^2 + 1.2s + 4)$.

- **Poles**: $s = -0.6 \pm 1.908j$, the same as the eigenvalues of the state-space model (see [[State_Space_Representation]]).
- **Static gain**: $G(0) = 1/4 = 0.25$, so a constant 1 N force moves the mass 0.25 m.
- **At resonance**, $\omega = 2$ rad/s: $G(2j) = 1/(4 - 4 + 2.4j) = 1/(2.4j)$, so the gain is $1/2.4 = 0.417$ (1.7× the static gain) and the phase is −90°.

A lightly damped joint or flexible arm link behaves like this. Excite it near resonance and it swings far more than the static gain suggests, which is why controllers and trajectories avoid exciting structural resonances (input shaping, notch filters).

## Feedback in transfer functions

For plant $P(s)$ and controller $C(s)$ in a unity-feedback loop:

$$T(s) = \frac{PC}{1 + PC}\ \ (\text{reference} \to \text{output}), \qquad S(s) = \frac{1}{1 + PC}\ \ (\text{disturbance} \to \text{output})$$

These two functions summarise what feedback does (see [[Control_Theory_and_Principles]]). A PID controller is $C(s) = K_p + K_i/s + K_d s$ (see [[PID_Control]]).

## Frequency response, Bode plots and margins

Setting $s = j\omega$ gives the **frequency response**: its magnitude and phase plotted against frequency form the **Bode plot**. Gain and phase margins read from it say how much extra gain or delay the loop tolerates before going unstable.

## Use in robotics (2026)

- **Tuning single loops**: motor current loops, joint velocity loops, drone rate loops, and filters (low-pass, notch) are designed and checked with transfer functions and Bode plots.
- **System identification**: frequency sweeps on a real joint measure its transfer function, revealing resonances and delays before tuning.
- **Limits**: transfer functions describe linear, time-invariant, single-input single-output systems. Coupled, nonlinear robot dynamics, MPC and learned controllers use state space instead.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Transfer_Function]])
```
