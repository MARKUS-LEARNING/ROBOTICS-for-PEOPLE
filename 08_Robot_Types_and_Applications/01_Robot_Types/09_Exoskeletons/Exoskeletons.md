---
title: Exoskeletons
description: Overview of robotic exoskeletons, wearable devices designed to augment, assist, or restore human physical capabilities.
tags:
  - robot-types
  - exoskeleton
  - wearable-robotics
  - HRI
  - assistive-robotics
  - rehabilitation
  - augmentation
  - biomechanics
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-04-29
permalink: /exoskeletons/
related:
  - "[[Robots]]"
  - "[[Wearable_Robotics]]"
  - "[[Human-Robot_Interaction]]"
  - "[[Actuator]]"
  - "[[Sensor]]"
  - "[[Control_Systems]]"
  - "[[Biomechanics]]"
  - "[[Rehabilitation_Robotics]]"
  - "[[Robot_Types_and_Applications]]"
---

# Exoskeletons

**Exoskeletons** (in the context of robotics) are wearable mechanical or electromechanical structures designed to be worn by a person, typically mirroring the user's limb structure. Powered exoskeletons utilize [[Actuator|actuators]] to apply forces and torques aligned with the user's joints, aiming to augment strength/endurance, assist movement, or restore lost function. They represent a significant area within [[Wearable Robotics]] and [[Human-Robot Interaction (HRI)]].

*(Note: This differs from passive exoskeletons, which provide support via springs or elastic elements without active power.)*

---

## Purpose

Powered robotic exoskeletons are designed for several primary purposes:

* **Augmentation:** Enhance the physical capabilities of able-bodied individuals, increasing strength or endurance for tasks like heavy lifting (industrial settings), load carrying (military), or reducing fatigue.
* **Assistance:** Provide powered support for individuals with weakness or partial paralysis (e.g., due to aging, muscle disease) to perform daily activities or maintain mobility.
* **[[Rehabilitation Robotics|Rehabilitation]]:** Used in therapeutic settings to help patients with neurological injuries (e.g., stroke, spinal cord injury) regain motor function through repetitive, guided movements and task-oriented practice.

---

## Classification / Types

Exoskeletons can be classified based on the body parts they interact with:

* **Lower-limb Exoskeletons:** Cover the legs and potentially the hip/pelvis. Designed for walking assistance, gait rehabilitation, load carrying, or strength augmentation for lifting.
* **Upper-limb Exoskeletons:** Cover the arm(s) and potentially the shoulder/torso. Used for arm rehabilitation, assisting with tasks requiring arm strength or endurance (e.g., tool holding in manufacturing), or providing haptic feedback.
* **Full-body Exoskeletons:** Integrate actuation across both upper and lower limbs and the torso. Aim for significant strength/endurance augmentation (e.g., conceptual military suits) or comprehensive mobility assistance.

---

## Key Technologies

Developing effective exoskeletons involves integrating several technologies:

* **Mechanical Design:** Structures must be lightweight yet strong, with [[Joint_Kinematics|joints]] closely aligned with human anatomical joints to allow natural movement. Comfortable and secure human attachment interfaces are crucial.
* **[[Actuator|Actuation]]:** Providing sufficient power in a compact, lightweight form is a major challenge. Common approaches include:
    * [[Electric_Motor|Electric Motors]] with [[Transmission|transmissions]] ([[Gearbox|gears]], ball screws). Series Elastic Actuators (SEAs) are often used to introduce compliance and allow for better force control.
    * [[Hydraulic_Systems|Hydraulics]]: Offer high [[Power_Density]] for strength augmentation but require a bulky power unit.
    * [[Pneumatic Actuators]]: Can be lightweight (e.g., McKibben artificial muscles) but require a compressed air source and can be harder to control precisely.
* **[[Sensor|Sensing]]:** Critical for monitoring both the human user and the exoskeleton state:
    * **[[IMU_Sensors]]:** Measure limb segment orientation and angular velocity.
    * **Joint Angle Sensors:** Encoders or potentiometers measure robot joint positions.
    * **Force/Torque Sensors:** Measure interaction forces between the user and the exoskeleton, or between the exoskeleton and the environment (e.g., ground reaction forces).
    * **Physiological Sensors:**
        * *EMG (Electromyography):* Detects electrical signals from muscle activation to infer user intent.
        * *EEG (Electroencephalography):* Detects brain signals, sometimes explored for direct neural control (BCI).
* **[[Control Systems|Control Strategies]]:** The core challenge is coordinating the exoskeleton's actions with the user's intent and movements seamlessly and safely. Common strategies include:
    * **Trajectory Control:** Guiding the user's limbs along predefined paths (common in rehabilitation).
    * **Assist-as-Needed:** Monitoring user effort and providing assistance only when necessary or proportional to weakness.
    * **Intent Detection:** Using EMG, EEG, force sensors, or kinematic cues (early motion detection) to predict the user's desired movement and initiate exoskeleton assistance proactively.
    * **Impedance/Admittance Control:** Regulating the dynamic relationship (stiffness, damping, inertia) between the exoskeleton and the user, aiming for compliant and natural interaction. See [[Force Control]].

---

## Challenges

Despite significant progress, widespread adoption of exoskeletons faces several hurdles:

* **Human-Robot Interface:** Designing intuitive cognitive interfaces (intent detection) and comfortable, non-restrictive physical attachments is difficult. Joint misalignment between human and robot can cause discomfort or injury.
* **Weight and Bulk:** Current systems are often heavy and cumbersome, limiting user acceptance and practicality.
* **Power Source:** Providing sufficient onboard power for extended, untethered operation remains a major challenge, especially for high-power augmentation tasks.
* **Control Robustness:** Controllers must reliably interpret user intent and adapt to the variability and unpredictability of human movement and interaction with complex environments.
* **[[Safety]]:** Ensuring the system cannot overpower the user or cause injury, especially during sensor failures or unexpected situations. Requires robust fault detection and safe failure modes.
* **Cost:** High development and component costs currently limit accessibility for many potential users.
* **User Adaptation & Learning:** Users often require training and time to adapt to working effectively with an exoskeleton.

Exoskeletons hold immense potential for improving quality of life through rehabilitation and assistance, and for enhancing human capabilities in demanding physical tasks, making them a key area of ongoing robotics research and development.

---

## Dataview Plugin Features

To integrate this entry with the Dataview plugin, you can use the following queries to dynamically generate lists and tables:

### List of Related Concepts

```dataview
LIST FROM #robotics OR #automation WHERE contains(file.outlinks, [[Robots]])