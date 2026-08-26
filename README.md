# FxT-STA for 5-DOF Upper-Limb Exoskeleton

This repository contains a MATLAB/Simulink implementation of the **Continuous Fixed-Time Super-Twisting Algorithm (FxT-STA)** designed for robust, chattering-free trajectory tracking of a 5-DOF upper-limb rehabilitation exoskeleton robot. 

## Overview

Controlling upper-limb exoskeletons presents significant challenges due to highly nonlinear coupled dynamics, parameter uncertainties, and varying human-robot interaction (HRI) forces. While traditional Sliding Mode Control (SMC) offers robustness, it suffers from high-frequency chattering that can degrade user comfort and damage actuators. Furthermore, standard finite-time controllers have settling times that depend heavily on initial conditions.

This implementation addresses these issues by utilizing a novel bi-limit homogeneous FxT-STA. It completely replaces discontinuous signum functions with continuous power functions, structurally eliminating chattering while guaranteeing **practical fixed-time convergence** (a settling time bound independent of initial states).

## Key Features

* **Chattering Elimination:** Replaces discontinuous `sign()` switching logic with continuous bi-limit homogeneous power functions ($\phi_1$ and $\phi_2$).
* **Fixed-Time Stability:** Guarantees convergence to a bounded residual set within a fixed maximum time ($T_{max}$), regardless of the initial tracking error.
* **Singularity-Free Operation:** Incorporates a Non-singular Fast Terminal Sliding Manifold (NFTSM) using strictly positive fractional powers to avoid control singularities near the equilibrium.
* **Decentralized Architecture:** Implements a decentralized control law where inter-joint couplings and HRI forces are treated as lumped disturbances rejected by the robust FxT-STA action.

## Implementation Details

The core control logic is implemented in the `SMC_controller.m` file, which is intended to be used as a **MATLAB Function block** within a Simulink environment. 

* **Dynamics:** The script explicitly computes the 5-DOF Euler-Lagrange dynamics (Inertia matrix $M(q)$, Coriolis matrix $C(q,\dot{q})$, and Gravity vector $G(q)$).
* **Control Law:** The controller calculates the equivalent control derivative and the continuous super-twisting switching control derivative to output the joint control signals ($\dot{u}_1, \dots, \dot{u}_5$).
* **Parameters:** The controller uses bi-limit homogeneity degrees ($\mu = 0.5, \nu = 0.6$) and NFTSM parameters ($\gamma = 0.8$) tuned for optimal performance and actuator limits.

## Reference

This code implements the methodology detailed in the paper: *"Bi-Limit Homogeneous Fixed-Time Super-Twisting Control for Upper-Limb Exoskeleton robot"*. If you use this code in your research, please cite the original work.
