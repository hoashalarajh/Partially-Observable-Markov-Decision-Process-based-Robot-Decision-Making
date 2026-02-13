# POMDP-Based Socially Compliant Robot Navigation

This repository implements a **Partially Observable Markov Decision Process (POMDP)** model for socially compliant robot navigation in a simplified crowd environment. The project is written in **Julia** and uses the **POMDPs.jl** ecosystem with an online Monte Carlo planning approach.

The goal is to demonstrate how uncertainty in crowd perception and social context can be incorporated into robot navigation decision-making.

---

## Overview

The robot navigates a discrete 2D grid while interacting with a crowd characterized by:
- **Crowd distance** (far / near)
- **Crowd flow direction** (aligned / opposing)
- **Crowd density** (low / high)

These crowd properties are **partially observable** and perceived through noisy observations. At each step, the robot selects a socially relevant action and receives a reward that encourages safe and socially compliant behavior.

Planning is performed online using **POMCPOW**, a Monte Carlo Tree Search (MCTS) algorithm designed for large POMDPs.

---

## State Space

The POMDP state consists of:

- Robot position: `(rx, ry)`
- Crowd distance: `1 = far`, `2 = near`
- Crowd flow: `1 = aligned`, `2 = opposing`
- Crowd density: `1 = low`, `2 = high`

```text
CrowdState = (robot_x, robot_y, distance, flow, density)
```

---

The visualization shows the results of the 1000-step simulation run

<img width="1500" height="1000" alt="pomdp_simulation_analysis" src="https://github.com/user-attachments/assets/f0eba3d6-603e-4ca9-bf27-4ad9f3da5495" />
<img width="1500" height="1000" alt="critical_situation_analysis" src="https://github.com/user-attachments/assets/0afd4c86-33f9-4249-823d-fea95430bb34" />
