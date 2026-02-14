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

```mermaid
flowchart TD
    A[Start] --> B{Process};
    B --> C{Decision?};
    C --> D[End];
    C --> E[Go back];
    E --> B;
```


```text
CrowdState = (robot_x, robot_y, distance, flow, density)
```

---

#### The visualization shows the results of the 1000-step simulation run

<img width="1500" height="1000" alt="pomdp_simulation_analysis" src="https://github.com/user-attachments/assets/8d47631f-c14f-4e96-a1d7-457281923e7d" />
<img width="1500" height="1000" alt="critical_situation_analysis" src="https://github.com/user-attachments/assets/ad05fcec-5c7a-4ff1-acd7-a08141d802ae" />

Here the reward structure is based on social rules considering various social situations. But this remains still incomplete becuase all the possible social rules cannot be encoded into a single rule base system. Plus, I tried a reward structure utilizing a quadratic function - still the results are not perfect with respect to a human expectation. So, we need a more complicated reward structure that could capture most of  the social nuances. 

