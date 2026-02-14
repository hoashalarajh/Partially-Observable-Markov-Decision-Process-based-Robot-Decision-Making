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


```mermaid
stateDiagram-v2
    direction TB

    %% Define the states
    state "Current State (s)" as S {
        direction TB
        Robot_Pos: (rx, ry)
        Crowd_State: (dist, flow, density)
    }

    state "Next State (s')" as S_prime {
        direction TB
        Robot_Pos_New: (rx', ry')
        Crowd_State_New: (dist', flow', density')
    }

    %% Part 1: Deterministic Robot Transitions based on Action (a)
    note left of S
        Robot movement is deterministic.
        Positions are bounded within [1, grid_size].
    end note
    
    S --> S_prime : a = FollowFlow (rx' = rx + 1)
    S --> S_prime : a = Overtake (ry' = ry + 1)
    S --> S_prime : a = FindGap (ry' = ry - 1)
    S --> S_prime : a = Yield or Wait (No change in position)

    %% Part 2: Stochastic Crowd Transition (Independent of Action)
    note right of S_prime
        Crowd dynamics are stochastic
        and independent of the previous state or action.
    end note

    state Crowd_State_New {
        direction TB
        state "new_dist'" as ND {
            1 (Far) : P = 0.6
            2 (Near) : P = 0.4
        }
        state "new_flow'" as NF {
            1 (Aligned) : P = 0.5
            2 (Opposing) : P = 0.5
        }
        state "new_density'" as NDen {
            1 (Low) : P = 0.7
            2 (High) : P = 0.3
        }
    }
```

---

#### Following visualizations show the results of the 1000-step simulation run:

<img width="1500" height="1000" alt="pomdp_simulation_analysis" src="https://github.com/user-attachments/assets/8d47631f-c14f-4e96-a1d7-457281923e7d" />
<img width="1500" height="1000" alt="critical_situation_analysis" src="https://github.com/user-attachments/assets/ad05fcec-5c7a-4ff1-acd7-a08141d802ae" />

Here the reward structure is based on social rules considering various social situations. But this remains still incomplete becuase all the possible social rules cannot be encoded into a single rule base system. Plus, I tried a reward structure utilizing a quadratic function - still the results are not perfect with respect to a human expectation. So, we need a more complicated reward structure that could capture most of  the social nuances. 

