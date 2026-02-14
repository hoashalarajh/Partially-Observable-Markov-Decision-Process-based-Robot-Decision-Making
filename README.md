# POMDP-Based Socially Compliant Mobile Robot Navigation

This repository implements a **Partially Observable Markov Decision Process (POMDP)** model for socially compliant robot navigation in a simplified crowd environment. The project is written in **Julia** and uses the **POMDPs.jl** ecosystem with an online Monte Carlo Tree Search based planning approach.

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

#### State transition diagram is visualized as below:
```mermaid
graph TD
    subgraph Environment ["Environment (World)"]
        direction TB
        s_curr((Current State s)) -->|Action a| s_next((Next State s'))
        s_next -->|Observation Function O| o[Observation o]
        s_next -->|Reward Function R| r[Reward r]
    end

    subgraph Agent ["Agent (Robot)"]
        direction TB
        %% The Planner (The Decision Maker)
        b_curr[(Current Belief b)] -->|Plan| Planner
        r -->|Feedback| Planner["Planner (POMCPOW)"]
        Planner --> a[Action a]
        o --> |Sense| Updater{Belief Updater}
        a --> Updater
        b_curr --> Updater
        Updater -->|Bayes Update| b_next[(Next Belief b')]
    end

    %% Connections between Agent and Environment
    a -->|Execute| s_curr

    %% The Loop Connection
    b_next -.-> b_curr
    s_next -.-> s_curr

    %% Styling
    %% I added 'color:black' (text) and 'stroke:black' (border) to Environment
    style Environment fill:#4e4d82,stroke:black,stroke-width:2px,color:black
    style Agent fill:##824d67,stroke:black,stroke-width:2px,color:black
    style Updater fill:#ffd54f,stroke:black,stroke-width:2px,color:black
```

#### Functional block diagram is visualized as below:

```mermaid
stateDiagram
  direction TB
  state S {
    direction TB
    Robot_Pos
    Crowd_State
  }
  state S_prime {
    direction TB
    Robot_Pos_New
    state Crowd_State_New {
      direction LR
      state ND {
        direction TB
        (Far)
        (Near)
      }
      state NF {
        direction TB
        (Aligned)
        (Opposing)
      }
      state NDen {
        direction TB
        (Low)
        (High)
      }
    }
  }
  S --> S_prime:a = FollowFlow (rx' = rx + 1)
  S --> S_prime:a = Overtake (ry' = ry + 1)
  S --> S_prime:a = FindGap (ry' = ry - 1)
  S --> S_prime:a = Yield or Wait (No change in position)
  S:Current State (s)
  Robot_Pos:(rx, ry)
  Crowd_State:(dist, flow, density)
  S_prime:Next State (s')
  Robot_Pos_New:(rx', ry')
  Crowd_State_New:(dist', flow', density')
  ND:new_dist'
  (Far):P = 0.6
  (Near):P = 0.4
  NF:new_flow'
  (Aligned):P = 0.5
  (Opposing):P = 0.5
  NDen:new_density'
  (Low):P = 0.7
  (High):P = 0.3
  note left of S 
  Robot movement is deterministic.
        Positions are bounded within [1, grid_size].
  end note
  note right of S_prime 
  Crowd dynamics are stochastic
        and independent of the previous state or action.
  end note
```

---

#### Following visualizations show the results of the 1000-step simulation run:

<img width="1500" height="1000" alt="pomdp_simulation_analysis" src="https://github.com/user-attachments/assets/8d47631f-c14f-4e96-a1d7-457281923e7d" />
<img width="1500" height="1000" alt="critical_situation_analysis" src="https://github.com/user-attachments/assets/ad05fcec-5c7a-4ff1-acd7-a08141d802ae" />

**Lesson learned and possible improvement:** Here the reward structure is based on social rules considering various social situations. The above results may make sense at first glance, but this still remains incomplete becuase all of the possible social rules cannot be encoded into a single rule base system. Plus, I tried reward structures utilizing a quadratic and polynomial function - but still the results were not promising. So, we need a more complicated reward structure that could capture most of  the social nuances. 

