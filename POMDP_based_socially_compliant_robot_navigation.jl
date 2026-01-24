# installing and importing required packages
#import Pkg; Pkg.add("POMDPs")
#import Pkg; Pkg.add("Distributions")
#import Pkg; Pkg.add("POMCPOW")
#import Pkg; Pkg.add("BeliefUpdaters")
#import Pkg; Pkg.add("POMDPTools")
#import Pkg; Pkg.add("StatsBase")

# description of actions:
# FollowFlow: Move with dominant pedestrian flow
# FindGap: Search for a collision-free opening
# Yield: Slow down / wait to let others pass
#=
======================================================
| Action       | Meaning (social navigation)         |
| ------------ | ----------------------------------- |
| `FollowFlow` | Move with dominant pedestrian flow  |
| `FindGap`    | Search for a collision-free opening |
| `Yield`      | Slow down / wait to let others pass |
======================================================
=#

# crowd observation descriptions
#=
CrowdObs(2, 1, 1)
Indicates:
density = 2
flow = 1
attention = 1

=================================================================
| Field           | Value   | Example meaning                   |
| --------------- | ------- | --------------------------------- |
| `density = 2`   | medium  | moderate crowd density            |
| `flow = 1`      | aligned | pedestrians moving same direction |
| `attention = 1` | aware   | people have noticed the robot     |
=================================================================
=#


using POMDPs
using Distributions
using Random

using POMCPOW
using BeliefUpdaters

using POMDPTools
using StatsBase


struct CrowdState
    rx::Int        # robot x cell
    ry::Int        # robot y cell
    crowd_dist::Int  # 1=far, 2=near
    flow::Int        # 1=aligned, 2=opposing
    density::Int     # 1=low, 2=high
end
@enum Action begin
    FollowFlow
    Yield
    Overtake
    FindGap
    Wait
end
struct CrowdObs
    crowd_dist::Int
    flow::Int
    density::Int
end

struct SocialNavPOMDP <: POMDP{CrowdState,Action,CrowdObs}
    grid_size::Int
    discount::Float64
end

POMDPs.discount(p::SocialNavPOMDP) = p.discount

# This tells the solver which actions are available for the POMDP
function POMDPs.actions(p::SocialNavPOMDP)
    return [FollowFlow, Yield, Overtake, FindGap, Wait]
end

function POMDPs.transition(p::SocialNavPOMDP, s::CrowdState, a::Action)
    rx, ry = s.rx, s.ry

    if a == FollowFlow
        rx += 1
    elseif a == Overtake
        ry += 1
    elseif a == Yield || a == Wait
        # No change
    elseif a == FindGap
        ry -= 1
    end

    rx = clamp(rx, 1, p.grid_size)
    ry = clamp(ry, 1, p.grid_size)

    # Use sample() for weighted transitions
    new_dist = sample([1, 2], Weights([0.6, 0.4]))
    new_flow = rand([1, 2])
    new_density = sample([1, 2], Weights([0.7, 0.3]))

    return Deterministic(CrowdState(rx, ry, new_dist, new_flow, new_density))
end

function POMDPs.observation(p::SocialNavPOMDP, a::Action, sp::CrowdState)
    noise(p) = rand() < 0.85 ? p : rand([1,2])
    return Deterministic(CrowdObs(
        noise(sp.crowd_dist),
        noise(sp.flow),
        noise(sp.density)
    ))
end


function POMDPs.reward(p::SocialNavPOMDP, s::CrowdState, a::Action)
    r = -1.0  # time penalty

    if s.crowd_dist == 2 && a == Overtake
        r -= 5.0   # socially bad
    end
    if a == FollowFlow && s.flow == 1
        r += 3.0
    end
    if a == Yield && s.density == 2
        r += 2.0
    end
    return r
end

pomdp = SocialNavPOMDP(10, 0.95)

solver = POMCPOWSolver(
    max_depth = 15,
    tree_queries = 1000,
    rng = MersenneTwister(1)
)

planner = solve(solver, pomdp)

function POMDPs.initialstate(p::SocialNavPOMDP)
    rx = 1
    ry = 1
    
    # Use sample() instead of rand() for Weights
    crowd_dist = sample([1, 2], Weights([0.6, 0.4]))
    flow       = rand([1, 2]) # rand is ok here as there are no weights
    density    = sample([1, 2], Weights([0.7, 0.3]))

    return Deterministic(CrowdState(rx, ry, crowd_dist, flow, density))
end


rng = MersenneTwister(1)

pomdp = SocialNavPOMDP(10, 0.95)

solver = POMCPOWSolver(
    max_depth = 15,
    tree_queries = 1000,
    rng = rng
)

planner = solve(solver, pomdp)

for (s, a, o, r, b) in stepthrough(pomdp, planner, "s,a,o,r,b"; max_steps=1000, rng=rng)
    println("action=$a | obs=$o | reward=$r")
end
