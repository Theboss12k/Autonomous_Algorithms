# Hierarchical Event-Triggered Multi-Agent Trajectory Coordination

A research implementation of a hierarchical multi-agent trajectory coordination algorithm combining **localized A* replanning, event-triggered conflict resolution, priority-weighted lateral evasion, and iterative geometric constraint correction**.

The algorithm is designed for autonomous vehicles operating in a shared 2D/3D environment where multiple agents may encounter dynamic conflicts while following independent missions.

---

## Abstract

Multi-agent trajectory planning requires balancing global path quality, computational cost, and rapid response to dynamic interactions.

A straightforward approach is to repeatedly invoke a global planner whenever the environment changes. While effective in relatively small environments, repeatedly solving large planning problems can become computationally expensive as the number of agents increases.

This work investigates a hierarchical alternative.

The proposed algorithm separates trajectory coordination into progressively more localized stages:

```text
                    Vehicle Intent
                         │
                         ▼
                Localized A* Planning
                         │
                         ▼
               Conflict Detection
                         │
                  Conflict Event
                         │
                         ▼
             Priority-Weighted Evasion
                         │
                         ▼
             Geometric Constraint
                  Resolution
                         │
                         ▼
                  State Commit
                         │
                         ▼
                  Replanning
```

The planner therefore does not treat every interaction as a complete global planning problem.

Instead, computational effort is escalated only when the current coordination layer cannot adequately resolve the interaction.

---

# Algorithm Overview

The proposed framework consists of four primary mechanisms.

### 1. Localized A* Replanning

A* is used as the deliberative path planner.

Rather than constructing a dynamic planning problem containing every vehicle in the environment, each vehicle considers only agents within a configurable local planning frame.

For vehicle `i`:

```text
Nᵢ = { j | ||xᵢ - xⱼ|| ≤ R_frame }
```

where:

* `xᵢ` is the position of vehicle `i`
* `xⱼ` is the position of another vehicle
* `R_frame` is the local planning radius

The A* search therefore operates on:

```text
Static Obstacles
       +
Nearby Dynamic Agents
       ↓
Localized Planning Problem
```

This provides a tunable trade-off between planning scope and computational effort.

---

### 2. Event-Triggered Conflict Resolution

Vehicles normally follow their planned trajectories.

A reactive resolution process is activated only when a safety constraint is approached or violated.

For two vehicles:

```text
d_required =
    marginᵢ
  + marginⱼ
  + safety_buffer
```

A conflict is detected when:

```text
||xᵢ - xⱼ|| < d_required
```

This event triggers the local avoidance mechanism.

The important distinction is that conflict resolution is **event-driven**, rather than requiring complete replanning for every interaction.

---

### 3. Priority-Weighted Lateral Evasion

When two vehicles enter a conflict state, lateral maneuver candidates are generated relative to the vehicle's current direction of travel.

```text
                    Left
                     ↖
                      \
                       \
                        → Heading
                       /
                      /
                     ↙
                   Right
```

The candidate maneuver is selected using a combination of:

* conflict geometry,
* opposing-agent direction,
* goal direction,
* and nearby obstacle influence.

Vehicle priority and velocity influence the magnitude of the maneuver through a priority-weighted mobility coefficient:

```text
Wᵢ = Pᵢ × Vᵢ
```

where:

* `Pᵢ` = vehicle priority
* `Vᵢ` = vehicle velocity
* `Wᵢ` = priority-weighted mobility coefficient

This allows the interaction between agents to be asymmetric.

Two vehicles with identical geometry but substantially different priorities do not necessarily receive identical maneuver responses.

---

### 4. Iterative Geometric Constraint Resolution

Reactive maneuvers may still produce small geometric overlaps due to discrete simulation steps or simultaneous vehicle interactions.

An iterative position-based constraint-resolution stage is therefore applied after the proposed motion.

Conceptually:

```text
Proposed Positions
       │
       ▼
Constraint Detection
       │
       ▼
Calculate Overlap
       │
       ▼
Priority-Weighted Correction
       │
       ▼
Update Positions
       │
       ▼
Repeat
```

The correction can operate locally around active conflicts or across the complete active fleet.

This stage is intended as a **constraint correction mechanism**, not as a replacement for trajectory planning.

---

# Hierarchical Design Principle

The central design principle is:

> **Use the least computationally expensive mechanism capable of resolving the current situation.**

The resulting hierarchy can be represented as:

```text
┌─────────────────────────────────┐
│       Deliberative Layer        │
│                                 │
│       Localized A*              │
│       Higher computation        │
└────────────────┬────────────────┘
                 │
          Conflict Event
                 ▼
┌─────────────────────────────────┐
│         Reactive Layer          │
│                                 │
│  Priority-Weighted Evasion      │
│       Lower computation         │
└────────────────┬────────────────┘
                 │
          Residual Constraint
                 ▼
┌─────────────────────────────────┐
│       Constraint Layer          │
│                                 │
│   Iterative Position Correction │
│       Local geometric fix       │
└─────────────────────────────────┘
```

This architecture is intended to reduce unnecessary invocation of expensive planning operations while retaining a mechanism for rapid local response.

---

# Vehicle Model

Each agent is represented using:

* unique identifier
* start position
* destination
* velocity
* priority
* safety margin
* activation time
* current position
* proposed position
* previous position
* trajectory state
* waypoint sequence
* original trajectory
* resolved trajectory
* timing information

The primary vehicle states are:

```text
NORMAL
   │
   │ Conflict
   ▼
DEFLECTING
   │
   │ Maneuver completed / progress failure
   ▼
NORMAL
```

If a reactive maneuver becomes ineffective, the vehicle can request a new local A* solution.

---

# Environment Representation

The current implementation supports cylindrical volumetric obstacles represented by:

```text
(x, y, radius, z_min, z_max)
```

The planner can operate in either:

```text
2D
```

or:

```text
3D
```

mode.

In 3D mode, the search includes vertical motion in addition to horizontal movement.

---

# Safety Model

For every pair of vehicles, a minimum separation constraint is defined.

```text
||xᵢ - xⱼ|| ≥
    mᵢ + mⱼ + ε
```

where:

* `mᵢ` = safety margin of vehicle `i`
* `mⱼ` = safety margin of vehicle `j`
* `ε` = additional safety buffer

When the constraint is not satisfied, the conflict-resolution process is activated.

---

# Local Planning

The local planning frame is one of the main configurable parameters.

A small planning radius produces:

```text
Smaller Search Space
       ↓
Lower Computational Cost
       ↓
Less Global Traffic Information
```

while a larger radius produces:

```text
Larger Search Space
       ↓
Higher Computational Cost
       ↓
More Traffic Information
```

This creates an experimentally measurable relationship between locality, computational cost, and coordination performance.

---

# Adversarial Evaluation

The implementation contains an adversarial scenario suite designed to deliberately stress the coordination mechanism.

Current scenarios include:

### Head-On Encounters

Two vehicles approach one another on opposing trajectories.

### Extreme Priority Asymmetry

Vehicles with substantially different priority/mobility weights interact.

### Multi-Agent Intersections

Multiple agents converge on a common region.

### Diagonal Crossings

Agents intersect along non-axis-aligned trajectories.

### Dynamic Insertion

A vehicle enters an environment containing active traffic.

### Extreme Velocity Differences

Fast and slow agents interact.

### Goal Swapping

Vehicles attempt to exchange destinations.

### Cascading Conflicts

A sequence of vehicles produces dependent interactions.

### Constraint Stress

Several agents enter close proximity simultaneously.

### Obstacle Interaction

Dynamic conflict resolution occurs near static obstacles.

### Grazing Trajectories

Vehicles approach the safety boundary without large initial overlap.

### Edge Cases

Additional tests include:

* zero-distance configurations
* vehicles initialized inside obstacles
* extreme priority values
* delayed vehicle activation
* vertical motion
* high-speed interactions

The purpose of these scenarios is to identify failure modes rather than simply demonstrate successful nominal operation.

---

# Experimental Evaluation

The algorithm is intended to be evaluated along four primary dimensions.

## Safety

* Minimum inter-agent separation
* Number of safety violations
* Obstacle violations
* Mission completion rate

## Efficiency

* Total path length
* Travel time
* Additional delay
* Trajectory deviation

## Computational Cost

* A* execution time
* A* node expansions
* Number of replanning events
* Constraint-resolution iterations
* Total simulation time

## Scalability

Experiments can progressively increase fleet size:

```text
10
25
50
100
250
500
1000
```

subject to the computational resources available.

---

# Ablation Study

A key component of the evaluation is determining whether each layer of the architecture provides measurable benefit.

The proposed ablation sequence is:

```text
                    A*
                     │
                     ▼
              A* + Evasion
                     │
                     ▼
          A* + Evasion + PBD
                     │
                     ▼
       A* + Evasion + Local PBD
```

These configurations can be compared using identical traffic scenarios and random seeds.

The objective is to determine:

* whether reactive evasion reduces replanning,
* whether constraint resolution improves robustness,
* whether localization reduces computational cost,
* and whether the complete hierarchy provides a meaningful advantage over individual components.

---

# Baselines

The algorithm should be evaluated against established approaches rather than only against itself.

Potential comparison methods include:

* A*-only planning
* A* with conventional reactive avoidance
* Reciprocal Velocity Obstacles (RVO)
* Optimal Reciprocal Collision Avoidance (ORCA)
* Hybrid Reciprocal Velocity Obstacles (HRVO)
* Multi-Agent Path Finding approaches
* Model Predictive Control / Distributed MPC
* variants of the proposed architecture with individual layers removed

The exact baseline set will depend on implementation feasibility and experimental scope.

---

# Research Questions

The project is intended to investigate questions such as:

### RQ1 — Computational Efficiency

Can localized, event-triggered coordination reduce unnecessary replanning compared with repeatedly invoking a larger planning problem?

### RQ2 — Safety

Can the layered architecture maintain required separation under dense and adversarial traffic?

### RQ3 — Scalability

How does computational cost change as the number of interacting agents increases?

### RQ4 — Locality

What is the relationship between the local planning radius and coordination performance?

### RQ5 — Priority

Does incorporating asymmetric vehicle priority produce useful behavior in heterogeneous fleets?

### RQ6 — Constraint Resolution

Does the geometric correction layer improve robustness in situations where discrete trajectory updates produce residual constraint violations?

---

# Limitations

This implementation is currently a research prototype.

### Discrete-Time Evaluation

The simulator operates using discrete time steps. Therefore, successful discrete-time verification should not be interpreted as a mathematical proof of continuous-time collision avoidance.

### No Formal Safety Guarantee

The current implementation does not establish a formal invariant proving that collisions are impossible under all initial conditions.

### Simplified Vehicle Dynamics

The current vehicle model does not fully represent aerodynamic effects, acceleration constraints, actuator limitations, communication latency, localization uncertainty, or vehicle-specific dynamics.

### Bounded A* Search

The local planner is subject to finite search resolution and iteration limits.

### Simulation-to-Reality Gap

Results obtained from the simulator require further validation using realistic dynamics and, ultimately, physical systems.

---

# Repository Structure

The repository is intended to separate the core algorithm from the experimental framework:

```text
algorithm/
    planner/
    conflict_detection/
    evasion/
    constraint_resolution/

simulation/
    vehicle_model/
    environment/
    scenarios/

experiments/
    adversarial/
    scalability/
    ablation/
    baselines/

results/
    figures/
    tables/

docs/
    algorithm.md
    mathematical_formulation.md
```

The separation is intentional: the algorithm should remain usable independently of the simulation environment.

---

# Project Status

**Research Prototype**

Current development is focused on:

* algorithm formalization
* adversarial testing
* computational benchmarking
* baseline comparison
* ablation studies
* scalability experiments
* mathematical analysis
* reproducibility
* research publication

---

# Reproducibility

Experiments will be configured using explicit parameters rather than manually modified source code wherever possible.

Each experiment should record:

```text
Number of Agents
Environment Size
Obstacle Configuration
Vehicle Speeds
Vehicle Priorities
Safety Margins
Planning Resolution
Local Planning Radius
Simulation Timestep
3D Planning Mode
Constraint Resolution Mode
Random Seed
```

Results should be generated from reproducible experiment configurations.

---

# Research Artifact

This repository contains the reference implementation of the proposed algorithm and its experimental evaluation framework.

A versioned release will be archived using a persistent research identifier once the implementation reaches a stable experimental state.

**GitHub:** `<repository-url>`

**DOI:** `<to be assigned>`

---

# Citation

A formal citation will be added with the corresponding research publication and archived software release.

```bibtex
@software{paramesh_sriram_trajectory_coordination,
  author  = {Paramesh Sriram},
  title   = {Hierarchical Event-Triggered Multi-Agent Trajectory Coordination},
  year    = {2026},
  version = {1.0.0},
  url     = {<repository-url>},
  doi     = {<doi>}
}
```

---

# Author

**Paramesh Sriram**

Robotics · Autonomous Systems · Path Planning · Computer Vision · Multi-Agent Coordination

---

## Disclaimer

This repository represents ongoing research. The implementation and experimental results should not be interpreted as demonstrating guaranteed collision-free operation in real-world autonomous systems without additional validation and formal safety analysis.
