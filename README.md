# Cooperative Robotics — MATLAB Assignments

> Three MATLAB projects focused on robotic manipulation and multi-robot cooperation, implementing Task Priority Inverse Kinematics algorithms and hierarchical constraint management for complex manipulator systems.

## Table of Contents

- [Overview](#overview)
- [Project 1 — Marine Vehicle with Manipulator](#project-1--marine-vehicle-with-manipulator)
- [Project 2 — Bimanual Rigid Grasping](#project-2--bimanual-rigid-grasping)
- [Project 3 — Cooperative Manipulation](#project-3--cooperative-manipulation)
- [Python Simulator](#python-simulator)
- [Requirements](#requirements)
- [How to Run](#how-to-run)

## Overview

This repository collects all assignments developed for the **Cooperative Robotics** course. The common thread across all three projects is the **Task Priority Inverse Kinematics** framework, used to handle hierarchical constraints and redundancy in complex robotic systems — from marine vehicles to dual-arm manipulators.

| Project | System | Key concept |
|:---:|:---|:---|
| 1 | Marine vehicle + manipulator | Waypoint navigation, landing alignment, manipulation |
| 2 | Bimanual Franka Emika Panda | Bimanual Rigid Grasping Constraint |
| 3 | Two independent manipulators | Cooperative Rigid Constraint with mutual adaptation |

## Project 1 — Marine Vehicle with Manipulator

### `submarine/`

The first project addresses the control of a **marine vehicle equipped with a robotic manipulator**. The mission is divided into three sequential phases, each managed through a Task Priority algorithm with dedicated constraint hierarchies.

### Phases

**1. Waypoint navigation**
The vehicle navigates safely through a sequence of waypoints in the underwater environment. Kinematic and safety tasks are prioritized to avoid singularities and workspace violations during transit.

**2. Landing with precise alignment**
The landing phase requires:
- Aligning the vehicle's x-axis with the **projection onto the horizontal plane** of the vector connecting the vehicle frame to the target frame
- Ensuring that, once landed, the **target always remains within the manipulator's workspace**

This dual constraint — directional alignment and reachability guarantee — is handled simultaneously through the task priority hierarchy.

**3. Manipulation**
Once landed, the vehicle is **rigidly constrained** to the ground. During this phase, priorities are reassigned exclusively to kinematic and safety tasks for the manipulator, while the vehicle degrees of freedom are locked.

## Project 2 — Bimanual Rigid Grasping

### `bimanual_manipulators/`

The second project implements a **bimanual manipulation strategy** using two **Franka Emika Panda** manipulators, modeled as a single unified redundant robotic system.

### Implementation details

- **Kinematic modeling**: forward kinematics computed from both base frames, with custom tool frames defined for each end-effector
- **Move-to action**: a coordinated motion primitive including:
  - Joint limit constraints
  - Minimum end-effector altitude constraint (safety floor)
- **Bimanual Rigid Grasping Constraint**: after both end-effectors reach the grasping points, the object frame is modeled as rigidly attached to both end-effectors simultaneously. The object is then transported as a rigid body, with the constraint handled as a **binary task** (inactive → active transition at grasp).

The key insight is treating the two arms as a single kinematic chain, which allows a unified Task Priority formulation over the combined joint space.

## Project 3 — Cooperative Manipulation

### `cooperative_manipulation_exercise/`

The third project addresses the same dual-arm scenario as Project 2, but with a fundamentally different architecture: the two manipulators are treated as **distinct robots**, each running its own independent Task Priority algorithm.

### Implementation details

**Phase 1 — Independent reaching**
Each robot independently moves to its grasping point with active safety constraints (joint limits, altitude floor), without any coordination between them.

**Phase 2 — Cooperative Rigid Constraint**
After grasping, cooperation is enforced through the following pipeline:

1. Compute **non-cooperative object velocities** from each robot's independent Task Priority solution
2. Apply a **coordination policy** to reconcile the two independent velocity fields
3. Derive the final **cooperative object velocities**, ensuring mutual adaptation

The constraint transition (free → cooperative) is modeled as **binary**, with appropriate modifications to the action management structure to handle the switch cleanly.

The result demonstrates genuine mutual adaptation: neither robot dominates — both adjust their motion to reach a shared consistent object trajectory.

## Python Simulator

### `python_simulator/`

A Python-based 3D simulator used to visualize the **Franka Emika Panda** dual-arm setup. The simulator places the two robot arms facing each other, providing a visual reference for the grasping configuration used in Projects 2 and 3.

It is used as a visual companion to the MATLAB implementations — launch it first to see the robot configuration, then run the corresponding MATLAB script.

### Launch

```bash
# From the python_simulator/ folder
python3 simulator.py
```

## Requirements

### MATLAB Toolboxes

| Toolbox | Used for |
|:---|:---|
| **Robotics System Toolbox** | Kinematic modeling, transforms, Jacobians |
| **Control System Toolbox** | System analysis and control design |

Tested on **MATLAB R2023b** or later.

### Python (simulator only)

```bash
pip install numpy matplotlib
# or
pip install -r requirements.txt
```

## How to Run

### Projects 1, 2, 3 (MATLAB)

Each project folder contains a main entry-point script. Open MATLAB, navigate to the project folder, and run:

```matlab
% Standard version
main

% Robust version (if available — includes additional constraint handling)
robustmain
```

> **Tip:** always launch the Python simulator first if you want a live visual of the robot configuration while the MATLAB script runs.

### Recommended order

```
1. cd python_simulator/ → python3 simulator.py     (optional, visual only)
2. Open MATLAB
3. cd <project_folder>/
4. Run main.m  or  robustmain.m
```

## Repository Structure

```
cooperative/
├── submarine/                        # Project 1 — Marine vehicle + manipulator
├── bimanual_manipulators/            # Project 2 — Bimanual Rigid Grasping
├── cooperative_manipulation_exercise/ # Project 3 — Cooperative Rigid Constraint
└── python_simulator/                 # Franka dual-arm 3D visualizer
```
