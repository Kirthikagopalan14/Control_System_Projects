# Advanced control of an inverted pendulum
MATLAB-LQR and iLQR Project repo
<img width="1307" height="665" alt="image" src="https://github.com/user-attachments/assets/1f24e3c6-15d2-4209-a2b9-7999dcdd8c2b" />

## Overview

This project investigates the stabilization and trajectory tracking of an inverted pendulum using several modern control techniques implemented in MATLAB.

The controllers are evaluated on both **sine** and **square-wave reference trajectories**, comparing their tracking accuracy, stability, control effort, and learning capability.

## Controllers Implemented

- Model Predictive Control (MPC)
- MPC + Iterative Learning Control (ILC)
- PID Control
- PID + ILC
- **Linear Quadratic Regulator (LQR)** 
- **Iterative Linear Quadratic Regulator (iLQR)** 
- Pole Placement
- Pole Placement + ILC
My primary contribution to this project was the design and implementation of the **LQR** and **iLQR** controllers.
---

# My Contribution

### Linear Quadratic Regulator (LQR)

- Developed a full-state feedback controller.
- Designed the optimal feedback gain using the Algebraic Riccati Equation.
- Tuned the weighting matrices **Q** and **R** to balance tracking accuracy and control effort.
- Evaluated controller performance on multiple reference trajectories.

### Iterative Linear Quadratic Regulator (iLQR)

Implemented an iterative optimal control framework consisting of:

- Trajectory rollout
- Cost function optimization
- Backward pass
- Forward pass
- Line search
- Iterative policy improvement

The controller progressively refines the control sequence to improve trajectory tracking while reducing unnecessary control effort.

---
# Features

- MATLAB implementation
- Nonlinear inverted pendulum simulation
- Optimal state-feedback control
- Iterative optimal trajectory optimization
- Sine and square trajectory tracking
- Performance comparison among multiple controllers

---

# Results

## LQR

✅ Stabilizes the pendulum successfully.

✅ Tracks both sine and square references.

⚠️ Exhibits tracking delay and relatively aggressive control inputs.

---

## iLQR

Compared with LQR, the iterative controller demonstrates:

- Improved tracking accuracy
- Reduced steady-state offset
- Lower control effort
- Smoother control signals
- Better convergence over successive iterations

---

# Performance Highlights

| Metric | LQR | iLQR |
|---------|-----|-------|
| Pendulum Stabilization | ✅ | ✅ |
| Reference Tracking | Good | Excellent |
| Tracking Delay | Moderate | Low |
| Control Effort | High | Reduced |
| Learning Capability | ❌ | ✅ |
| Iterative Optimization | ❌ | ✅ |

---
# Repository Structure

```
.
├── LQR/
│   ├── Simulation/
|        ├── Simulation_LQR_trajectory_sine.m
|        ├── Simulation_LQR_trajectory_square.m
│   ├── Real_Time_Deployment/
|        ├── Real_time_sine.m
|        ├── Real_time_square.m
│
├── iLQR/
│   ├── Simulation/
|        ├── Simulation_iLQR_sine.m
|        ├── Simulation_iLQR_square.m
│   ├── Real_Time_Deployment/
|        ├── Real_Time_iterative_LQR_sine.m
|        ├── Real_Time_iterative_LQR_square.m
│
├── Results/
│   ├── LQR_Results/
│     ├── Simulation/
|        ├── sim_square_LQR.jpg
|        ├── sim_sine_LQR.jpg
│     ├── Real_Time_Results/
|        ├── Real_time_sine.jpg
|        ├── Real_time_square.jpg
│   ├── iLQR_Results/
│     ├── Simulation/
|        ├── Simulation results_iLQR.jpg
|        ├── Simulation results_iter.jpg
|        ├── Square_simulation_iLQR.jpg
|        ├── Square_simulation_iter.jpg
│     ├── Real_Time_Results/
|        ├── Real_time_iLQR.jpg
|        ├── Real_time iter.jpg
|        ├── Square_Real_time_iLQR.jpg
|        ├── real_time_square.jpg
├── Videos/
│   ├── LQR_Results/
└── README.md
```
