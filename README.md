

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
> My primary contribution to this project was the design and implementation of the **LQR** and **iLQR** controllers.
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

Video of Real_Time_LQR_Sine_Wave:
https://github.com/user-attachments/assets/447cfacf-22f1-4f2b-9571-8c07defc4a4f

Video of Real_Time_LQR_Square_Wave:
https://github.com/user-attachments/assets/14a66773-ab24-4a21-ae49-fabb6a33d0a9

✅ Stabilizes the pendulum successfully.

✅ Tracks both sine and square references.

⚠️ Exhibits tracking delay and relatively aggressive control inputs.

---

## iLQR

Video of Real_Time_iLQR_Sine_Wave:
https://github.com/user-attachments/assets/d3419d18-deee-4a78-9cc5-0c040fe860d4

Video of Real_Time_iLQR_Square_Wave:
https://github.com/user-attachments/assets/baaa836d-1664-4291-bcdc-be023f378ba4

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
├── LQR_Simulation/
|        ├── Simulation_LQR_trajectory_sine.m
|        ├── Simulation_LQR_trajectory_square.m
├── LQR_Real_Time_Deployment/

https://github.com/user-attachments/assets/84fa6f3d-f57c-4ea4-a053-8b7d22a96f61


|        ├── Real_time_sine.m
|        ├── Real_time_square.m
│
├── iLQR_Simulation/
|        ├── Simulation_iLQR_Sine.m
|        ├── Simulation_iLQR_Square.m
├── iLQR_Real_Time_Deployment/
|        ├── Real_Time_iterative_LQR_sine.m
|        ├── Real_Time_iterative_LQR_square.m
│
├── Results/
│   ├── LQR_Results/
│     ├── Simulation/
|        ├── Sim_Square_LQR.jpg
|        ├── Sim_Sine_LQR.jpg
│     ├── Real_Time_Results/
|        ├── Real_Time_Sine.jpg
|        ├── Real_Time_Square.jpg
|        ├── Real_Time_LQR_Sine_Wave.mp4
|        ├── Real_Time_LQR_Square_Wave.mp4
│   ├── iLQR_Results/
│     ├── Simulation/
|        ├── Sim_Sine_iLQR.jpg
|        ├── Sim_Sine_iter.jpg
|        ├── Sim_Square_iLQR.jpg
|        ├── Sim_Square_iter.jpg
│     ├── Real_Time_Results/
|        ├── Real_Time_Sine.jpg
|        ├── Real_Time_Square.jpg
|        ├── Real_Time_iLQR_Sine_Wave.mp4
|        ├── Real_Time_iLQR_Square_Wave.mp4
└── README.md
```
