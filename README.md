# 🐕Quadruped-Robot
A modular simulation framework for quadrupedal robot locomotion using Webots and MATLAB/Simulink. Features a flexible controller architecture designed for advanced motion planning and stabilization algorithms

## 📌 Project Overview
This project focuses on developing a robust control system for a 8-DOF quadruped robot. The system integrates high-fidelity physics simulation in **Webots** with advanced control laws implemented in **MATLAB/Simulink**.

## 📂 Directory Structure
To maintain a clean and modular environment, the project is organized as follows:

* **/Worlds**: Contains Webots world files (`.wbt`), robot descriptions (URDF/PROTO), and 3D assets (STL). 
    * *Primary Owner: Tran*
* **/Dynamics**: Stores mathematical derivations of the robot's EOM (Equations of Motion), including M, C, and G matrices.
    * *Primary Owner: Son*
* **/Controllers**: The core control logic. Includes the Sliding Mode Control (SMC) blocks and future control alternatives.
    * *Primary Owner: Thu*
* **/Gaits**: Trajectory generation scripts for various locomotion patterns (Trot, Walk, Bound).
    * *Primary Owner: Son*
* **/Scripts**: Utility scripts for environment initialization (`startup.m`) and data visualization/plotting.
* **/Data**: Storage for simulation logs and pre-calculated trajectory files (`.mat`).

## 🚀 Getting Started
1. **Clone the repository:** `git clone https://github.com/your-username/Quadruped-Robot.git`
2. **Initialize the Project:** Open MATLAB and double-click `Quadruped_Robot.prj`. This will automatically set up the project paths.
3. **Run Simulation:** Open the main Simulink model in `/Controllers` and ensure Webots is running the corresponding world file.

## 🛠 Development Guidelines
* **Git Workflow:** Always `Pull` before starting work and `Push` after completing a task.
* **Commit Convention:** Use clear, English descriptions (e.g., `feat: add swing leg trajectory`).
* **Simulink Safety:** Do not modify `.slx` files simultaneously on different machines to avoid merge conflicts.
