# 🐕Quadruped-Robot
A modular simulation framework for quadrupedal robot locomotion using Webots and MATLAB/Simulink. Features a flexible controller architecture designed for advanced motion planning and stabilization algorithms.

## 📌 Project Overview
This project focuses on developing a robust control system for an 8-DOF quadruped robot. The system integrates high-fidelity physics simulation in **Webots** with advanced control laws implemented in **C++** and **MATLAB/Simulink**.

## 📂 Directory Structure
To maintain a clean and modular environment, the project is organized as follows:

* **/Worlds**: Contains Webots world files (`.wbt`), robot descriptions (URDF/PROTO), and 3D assets (STL). 
* **/Dynamics**: Stores mathematical derivations of the robot's EOM (Equations of Motion), including M, C, and G matrices.
* **/Controllers**: The core control logic. Includes the Sliding Mode Control (SMC) blocks and future control alternatives.
* **/Gaits**: Trajectory generation scripts for various locomotion patterns (Trot, Bound, Pace, Walk).
* **/Scripts**: Utility scripts for environment initialization (`startup.m`) and data visualization/plotting.
* **/Data**: Storage for simulation logs and pre-calculated trajectory files (`.mat`).

## 🎮 Interactive Controls & Gaits
During the Webots simulation, you can dynamically switch between different locomotion gaits in real-time using the keyboard. 
*(Note: Ensure you click inside the Webots 3D viewport to make it active before pressing the keys).*

* **`T` - Trot:** A diagonal gait where diagonal pairs of legs (e.g., Front-Left and Back-Right) move simultaneously.
* **`W` - Walk:** A slow, highly stable gait where at least 3 feet are always in contact with the ground at any given time.
* **`P` - Pace:** A lateral gait where the two left legs move together, alternating with the two right legs.
* **`B` - Bound:** A leaping gait where the two front legs move together, followed by the two hind legs.

## 🛠 Development Guidelines
* **Git Workflow:** Always `Pull` before starting work and `Push` after completing a task.
* **Commit Convention:** Use clear, English descriptions (e.g., `feat: add swing leg trajectory`).
* **Simulink Safety:** Do not modify `.slx` files simultaneously on different machines to avoid merge conflicts.
