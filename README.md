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
* **/Data**: Storage for simulation logs and pre-calculated trajectory files (`.mat`).

## 🎮 Interactive Controls & Gaits
During the Webots simulation, you can dynamically switch between different locomotion gaits and control strategies in real-time using the keyboard.
*(Note: Ensure you click inside the Webots 3D viewport to make it active before pressing the keys).*

**Default Initialization:** The system starts with Model-Based SMC and the Trot Gait.

**Control Strategies:** Toggle between Model-Free SMC (press "1") and Model-Based SMC (press "2").

**Gaits:** Switch between various gaits such as: 
* **`T` - Trot:** A diagonal gait where diagonal pairs of legs (e.g., Front-Left and Back-Right) move simultaneously.
<table style="width: 100%; text-align: center;">
  <tr>
    <th style="width: 50%;">Model-Free SMC</th>
    <th style="width: 50%;">Model-Based SMC</th>
  </tr>
  <tr>
    <td>
      <video src="https://github.com/user-attachments/assets/11ef0175-4e76-4521-b26d-f165402c2548" width="100%" controls autoplay loop muted></video>
    </td>
    <td>
      <video src="https://github.com/user-attachments/assets/3212cb97-ae6e-4037-bb9d-5528dcd3ab7a" width="100%" controls autoplay loop muted></video>
    </td>
  </tr>
</table>

* **`W` - Walk:** A slow, highly stable gait where at least 3 feet are always in contact with the ground at any given time.
<table style="width: 100%; text-align: center;">
  <tr>
    <th style="width: 50%;">Model-Free SMC</th>
    <th style="width: 50%;">Model-Based SMC</th>
  </tr>
  <tr>
    <td>
      <video src="https://github.com/user-attachments/assets/2ed65e74-6ee4-410b-a1dc-4054953d198b" width="100%" controls autoplay loop muted></video>
    </td>
    <td>
      <video src="https://github.com/user-attachments/assets/05c444c5-f205-43e9-95da-8df98727b470" width="100%" controls autoplay loop muted></video>
    </td>
  </tr>
</table>

* **`P` - Pace:** A lateral gait where the two left legs move together, alternating with the two right legs.
<table style="width: 100%; text-align: center;">
  <tr>
    <th style="width: 50%;">Model-Free SMC</th>
    <th style="width: 50%;">Model-Based SMC</th>
  </tr>
  <tr>
    <td>
      <video src="https://github.com/user-attachments/assets/e3df521c-f3b8-42ec-be1b-5afc5914cc1c" width="100%" controls autoplay loop muted></video>
    </td>
    <td>
      <video src="https://github.com/user-attachments/assets/648aef62-23cb-477a-8a12-98e8a53e89d6" width="100%" controls autoplay loop muted></video>
    </td>
  </tr>
</table>

* **`B` - Bound:** A leaping gait where the two front legs move together, followed by the two hind legs.
<table style="width: 100%; text-align: center;">
  <tr>
    <th style="width: 50%;">Model-Free SMC</th>
    <th style="width: 50%;">Model-Based SMC</th>
  </tr>
  <tr>
    <td>
      <video src="https://github.com/user-attachments/assets/d4465eb6-d4db-46ad-965c-e6abe183f684" width="100%" controls autoplay loop muted></video>
    </td>
    <td>
      <video src="https://github.com/user-attachments/assets/3d16f8f0-690b-40d1-af11-016b4956cf01" width="100%" controls autoplay loop muted></video>
    </td>
  </tr>
</table>

## 🛠 Development Guidelines
* **Git Workflow:** Always `Pull` before starting work and `Push` after completing a task.
* **Commit Convention:** Use clear, English descriptions (e.g., `feat: add swing leg trajectory`).
* **Simulink Safety:** Do not modify `.slx` files simultaneously on different machines to avoid merge conflicts.
