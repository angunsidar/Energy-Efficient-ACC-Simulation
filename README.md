# Energy-Efficient Adaptive Cruise Control Simulation

**Project Type:** Independent Research Project  
**Author:** Sidar Angün  
**Date:** January 2026  

## 📌 Project Overview
This repository contains the source code and research report for a comparative analysis of **Static (PID)** vs. **Dynamic (Kinematic Horizon)** Adaptive Cruise Control strategies. The study utilizes a physics-based simulation engine calibrated to **Tesla Model Y Long Range AWD** specifications to evaluate energy efficiency under stochastic traffic conditions.

## 🚀 Key Features
- **High-Fidelity Physics Engine:** Models aerodynamic drag ($C_d=0.23$), rolling resistance, and regenerative braking limits (75 kW).
- **Stochastic Traffic Generation:** Simulates realistic "stop-and-go" waves and high-speed highway merging.
- **Dynamic Horizon Logic:** Implements a predictive control strategy that utilizes an elastic safety buffer to minimize thermal waste.
- **Forensic Data Analysis:** Automatically detects "Death Points" (battery depletion) and calculates efficiency gains.

## 📂 Repository Contents
- **`S_Angun_EV_ACC_Research_Report.pdf`**: Full academic report detailing the methodology, mathematical models, and results.
- **`Source_Code/`**: Contains the MATLAB simulation script (`ACC_Simulation_Source_Code.m`).

## 💻 How to Run
1. Ensure you have MATLAB (R2020a or newer) installed.
2. Download the `ACC_Simulation_Source_Code.m` file.
3. Open in MATLAB and click **Run**.
4. The script will generate:
   - Velocity Profile Comparison Plot
   - State of Charge (SoC) Depletion Plot
   - Efficiency Metrics Bar Charts
   - Terminal Output with Efficiency Calculation

---
*This project validates that software-defined driving strategies can improve EV range by ~21-23% without hardware upgrades.*
