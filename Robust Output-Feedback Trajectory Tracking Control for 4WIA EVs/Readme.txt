Robust Output-Feedback Trajectory Tracking Control for 4WIA EVs

This repository contains the MATLAB implementation of a Robust H-infinity Output-Feedback Controller for a 4-Wheel Independent Actuated (4WIA) Electric Vehicle.
The project reproduces and extends the control strategy proposed by Li, Jiao, and Zhang (2023) using a direct LMI-based synthesis approach combined with a physics-informed polarity correction mechanism.

The repository includes controller design, validation under nominal conditions, and robustness evaluation under realistic uncertainties such as payload variation and reduced road friction.

---

Prerequisites and Installation

To run the simulations, you will need MATLAB, YALMIP, and the MOSEK solver. Follow the steps below to set up your environment.

---

1. Install MATLAB

Download and install MATLAB R2022b or later from the MathWorks website.

Ensure that the Control System Toolbox is installed. This toolbox is typically included with academic licenses.

---

2. Install MOSEK Solver

The LMI optimization problems are solved using MOSEK.

Step 1: Download
Visit the MOSEK downloads page and download the installer for your operating system.

Step 2: Install
Run the installer and note the installation directory, for example:
C:\Program Files\Mosek

Step 3: Obtain License
Students can apply for a free academic license from MOSEK.

After approval, you will receive a license file named "mosek.lic".

Place the license file in the following directory:

Windows:
C:\Users<YourUser>\mosek\

Linux or macOS:
/home/<user>/mosek/

---

3. Install YALMIP Toolbox

YALMIP is used as the optimization modeling interface in MATLAB.

Download the latest version of YALMIP from its official GitHub repository.

Extract the downloaded folder to a convenient location, for example:
C:\MATLAB\YALMIP-master

---

4. Configure MATLAB Path

Open MATLAB.

Go to Home -> Set Path -> Add with Subfolders.

Add the extracted YALMIP folder.

Add the MOSEK toolbox folder, for example:
C:\Program Files\Mosek\10.0\toolbox\r2017a

Click Save and Close.

To verify the installation, run the following command in the MATLAB Command Window:

yalmiptest

MOSEK should be detected successfully.

---

Repository Structure

Robustness_Stress_Test_Script.m
Run this file first. It defines the vehicle model, solves the robust LMI optimization problem, constructs the controller, and validates nominal performance.

Controller_Design_and_Validation_Scripts.m
Run this file second. It performs robustness analysis under parameter variations and generates comparison plots and performance metrics.

Baseline_Comparison_Script_Optional.m
Optional file. Implements a conventional PID controller for baseline comparison.

---

How to Run the Simulations

---

Step 1: Controller Design and Nominal Validation

Run the script:
Robustness_Stress_Test_Script.m

What this script does:

* Solves the robust H-infinity LMI optimization using MOSEK
* Applies physics-informed polarity correction
* Saves the valid controller in FWIA_Controller.mat

Expected output:

* A console message indicating successful controller synthesis
* A plot showing lateral error convergence to zero within approximately 2.5 seconds

---

Step 2: Robustness Extension Analysis

Run the script:
Controller_Design_and_Validation_Scripts.m

Prerequisite:
Ensure that FWIA_Controller.mat is present in the same directory.

What this script does:

* Loads the designed controller
* Simulates three scenarios:

  1. Nominal conditions
  2. Heavy load with a 40 percent increase in vehicle mass
  3. Slippery road with a 60 percent reduction in tire-road friction

Expected output:

* Comparative trajectory tracking plots
* A table in the MATLAB Command Window reporting RMS error and settling time

---

Step 3: Baseline Comparison (Optional)

Run the script:
Baseline_Comparison_Script_Optional.m

Purpose:

* Simulates a standard PID controller on the same vehicle model
* Demonstrates the limitations of classical control under uncertainty

---

Results Summary

Nominal Performance:
Zero steady-state lateral tracking error with minimal overshoot.

Robustness:
Stable trajectory tracking is maintained even when:

* Vehicle mass is increased by 40 percent
* Road friction is reduced by 60 percent (wet or icy conditions)

Trade-off:
Under extreme conditions, the settling time increases by approximately 30 percent, which is consistent with physical traction limits.

---

Authors

Name: Touha Khalid & Muhammad Nawaz Awan
Course: EE-605 Robust Control Systems
Semester: Fall 2025


