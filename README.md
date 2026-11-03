# Interactive Aircraft Control System Visualizer

An interactive Python-based simulation of an aircraft control system demonstrating altitude and heading regulation using nested PID controllers, simplified flight dynamics, and real-time response visualization.

## 1. Project Overview

The Interactive Aircraft Control System Visualizer is an educational simulation developed to demonstrate how feedback control systems regulate an aircraft's altitude and heading.

The project models two interconnected control architectures: an altitude–pitch control loop and a heading–roll control loop. Through an interactive graphical interface, users can adjust reference values, tune PID controller gains, introduce external disturbances, and observe the system's dynamic response.

The simulation provides a practical way to explore feedback control, PID tuning, nested control loops, disturbance rejection, and system response.

## 2. Project Objectives

- Develop an interactive platform for visualizing aircraft control-system behavior.
- Implement nested PID control loops for altitude and heading regulation.
- Investigate the relationship between pitch, roll, altitude, and heading.
- Examine the effects of proportional, integral, and derivative gains on system response.
- Analyze system behavior under different external disturbances.
- Visualize the dynamic response of the system through real-time plots.

## 3. System Architecture

The simulation consists of two interconnected nested control loops.

### 3.1 Altitude–Pitch Control Loop

The altitude control architecture consists of an outer altitude PID controller and an inner pitch PID controller.

**Control sequence:**

Altitude Reference → Altitude PID → Pitch Reference → Pitch PID → Elevator Command → Aircraft Pitch Dynamics → Pitch Angle → Altitude Update

The outer controller calculates the desired pitch reference based on the altitude error. The inner controller then adjusts the elevator command to achieve the desired pitch angle.

Altitude is updated using a simplified kinematic relationship between forward airspeed and pitch angle.

### 3.2 Heading–Roll Control Loop

The heading control architecture consists of an outer heading PID controller and an inner roll PID controller.

**Control sequence:**

Heading Reference → Heading PID → Roll Reference → Roll PID → Aileron Command → Aircraft Roll Dynamics → Roll Angle → Heading Update

The outer controller generates a roll reference from the heading error, while the inner controller adjusts the aileron command to regulate the roll angle.

Heading is updated using a simplified relationship between roll angle and heading rate.

## 4. Mathematical Modeling

The simulation uses simplified second-order models for pitch and roll dynamics.

**Pitch dynamics:**

$$
\ddot{\theta} + 2\zeta_\theta \omega_{n_\theta} \dot{\theta} + \omega_{n_\theta}^{2} \theta = k_\theta u_e
$$

**Roll dynamics:**

$$
\ddot{\phi} + 2\zeta_\phi \omega_{n_\phi} \dot{\phi} + \omega_{n_\phi}^{2} \phi = k_\phi u_a
$$

Where:

- $\theta$: Pitch angle
- $\phi$: Roll angle
- $\zeta$: Damping ratio
- $\omega_n$: Natural frequency
- $u_e$: Elevator command
- $u_a$: Aileron command
- $k_\theta, k_\phi$: Control input gains

The altitude and heading updates use simplified kinematic approximations:

$$
\dot{h} \approx V \theta
$$

$$
\dot{\psi} \approx k_\psi \phi
$$

The differential equations are numerically integrated using the forward Euler method.

## 5. PID Controller Implementation

Four PID controllers are implemented in a nested-loop configuration.

| Controller | Function |
|---|---|
| Altitude PID | Generates the desired pitch reference |
| Pitch PID | Regulates pitch through elevator input |
| Heading PID | Generates the desired roll reference |
| Roll PID | Regulates roll through aileron input |

Each controller incorporates proportional, integral, and derivative actions. Users can independently modify the $K_p$, $K_i$, and $K_d$ gains of all four controllers through the graphical interface.

## 6. Interactive Features

- **Adjustable Setpoints:** Specify desired altitude and heading.
- **PID Gain Tuning:** Modify the proportional, integral, and derivative gains of each controller.
- **Disturbance Injection:** Introduce step-like, impulse, and sinusoidal disturbances.
- **Simulation Speed Control:** Adjust the simulation speed or enable fast simulation mode.
- **Real-Time Visualization:** Observe the system response through dynamically updated plots.
- **Simulation Controls:** Start, stop, and reset the simulation.

## 7. Visualization and Results

The application displays three main response plots:

1. **Altitude Response:** Comparison between the desired altitude reference and actual simulated altitude.
2. **Heading Response:** Comparison between the desired heading reference and actual simulated heading.
3. **Pitch, Roll, and Actuator Response:** Visualization of pitch angle, roll angle, elevator command and aileron command.

These plots help illustrate controller behavior, system dynamics and the response to external disturbances.

## 8. Technologies Used

- **Programming Language:** Python
- **Numerical Computation:** NumPy
- **Data Visualization:** Matplotlib
- **Graphical User Interface:** Tkinter
- **Numerical Integration:** Forward Euler Method
- **Control Technique:** PID Control

## 9. Installation and Execution

### Prerequisites

- Python 3.9 or later
- NumPy
- Matplotlib
- Tkinter (included with many standard desktop Python installations)

### Installation

Clone the repository:

```bash
git clone https://github.com/mashrursaademad/interactive-aircraft-control-system-visualizer.git
```

Navigate to the project directory:

```bash
cd interactive-aircraft-control-system-visualizer
```

Install the required libraries:

```bash
pip install -r requirements.txt
```

Run the application:

```bash
python src/aircraft_control_visualizer.py
```

## 10. Project Structure

```text
interactive-aircraft-control-system-visualizer/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   └── aircraft_control_visualizer.py
│
├── documentation/
│   └── EEE4606_Project_Report.pdf
│
└── results/
    └── figures/
        ├── interface.png
        ├── altitude_response.png
        ├── heading_response.png
        └── pitch_roll_actuator_response.png
```

## 11. Documentation

For detailed mathematical derivations, laboratory findings, and controller design analysis, refer to the complete academic report:
- 📄 [EEE4606 Project Report (PDF)](documentation/EEE4606_Project_Report.pdf)

## 12. Model Scope and Limitations

This project uses simplified aircraft dynamics to demonstrate fundamental feedback control concepts. The pitch and roll axes are represented using second-order models, while altitude and heading are calculated through simplified kinematic relationships.

The simulation is intended for educational and control-system visualization purposes. It is not a high-fidelity aircraft flight-dynamics model or a flight-certified autopilot implementation.

## 13. Contributors and Repository Maintainer

### 13.1 Contributors

Developed as a group academic project for:

**Course:** EEE 4606 – Control System Engineering Lab  
**Institution:** Islamic University of Technology (IUT)  
**Team:** Team 00  
**Date of Submission:** 05 September 2025

**Team Members:**

- Mashrur Saad Emad
- Arad Taousif
- Shafath Uddin Khoundaker
- Fahimun Nabi Torongo
- Khandakar Siaum Masud

### 13.2 Repository Maintainer

**Mashrur Saad Emad**  
Electrical and Electronic Engineering Graduate  
Islamic University of Technology (IUT)

[GitHub Profile](https://github.com/mashrursaademad)  
[LinkedIn Profile](https://www.linkedin.com/in/mashrur-saad-emad-787995436/)
