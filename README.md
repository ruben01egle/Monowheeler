# Active Gyroscopic Stabilization of a One-Wheeled Vehicle
**Inspired by Charles Taylor | Advanced Control Systems Engineering**
This repository contains the complete development of an actively stabilized one-wheeled vehicle. Unlike traditional unicycles, this system utilizes the gyroscopic effect for lateral stability and maneuverability, allowing it to perform controlled turns by exploiting non-linear coupling and friction dynamics.

## From Taylor to Today
In the 1960s, Charles Taylor conceptualized a one-wheeled vehicle that intrigued the world with its unique physics. Taylor's original design laid the groundwork this project builds upon: it is the first of its kind to implement active gyroscopic stabilization specifically designed for intentional cornering.

By moving from passive stability to active control, this vehicle can navigate paths by dynamically shifting between stable equilibrium and controlled instability.
<table style="width: 100%; border-collapse: collapse; border: none;">
  <tr style="border: none;">
    <td align="center" valign="bottom" style="border: none; width: 50%; padding: 10px;">
      <b>Charles Taylor One wheeled vehicle 1960s</b><br><br>
      <img src="https://github.com/user-attachments/assets/2c74af30-6b0d-4a26-a515-28ff0a9f9db0" height="250" alt="TaylorOneWheel">
    </td>
    <td align="center" valign="bottom" style="border: none; width: 50%; padding: 10px;">
      <b>Monowheeler 2026</b><br><br>
      <img src="https://github.com/user-attachments/assets/a1e4c03b-b59b-4914-84b5-c801ddc55408" height="250" alt="Monowheeler">
    </td>
  </tr>
</table>

## See it in Action
**1. Disturbance Rejection & Balancing**\
*Demonstration of the vehicle maintaining stability against multi-directional external forces.*

<video src="https://github.com/user-attachments/assets/cb0de84c-932e-430b-98dd-ca0584cf337f" autoplay loop muted playsinline width="100%"></video>

**2. Active Cornering Control**\
*Showcasing precise trajectory tracking and controlled cornering dynamics.*

<video src="https://github.com/user-attachments/assets/f2f32b2e-e68b-4af0-807c-041e5cef68a0" autoplay loop muted playsinline width="100%"></video>

**3. Precision Maneuvering & Obstacle Avoidance**\
*Displaying the vehicle's agility and responsiveness required to navigate around obstacles and follow paths.*

<video src="https://github.com/user-attachments/assets/538ec53f-5d4b-4c0f-b6e7-cc7b96c15382" autoplay loop muted playsinline width="100%"></video>

## Project Scope
The project covers the full mechatronic V-model:
* **Hardware**
    * Custom mechanical design.
    * Electrical PCB integration.
* **Modeling**
    * Derivation of Equations of Motion ($EoM$) for a highly coupled, underactuated system.
* **Simulation**
    * Python-based simulation environment.
    * Verification of control laws against non-linear dynamics.
* **Embedded Systems**
    * **Sensor Fusion:** IMUs and digital filtering.
    * **Real-Time Control:** Control loop running on a Linux RT (Real-Time) Kernel.
    * **Telemetry:** Live data logging and custom Python GUI for real-time visualization.
 
## Bachelor Thesis
You can read the full documentation of the engineering process, including mathematical derivations, free body diagrams, and control theory:

* 🇩🇪 **[German Version (Original)](./thesis/Monowheeler_Thesis_DE.pdf)**
* 🇬🇧 **[English Version (Translation)](./thesis/Monowheeler_Thesis_EN.pdf)**

> **Note on Control Strategy:** While the original thesis utilizes a **Cascaded PID Controller** for balancing the Roll-Yaw-System, this repository has since been upgraded to a full **State-Space Controller**. This evolution provides significantly better stability.

>Note on Language: The original thesis was written in German. For the purpose of this repository and international accessibility, the content has been translated into English using DeepL. Some technical nuances may be best reflected in the original German text.

## Technical Deep Dive: The Challenge of One-Wheel Control
The vehicle is divided into two primary control systems:
### The Pitch System
Stabilization along the travel axis is achieved by shifting the wheel's position relative to the vehicle's frame, maintaining the center of gravity above the contact point.

### The Roll-Yaw System (The Core Challenge)
This is a highly non-linear, underactuated, and coupled system. Lateral stability is managed exclusively through a large internal gyroscope.

* Balancing: A State-Space Controller is used for upright stability. It assumes decoupling between Roll and Yaw, which is physically provided by static friction at the operating point.

* Cornering: A Cascaded PID Controller manages the non-controllable Yaw axis.

* The "Friction Trick": To initiate a turn, the vehicle intentionally breaks its stable equilibrium. By accelerating, we reduce the effective friction, allowing the coupling between Roll and Yaw to emerge. To return to a straight line, the vehicle decelerates, increasing friction to re-establish the decoupling.

* The cornering maneuver utilizes the unstable equilibrium created by the interplay of gravity (lean angle), the gyroscopic effect, and centrifugal force.

## Simulation & Verification
The repository includes a simulation where the motion equations from the thesis are implemented. This allows for testing the State-Space and PID controllers before deployment. Since the physical hardware is not available to everyone, the [Simulation Folder](./simulation/) serves as the primary way to interact with the project.

>Disclaimer: Friction parameters in the simulation are approximations. However, the qualitative behavior and stability margins have been successfully verified on the physical hardware.

## Embedded System
The core control logic resides in the [/MonowheelerControl](https://github.com/ruben01egle/MonowheelerControl.git) submodule. While designed for custom hardware, the architecture demonstrates a professional approach to real-time robotics:

* **Real-Time Execution**: The control loops (Pitch and Roll-Yaw) run on a Linux RT Kernel to ensure deterministic timing, which is critical for balancing an inherently unstable system.

* **Sensor Fusion & Filtering**: Raw IMU data is processed with digital low-pass filtering to suppress mechanical vibrations from the high-RPM gyroscope

* **State Estimation**: The filtered data is integrated into a complementary filter-based sensor fusion algorithm to provide accurate state estimation.

* **Hardware Abstraction Layer (HAL)**: Custom drivers for hardware modules (PWM, SPI, UART etc.) utilizing Memory-mapped I/O for maximum performance.

* **Telemetry Stream**: A dedicated thread handles high-frequency data logging, streaming internal states via UDP to the Python GUI for live monitoring.

## Live Data Telemetry
*Telemtry data for passing through a set of cones*

![passThroughObstacel_StateSpaceRoll](https://github.com/user-attachments/assets/859d1ce8-5a99-465d-8722-b609ca1cf337)
