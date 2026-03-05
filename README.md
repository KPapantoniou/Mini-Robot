
### Modeling, Simulation, and Autonomous Control

> Master's Thesis — *K. Papantoniou*
> 📄 [Read the full thesis](presentation_thesis/thesis_KPapantoniou.pdf)

---

## What Is This?

Most robots move using wheels, legs, or tracks. This one moves using **vibrations**.

By spinning two small eccentric masses at controlled speeds, the robot generates asymmetric centrifugal forces that cause it to inch forward, backward, and rotate — with no conventional drivetrain. The result is an ultra-compact, mechanically simple platform capable of autonomous 2D navigation.

This repository contains everything built for the thesis: the physics simulation that validated the model, the embedded firmware and PC-side controller that runs on real hardware, and the computer vision pipeline that lets the robot navigate autonomously to a user-defined target.

---

## How the Pieces Fit Together

The project is structured in three layers that build on each other:

```
┌─────────────────────────────────────────────────────┐
│               SIMULATION (C++)                      │
│  Physics model · Motor dynamics · PID tuning        │
│  Validates behavior before touching hardware        │
└──────────────────────┬──────────────────────────────┘
                       │ validated parameters
                       ▼
┌─────────────────────────────────────────────────────┐
│            EMBEDDED CONTROLLER (ESP32)              │
│  Firmware receives UDP commands from PC             │
│  PID loop maintains target RPM per motor            │
└──────────────────────┬──────────────────────────────┘
                       │ position & orientation
                       ▼
┌─────────────────────────────────────────────────────┐
│         COMPUTER VISION CONTROLLER (C++/Python)     │
│  Camera detects robot markers in real time          │
│  Macro + Micro control closes the navigation loop   │
└─────────────────────────────────────────────────────┘
```

The simulation de-risks the design: motor dynamics, PID parameters, and force profiles are all validated in software first. Those parameters then go directly into the firmware. The vision system closes the outer loop, giving the robot a sense of where it is in the world.

---

## Repository Structure

```
├── simulation/              # C++ physics simulation
│   ├── src/                 # Motor, robot, force, PID models
│   ├── ploter/              # Python visualization scripts
│
├── controller/
│   ├── camera_controller/   # PC-side C++ vision & control app
│   └── esp32_controller/    # ESP32 firmware (Arduino/PlatformIO)
│
├── presentation_thesis/     # Thesis PDF and presentation slides
└── cad/                     # CAD schematics of the physical platform
```
---
## The Three Components

### 🧠 Physics Simulation
*[Full details → simulation/README.md]*

The simulation models the complete electromechanical system: DC motor electrical dynamics, eccentric mass rotation, centrifugal force generation, and the resulting robot motion in 1-DOF and 2-DOF. A Simulated Annealing optimizer searches for optimal PID parameters automatically and compairs them with a PI controller with parameters that are provided by solving the linear second degree system of the close loop controller.

Key outputs: motor force profiles, robot trajectory under various control schemes, settling time analysis, and PWM-to-voltage mapping.

**Stack:** C++ · CMake · Python 

---
### ⚙️ Embedded Firmware (ESP32) + Autonomus Vision Controller
*[Full details → controller/README.md]*

The ESP32 firmware handles everything on the robot side: Wi-Fi connection, UDP command reception, dual motor PWM control, and a per-motor PID loop that maintains target RPM using encoder feedback. It also streams real-time RPM data back to the PC for monitoring.

A C++ application captures live camera feed, detects colored markers on the robot using HSV-space thresholding, and runs a two-tier control strategy

Commands are simple single-character tokens (`F`, `B`, `L`, `R`, `S`) sent over UDP from the PC controller.

**Stack:** C++ · Arduino/PlatformIO · WiFiUDP · OpenCV · CMake

---
## Results

The full experimental results, parameter derivations, and performance analysis are documented in the thesis:

📄 [thesis_KPapantoniou.pdf](presentation_thesis/thesis_KPapantoniou.pdf)

---

## License

Academic project — Master's Thesis. Contact the author for reuse inquiries.
