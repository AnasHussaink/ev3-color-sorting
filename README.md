# 🤖 EV3 Manipulator Robot — Color Sorting with Conveyor

Project Highlights
✔️ Developed a 3-DOF robotic arm model using geometric inverse kinematics for accurate positioning
✔️ Implemented a robust homing mechanism using touch sensors for consistent reference alignment
✔️ Engineered real-time RGB color detection & calibration (Black, Green, Red, Blue)
✔️ Designed an intelligent sorting algorithm:
Black → Station 3
Green → Station 4
Blue → Station 1
Red → Station 2
✔️ Executed seamless pick-and-place operations synchronized with conveyor motion
✔️ Integrated PID control principles to enhance motion stability and system responsiveness

![Platform](https://img.shields.io/badge/Platform-LEGO%20EV3%20MicroPython-yellow)
![Language](https://img.shields.io/badge/Language-MicroPython-blue)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)
![Lab](https://img.shields.io/badge/Lab-Mechatronics-orange)

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Hardware Description](#-hardware-description)
- [Kinematic Model](#-kinematic-model)
- [Software Features](#-software-features)
- [Color Sorting Logic](#-color-sorting-logic)
- [PID Controller](#-pid-controller)
- [Getting Started](#-getting-started)

---

## 🎯 Project Overview

This project implements an **automated color-sorting mechatronic system** using a LEGO EV3 Mindstorms manipulator robot integrated with a conveyor belt. The system detects the color of balls on the conveyor using a color sensor and performs the following actions:


<img width="1208" height="699" alt="image" src="https://github.com/user-attachments/assets/091b6efd-d201-4f70-84bb-497c3c9d83ac" />

In this project, you will remotely or physically access the manipulator in Figure 1 and program it in MicroPython to achieve a set of pick-and-place tasks.


The manipulator is driven by three actuators(See Figure 2):

Three motors
In addition to the three actuators, there are also five sensors (See Figure 2):

Two touch sensors
Three encoders

<img width="820" height="703" alt="image" src="https://github.com/user-attachments/assets/a470ae37-6130-43fe-addb-ff0a0bdf04e3" />

In addition to the manipulator the conveyor has one motor with encoder and one color sensor.

The manipulator is made of five links (See Figure 3):

Link-0 (to be measured by you)
Link-1 of 50mm
Link-2 of 95mm
Link-3 of 185mm
Link-4 of 110mm
 

It is kinematically constrained and among kinematic constraints include:

Link-1 is rigidly connected to link-2 at an obtuse angle of 135°
Link-4 is always perpendicular to the ground plane
The rotation of link-3 is bound by link-2 on the lower side and a touch sensor on the upper side
The base rotation is bound by a touch sensor on one side

<img width="998" height="826" alt="image" src="https://github.com/user-attachments/assets/cbd3b49b-4c1b-4a0a-a375-49ca7eb99692" />

<img width="1359" height="521" alt="image" src="https://github.com/user-attachments/assets/b3f071ca-0fe2-4b4f-8ef9-6e68f98286dd" />

The operating area is made of 5 stations / locations (See Figure 5). 

 Station 1: This is the position in which the touch sensor of the base is pressed. Height is Z = 0
 Station 2: This is the position on the opposite side of Station 1. Height is Z =0
 Station 3: This is the position on the left end side of the conveyor.
 Station 4: This is the position on the right end side of the conveyor.
 Station 5: This is the position in the middle of the conveyor. Height Z must be selected by the user. 

 <img width="1275" height="783" alt="image" src="https://github.com/user-attachments/assets/225f83fa-ba67-43b4-961a-f9593507d0dc" />

 



| Color | Action |
|-------|--------|
| 🖤 **Black** | Conveyor transports ball to **Station 3** (left end) |
| 💚 **Green** | Conveyor transports ball to **Station 4** (right end) |
| 🔵 **Blue** | Conveyor stops at **Station 5** → Arm picks and places at **Station 1** |
| 🔴 **Red** | Conveyor stops at **Station 5** → Arm picks and places at **Station 2** |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    EV3 Brick (Controller)                │
│                                                         │
│  Port A → Gripper Motor     Port S1 → Base Touch Sensor │
│  Port B → Arm Motor         Port S2 → Arm Touch Sensor  │
│  Port C → Base Motor        Port S3 → Color Sensor      │
│  Port D → Conveyor Motor                                │
└─────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────┐    ┌──────────────────────────────────┐
│  Conveyor Belt  │    │     5-DOF Articulated Arm        │
│  - Color detect │    │  Base → Arm → Gripper            │
│  - 4 stations   │    │  Inverse Kinematics (geometric)  │
└─────────────────┘    └──────────────────────────────────┘
```

---

## 🔧 Hardware Description

### Actuators
| Motor | Port | Function |
|-------|------|----------|
| Motor 1 | A | Gripper (open/close) |
| Motor 2 | B | Arm elevation (θ₂) |
| Motor 3 | C | Base rotation (θ₁) |
| Motor 4 | D | Conveyor belt |

### Sensors
| Sensor | Port | Function |
|--------|------|----------|
| Touch Sensor 1 | S1 | Base homing reference |
| Touch Sensor 2 | S2 | Arm homing reference |
| Color Sensor | S3 | Ball color detection |

### Gearboxes
- **Base (Joint 1):** Gear ratio = 36:12 = **3.0**
- **Arm (Joint 2):** Gear ratio = 40:8 = **5.0**

### Link Dimensions

| Link | Length | Notes |
|------|--------|-------|
| Link-0 | 40 mm | Base height (ground → Joint 1) |
| Link-1 | 50 mm | Rigidly connected to Link-2 at 135° |
| Link-2 | 95 mm | Connected at 135° to Link-1 |
| Link-3 | 185 mm | Main reach link |
| Link-4 | 110 mm | Always perpendicular to ground |

### Workspace Stations

| Station | Description | Height Z |
|---------|-------------|----------|
| Station 1 | Base touch sensor home (reference) | Z = 0 |
| Station 2 | Opposite side of Station 1 | Z = 0 |
| Station 3 | Left end of conveyor | — |
| Station 4 | Right end of conveyor | — |
| Station 5 | Middle of conveyor (pick point) | User-defined |

---

## 📐 Kinematic Model

### θ₁ — Base Rotation (Z-axis)

θ₁ is determined from the top-view geometry using the arctangent of the target Y and X coordinates:

```
θ₁ = arctan(y / x)
```

### θ₂ — Arm Elevation

The effective arm length is computed as:

```
L12 = sqrt(L1² + L2² - 2·L1·L2·cos(135°))
L_Arm = L12 + L3
```

θ₂ is derived from the vertical displacement relative to the base:

```
dz = z - L0
θ₂ = -arcsin(dz / L_Arm)
```

Two geometric configurations exist depending on whether Link-3 is above or below the horizontal axis of Joint-2:

**Condition 1** (Link-3 below horizontal):
```
θ₂ = 45° - arcsin((L1 + 55 + L2/√2 - Z - L4) / L3)
```

**Condition 2** (Link-3 above horizontal):
```
θ₂ = arcsin((L4 - 55 - L2/√2 + Z - L1) / L3) + 45°
```

### θ₃ — Gripper

θ₃ controls the gripper open/close and does **not** affect Cartesian end-effector position.

---

## 💻 Software Features

### 1. Homing Procedure
- Base motor rotates until `base_home` (S1) touch sensor is triggered
- Arm motor rotates until `arm_home` (S2) touch sensor is triggered
- All encoder angles reset to zero — establishes repeatable reference frame
- Gripper initializes to closed position

### 2. Color Calibration
- On startup, user places each ball (Blue → Black → Red → Green) in front of color sensor
- System records 20 RGB samples per color and averages them
- Tolerance-based matching (±20 per channel) used for real-time identification

### 3. Ball Sorting
```python
if color == BLACK  → conveyor.run_angle(80, -700)   # → Station 3
if color == GREEN  → conveyor.run_angle(80, +70)    # → Station 4
if color == RED    → conveyor + arm pick & place     # → Station 2
if color == BLUE   → conveyor + arm pick & place     # → Station 1
```

### 4. Pick-and-Place Sequence (Red/Blue)
1. Open gripper
2. Move arm to pickup position above conveyor
3. Close gripper (grasp ball)
4. Lift arm to safe travel height
5. Rotate base to target station
6. Lower arm to drop height
7. Open gripper (release ball)
8. Return arm to safe position
9. Re-home robot

---

## 🎛️ PID Controller

A PID (Proportional-Integral-Derivative) controller minimizes position error in motor control:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

| Variant | Behaviour | Best For |
|---------|-----------|----------|
| **P** | Fast response, may have steady-state error | Simple position loops |
| **PI** | Eliminates steady-state error | Precise positioning |
| **PD** | Reduces overshoot, improves stability | Fast dynamic systems |
| **PID** | Combines all three for smooth, accurate control | Full motor control |

In this project the EV3 built-in motor controller uses encoder feedback internally. The `run_angle` and `run_target` methods apply PID-like closed-loop control under the hood.

---

## 🚀 Getting Started

### Prerequisites

- LEGO MINDSTORMS EV3 Brick
- [LEGO MINDSTORMS EV3 MicroPython](https://education.lego.com/en-us/product-resources/mindstorms-ev3/teacher-resources/python-for-ev3) (pybricks-micropython)
- Visual Studio Code with the **LEGO MINDSTORMS EV3 MicroPython** extension
- microSD card (flashed with ev3dev image)

### Installation

1. **Clone this repository:**
   ```bash
   git clone https://github.com/YOUR_USERNAME/ev3-color-sorting.git
   cd ev3-color-sorting
   ```

2. **Open in VS Code:**
   ```bash
   code .
   ```

3. **Connect EV3 via USB**, then use the VS Code EV3 extension to download and run.

4. **Flash the microSD card** with the pybricks/ev3dev image if not already done.
   - Download from: https://pybricks.com/ev3-micropython/

### Running the Program

1. Power on the EV3 brick
2. Open `src/main.py` in VS Code
3. Download to EV3 using the extension
4. The robot will:
   - Auto-home all joints
   - Guide you through 4-color calibration (10 seconds per color)
   - Begin sorting in a 40-iteration loop

---

## 🏷️ Keywords

`LEGO EV3` `MicroPython` `Pybricks` `Mechatronics` `Robotics` `Manipulator` `Pick-and-Place` `Color Sorting` `Inverse Kinematics` `Conveyor Belt` `Embedded Systems` `Automation` `PID Controller` `University of Siegen` `EV3 MicroPython` `Industrial Robotics` `Articulated Robot` `Touch Sensor` `Color Sensor` `Geometric Kinematics`

---

*University of Siegen · Mechatronics System Laboratory · WS 2025–2026*
