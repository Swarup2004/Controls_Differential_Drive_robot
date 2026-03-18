# Differential Drive Robot Control

Closed-loop control of a differential drive robot using quadrature encoders.  
This repository implements two control strategies:

1. Pose-based navigation using odometry and heading PID  
2. Velocity-controlled straight-line motion with distance termination  

---
## System Overview

<p align="center">
  <img src="TOP_VIEW.JPG" width="500"/>
</p>

This is a differential drive robot platform equipped with:
- Two DC motors with quadrature encoders  
- Motor driver for independent wheel control  
- Microcontroller for real-time control and feedback  
## Features

- Encoder-based closed-loop control  
- Differential drive kinematics  
- Odometry-based pose estimation (x, y, θ)  
- PID heading control  
- PI velocity control  
- Straight-line drift correction  

---

## Repository Structure

```
.
├── pose-control/
│   └── main.ino
│
├── velocity-distance-control/
│   └── main.ino
│
├── README.md
└── LICENSE
```

---

## Hardware Setup

### Components
- Microcontroller (Arduino / STM32 / Pico)
- Dual motor driver (L298N / Cytron MD30C)
- 2 DC motors
- Quadrature encoders
- External power supply

---

## Pin Configuration

```
================ MOTOR DRIVER =================

Right Motor:
R_EN   -> Pin 6   (PWM)
R_IN1  -> Pin 7   (DIR)
R_IN2  -> Pin 8   (DIR)

Left Motor:
L_EN   -> Pin 11  (PWM)
L_IN1  -> Pin 12  (DIR)
L_IN2  -> Pin 13  (DIR)

================ ENCODERS =================

Left Encoder:
L_ENC_A -> Pin 14 (Interrupt)
L_ENC_B -> Pin 15

Right Encoder:
R_ENC_A -> Pin 16 (Interrupt)
R_ENC_B -> Pin 17

================ POWER =================

Motor Driver -> External Supply  
Controller   -> USB / Battery  
Common GND REQUIRED
```

---

## Control Modes

### 1. Pose Control

**File:** `pose-control/main.ino`

#### Objective
Drive the robot to a target position (x, y) and align to a desired final heading θ.

#### System Diagram

```
        +----------------------+
        |   Goal (x, y, θ)     |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |  Navigation Control  |
        |   (PID on heading)   |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |   (v, ω) Generator   |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |  Wheel Kinematics    |
        |  (v → vL, vR)        |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |   PI Speed Control   |
        |  (RPM tracking)      |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |       Motors         |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |      Encoders        |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |      Odometry        |
        |   (x, y, θ update)   |
        +----------------------+
                   |
                   +---- feedback loop ----+
```

#### Key Elements
- Differential drive odometry  
- Heading error regulation using PID  
- Velocity command generation (v, ω)  
- Wheel-level PI speed control  

---

### 2. Velocity-Distance Control

**File:** `velocity-distance-control/main.ino`

#### Objective
Maintain a target velocity while moving in a straight line and stop after a specified distance.

#### System Diagram

```
        +----------------------+
        |   Target RPM + Dist  |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |   PI Speed Control   |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |   Straight Correction|
        | (Encoder difference) |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |        Motors        |
        +----------+-----------+
                   |
                   v
        +----------------------+
        |       Encoders       |
        +----------+-----------+
                   |
        +----------+-----------+
        |                      |
        v                      v
+------------------+   +----------------------+
|  RPM Estimation  |   | Distance Estimation  |
+------------------+   +----------------------+
        |                      |
        +----------+-----------+
                   |
            Stop Condition
```

#### Key Elements
- Constant RPM tracking using PI control  
- Distance estimation using encoder counts  
- Drift correction using left-right encoder difference  
- Termination based on traveled distance  

---

## Kinematics Model

```
v = (vR + vL) / 2
ω = (vR - vL) / WHEEL_BASE
```

```
vR = v + (ω * WHEEL_BASE / 2)
vL = v - (ω * WHEEL_BASE / 2)
```

---

## Odometry Update

```
ds     = (dR + dL) / 2
dθ     = (dR - dL) / WHEEL_BASE

x += ds * cos(θ)
y += ds * sin(θ)
θ += dθ
```

---

## Tuning Parameters

### Speed Control
- Kp_speed  
- Ki_speed  

### Heading Control
- Kp_heading  
- Ki_heading  
- Kd_heading  

### Straight Correction
- K_straight  

---

## How to Run

1. Upload the desired `main.ino` file from the selected control mode  
2. Connect hardware as per the pin configuration  
3. Open Serial Monitor (115200 baud)  
4. Observe runtime data (RPM, PWM, pose or distance)  

---

## Notes

- Ensure correct interrupt-capable pins for encoder inputs  
- Incorrect CPR value will affect distance estimation  
- Poor gain tuning may cause oscillations or drift  
- Always ensure a common ground between all components  

---

## Future Improvements

- IMU integration for improved heading estimation  
- PID auto-tuning  
- Path planning and waypoint tracking  
- ROS2 integration  

---

## Author

Swarup Patil  
Robotics | Control Systems | Embedded Systems  

---

## License

MIT License
