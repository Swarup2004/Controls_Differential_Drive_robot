# Differential Drive Robot Control

Closed-loop control of a differential drive robot using quadrature encoders.  
Implements two control strategies:
1. Position + Heading (θ) control using odometry and PID  
2. ADAS-style distance control using velocity PI and straight-line correction  

---

## 🚀 Features

- Encoder-based closed-loop control  
- Differential drive kinematics  
- Odometry-based pose estimation (x, y, θ)  
- PID heading control  
- PI velocity control  
- Straight-line error correction  

---

## 📂 Repository Structure

```
.
├── position_perf_theta/
│   └── position_perf_theta.ino
│
├── vel_theta_control/
│   └── vel_theta_control.ino
│
└── README.md
```

---

## 🔌 Hardware Setup

### Components
- Microcontroller (Arduino / STM32 / Pico)
- Dual motor driver (L298N / Cytron MD30C)
- 2 DC motors
- Quadrature encoders
- External power supply

---

## 📌 Pin Configuration

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

## ⚙️ Control Modes

---

### 1. Position + Theta Control

**File:** `position_perf_theta/position_perf_theta.ino`

#### Objective
Drive robot to a target position (x, y) and align to desired heading θ.

#### Control Pipeline
```
Encoders → Odometry → (x, y, θ)
          ↓
   Navigation Controller
          ↓
      (v, ω)
          ↓
 Wheel Velocities → RPM → PI → PWM
```

---

### 2. ADAS Distance Control

**File:** `vel_theta_control/vel_theta_control.ino`

#### Objective
Move forward a fixed distance with constant velocity and maintain straight path.

#### Control Pipeline
```
Encoders → RPM Estimation
          ↓
     PI Speed Control
          ↓
 Straight Error Correction
          ↓
         PWM
```

---

## 🧠 Kinematics Model

```
v = (vR + vL) / 2
ω = (vR - vL) / WHEEL_BASE
```

```
vR = v + (ω * WHEEL_BASE / 2)
vL = v - (ω * WHEEL_BASE / 2)
```

---

## 📏 Odometry Update

```
ds     = (dR + dL) / 2
dθ     = (dR - dL) / WHEEL_BASE

x += ds * cos(θ)
y += ds * sin(θ)
θ += dθ
```

---

## ⚙️ Tuning Parameters

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

## 🧪 How to Run

1. Upload desired `.ino` file  
2. Connect hardware as per pin configuration  
3. Open Serial Monitor (115200 baud)  
4. Observe RPM, PWM, distance or pose  

---

## ⚠️ Notes

- Ensure correct interrupt pins for your board  
- Wrong CPR → incorrect distance estimation  
- Poor tuning → oscillations or drift  
- Always use common ground  

---

## 🔥 Future Improvements

- IMU integration  
- PID auto-tuning  
- Path planning  
- ROS2 integration  

---

## 👤 Author

Swarup Laxmikant  
Robotics | Control Systems | Embedded Systems  

---

## 📜 License

MIT License
