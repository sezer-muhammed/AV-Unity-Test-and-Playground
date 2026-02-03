# Prometheus Advanced Vehicle Simulation (TestAV)

A high-fidelity, modular vehicle physics backbone for Unity, bridging the gap between game development and realistic simulation. This project features a custom-built powertrain system, SAE-standard slip calculations, and a robust telemetry logging suite.

## 🏎️ Core Architecture
The project is built on a modular "Component-Link" architecture, separating mechanical concerns into specialized classes for maximum precision and extensibility.

| Component | Responsibility | Key Physics Logic |
| :--- | :--- | :--- |
| **Engine** | Torque Generation | SmoothDamp RPM modeling, AnimationCurve torque LUT, Engine Drag |
| **Transmission** | Gear Reduction | Automatic R/N/D modes, shift time simulation, Torque Converter fade |
| **Drivetrain** | Power Distribution | AWD (25% per wheel), Drivetrain Elasticity (Jerk reduction) |
| **Telemetry** | Data Acquisition | 50Hz FixedUpdate logging, SAE Slip Ratio, JSON serialization |

## 📊 Default Simulation Parameters (The "Wow" Specs)

The vehicle comes pre-tuned with balanced "Performance Sedan" metrics. Every value is exposed for real-time optimization.

### Engine & Powertrain
| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| **Idle RPM** | 850 RPM | Baseline engine rotation without throttle input. |
| **Redline RPM** | 6,200 RPM | Maximum safe engine speed; rev limiter engages here. |
| **Peak Torque** | 430 Nm | Maximum torque output at 3,600 RPM (via Torque Curve). |
| **Engine Inertia** | 0.25 kg·m² | Resistance to RPM change; creates "weight" in engine response. |
| **Engine Drag** | 80 Nm | Internal friction/resistance applied when off-throttle. |
| **Stall RPM** | 2,100 RPM | Torque converter stall point for aggressive launches. |

### Transmission (4-Speed Automatic)
| Gear | Ratio | Shift Logic |
| :--- | :--- | :--- |
| **1st** | 2.87 | Upshift @ 5,800 RPM |
| **2nd** | 1.89 | Upshift @ 5,800 RPM |
| **3rd** | 1.28 | Upshift @ 5,800 RPM |
| **4th** | 1.00 | Downshift @ 2,400 RPM |
| **Final Drive**| 3.73 | Combined axle reduction ratio. |
| **Shift Time** | 0.45s | Time duration where drivetrain lock is 0% during shifts. |

### Physical Environment Resistance
| Variable | Value | Physical Impact |
| :--- | :--- | :--- |
| **Aero Drag (Cd)** | 0.32 | Air resistance coefficient (similar to a modern sports car). |
| **Frontal Area** | 2.05 m² | Projected surface area for drag force calculation ($F = 0.5 \cdot \rho \cdot v^2 \cdot C_d \cdot A$). |
| **Rolling Res.** | 12.5 | Tire/Road friction coefficient (standard asphalt). |
| **Weight Transfer**| 0.22 | Longitudinal suspension pitch factor during accel/brake. |

## 🧠 Advanced Simulation Logic

### 1. The R/N/D Gear System
Unlike arcade racers, this system requires intentional gear selection via **'Q'**.
*   **Safety Interlock:** The transmission logic prevents shifting into Reverse (R) or Drive (D) if the car is moving faster than **1 km/h**, protecting the virtual gearbox.
*   **Automatic Creep:** When in 'D' or 'R' with zero throttle, the engine applies **90 Nm** of "Creep Torque" up to **8 km/h**, simulating a torque converter's fluid coupling.

### 2. Physical Braking Model
Braking isn't a simple "stop" command. It applies a calculated **4,200 Nm** of resistance torque across the wheels.
*   **Brake Bias (65% Front):** Ensures stable deceleration by loading the front tires.
*   **Anti-Lock Resistance:** The "S" key applies torque that resists rotation rather than instantly setting velocity to zero, allowing for realistic weight transfer and tire heat simulation (future).

### 3. SAE Slip Ratio Calculation
We calculate wheel slip using the **SAE J670** standard:
$$\text{Slip Ratio} = \frac{\text{Wheel Linear Velocity} - \text{Ground Velocity}}{\max(1.0, \text{Ground Velocity})}$$ 
This provides high-fidelity data for the built-in **Traction Control (TC)** which reduces engine torque when slip exceeds **0.25**.

## 🎮 Control Mapping
*   **[W] Throttle:** Forward pressure in 'D', backward pressure in 'R'.
*   **[S] Brake:** Physical resistance torque (All 4 wheels).
*   **[Q] Gear Mode:** Cycle **Drive -> Neutral -> Reverse**.
*   **[Space] Handbrake:** Mechanical lock of rear axles (**6,500 Nm**).

## 🚀 Telemetry
Every frame is recorded to `VehicleTelemetry.json`. This data is compatible with external PID optimization tools or AI training scripts.