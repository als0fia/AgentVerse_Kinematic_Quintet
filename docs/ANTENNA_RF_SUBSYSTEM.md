# Antenna & RF Subsystem — Sensor Fusion Architecture

**Scope Note:** This specification defines the physical hardware and multi-sensor fusion architecture for the **Antenna** multi-robot paddy platform in the **AgentVerse Kinematic Quintet** architecture (target embodied hardware / physical subsystem specification; distinct from the pure-Python `rice.v1` contract tests in `src/ricebot/`). See [ARCHITECTURE](ARCHITECTURE.md) and [COMMERCIALIZATION_CHINA](COMMERCIALIZATION_CHINA.md).

---

In the AgentVerse Kinematic Quintet multi-robot paddy architecture, the **Antenna and RF subsystem** (comprising the dual Qorvo DWM3000 UWB transceivers and the Sub-1GHz Wi-Fi HaLow / ESP-NOW antenna) forms the backbone of the relative localization, formation keeping, and collaborative state estimation.

Below is the breakdown of how the antenna signals are fused with the rest of the sensory suite:

## 1. Dual-Antenna UWB Array Fusion (Relative Pose Estimation)

Each robot utilizes two Qorvo DWM3000 transceivers separated by a calibrated baseline on the chassis/mast:

- **Measurement Modalities:**
  - **DS-TWR (Double-Sided Two-Way Ranging):** Delivers peer-to-peer distance $r_{ij}$ ($\pm 5\text{ cm}$).
  - **PDoA (Phase Difference of Arrival):** Computes Angle-of-Arrival ($\text{AoA} / \theta$) in the robot’s horizontal plane.
- **Fusion with 9-DOF IMU/AHRS (ICM-42688-P + MMC5983MA):**
  - **Tilt/Attitude Compensation:** Pitch and roll angles measured by the IMU are used to project the antenna baseline vector into the local horizontal frame, preventing pitch/roll sinkage in mud from distorting the AoA bearing angle.
  - **Heading Alignment:** Fused with the MMC5983MA magnetometer heading to project peer-to-peer relative vectors into a shared global/field coordinate frame.

## 2. Antenna vs. Odometry Fusion (NLOS & Multipath Filtering)

Dense, wet rice canopy causes Non-Line-of-Sight (NLOS) delays and RF multipath reflections. The system filters these using kinematic sensors:

- **mmWave Radar (Infineon BGT60TR13C) + Wheel Encoders:**
  - Doppler radar provides true ground speed ($v_{\text{true}}$) invariant to wheel slip in mud.
  - An Extended Kalman Filter (EKF) / Factor Graph predicts the expected inter-robot range increment:
    $$\Delta r_{ij} \approx \int (v_i - v_j) \cdot \hat{u}_{ij} \, dt$$
  - Any sudden jump in UWB antenna range that deviates significantly from the radar/odometry dead-reckoning envelope is rejected as an NLOS reflection or canopy multipath glitch.

## 3. Antenna + Tow Hitch Strain Gauge (Tether Kinematics Fusion)

In the Kinematic Quintet tethered/winch topology, physical cables link units for traction assistance and self-recovery:

- **Strain Gauge (INA240 / Load Cell on Tow Hitch):** Measures pull tension $F_{\text{pull}}$ (up to $400\text{ N}$).
- **Sensor Fusion:**
  - The UWB antenna distance between the lead and trailing robot is fused with the tether reel encoder/winch line length.
  - This validates whether the cable is under tension ($F_{\text{pull}} > \text{threshold}$) or slacked, maintaining safe formation spacing and preventing the tether from becoming entangled in the mud or crop rows.

## 4. Antenna + Row Tracking (Boundary & Cross-Track Fusion)

- **Whiskers + ToF (VL53L8CX):** Provide high-frequency analog cross-track error ($e_y$) relative to rice plant stems.
- **UWB Antenna:** Provides longitudinal inter-robot following distance ($e_x$).
- **Fusion:** Decouples navigation into:
  1. **Lateral controller:** Whiskers + ToF directly lock the robot inside the narrow furrow.
  2. **Longitudinal/Swarm controller:** UWB antenna ranging maintains formation stagger and prevents collisions between the quintet members.

## 5. Multi-Rate Compute Split & Telemetry Sync

- **Low-Level (STM32H7 @ 200 Hz):**
  - Direct SPI processing of IMU, radar Doppler, and real-time antenna UWB timestamps (TWR/AoA).
  - Executes the high-rate strapdown INS/odometry error-state propagation.
- **Mid-Level (SBC: Raspberry Pi 5 / Jetson Orin Nano):**
  - Ingests local state vectors from the STM32 via high-speed UART/CAN.
  - Sends/receives peer state covariance packets over the Wi-Fi HaLow / ESP-NOW antenna to run the distributed multi-robot swarm consensus and LLM reasoning agent loops.
