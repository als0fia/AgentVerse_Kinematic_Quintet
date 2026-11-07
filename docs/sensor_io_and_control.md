# Sensor I/O and Closed-Loop Control Architecture

| Sensor / Subsystem | Physical Input | Processed Output | Direct Robot Action and Control Effect |
|---|---|---|---|
| **IMU + AHRS**<br>`ICM-42688-P`<br>`MMC5983MA` | Angular rates (w), linear accels (a), magnetic flux (B). | Pitch, Roll, Heading, vertical sink. | • Pitch/Roll > 20 deg -> Motor cut to prevent rollover.<br>• z-accel drop -> Detects chassis belly sink in mud.<br>• Un-tilts UWB antenna reference frame. |
| **Ground Speed Radar**<br>`Infineon BGT60TR13C` | 60GHz FMCW radar reflection off mud bed. | True ground velocity (v_true). | • Computes Slip Ratio s = 1 - (v_true / wr).<br>• s > 0.6 -> Throttles down RPM to stop digging trenches.<br>• Gating filter for UWB multipath spikes. |
| **Row Whiskers + ToF**<br>`Hall-Effect`<br>`VL53L8CX` | Stem contact deflection + multizone IR bounce. | Lateral cross-track error (e_y) from row center. | • Closed-loop PD steering: Delta_w = Kp * e_y + Kd * de_y/dt.<br>• Keeps robot centered between rice hills without vision GPUs. |
| **Effort and Load**<br>`INA240`<br>`Hitch Strain Gauge` | Motor current shunt drop + hitch pin strain. | Soil torque (tau_mud) and cable tension (F_pull). | • F_pull > 350 N -> Lead robot boosts torque to tow follower.<br>• F_pull < 20 N -> Winch auto-reels slack.<br>• Cutoff at 400 N to prevent line snap. |
| **UWB Swarm Ranging**<br>`2x Qorvo DWM3000` | RF chirp timestamps and phase difference on dual antennas. | Peer distance d_ij (+/- 5 cm) and Angle-of-Arrival (AoA). | • Regulates convoy following distance (e_x).<br>• Collision halt if d_ij < 1.0 m.<br>• Relative bearing for coordinated winch pulls. |
| **Swarm Comms**<br>`Wi-Fi HaLow`<br>`ESP-NOW` | Sub-1GHz RF telemetry packets through wet leaves. | Peer state vectors, slip indices, consensus tokens. | • Swarm triage: triggers cooperative pull if neighbor flags stuck.<br>• Synchronizes AgentVerse LLM reasoning state. |
