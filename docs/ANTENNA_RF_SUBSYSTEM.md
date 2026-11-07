# Antenna radio and sensor-fusion options — feasibility, not deployed hardware

## Selected role for each connectivity hop (D-0014, proposed)

The active robot intelligence architecture is **rover-local deterministic safety + numerical triage + local peer packet exchange**, with **optional nearby GPU-hosted LLM advisory inference on unusual incidents**. The old research on sharding one LLM across multiple Jetsons is not a current requirement.

| Hop | Responsible for | Plausible transport; verify at the deployment site |
| --- | --- | --- |
| Rover ↔ rover / field gateway | SOS, measured local map fragments, task bids, acknowledgement and valid short commands | Surveyed Wi-Fi, Wi-Fi HaLow (IEEE 802.11ah) or another permitted local IP/packet radio; keep UWB ranging separate |
| Farm gateway ↔ shared inference server | Compact contextual rescue snapshot, numerical tool calls/results, validated proposals and audit log | LAN for farm/co-op GPU; otherwise existing 4G/LTE/5G/fixed broadband; optional **lawful satellite terminal on the gateway** when terrestrial backhaul is inadequate |
| Onboard sensor/MCU ↔ local control | Immediate safety/reflex, bounded drive/docking, no-go and watchdog | Local wired/onboard buses; **never** require satellite, cellular or an LLM token for motor cutoff |

Satellite performance or accessibility cannot be assumed from a regional map: verify the **exact farm address**, availability, local rules, plan/mobility restrictions, terminal power, mounting, weather performance and measured jitter. In the target rice-growing regions, terrestrial coverage is site-specific and may be preferable. Direct-to-robot satellite is **not** selected; there is no demonstrated satellite RF hardware. LoRaWAN may support alert packets, but its duty cycle/data rate is unsuited to streaming high-resolution camera imagery, and should not be conflated with Wi-Fi HaLow or cellular.

Network outage means fall back to locally validated numerical rescue/abstain, never reuse expired LLM advice. Handle stale packet/state ID, duplicates, partial connectivity and helper ownership. See [selected architecture, event protocol and failure matrix](SWARM_EDGE_PLAN.md#active-topology-local-autonomy--nearby-shared-llm-server-d-0014).

**Reconciliation of PR #22 (7 Nov 2026).** Simulated Policy B keeps a separate map per rover and exchanges modelled gossip/blackout messages inside **one Python/MuJoCo process**. The team has not demonstrated UWB radios, Wi-Fi HaLow, ESP-NOW hardware, Jetson inference, real distributed processes, radar ground-speed measurements, a 200 Hz MCU, or live force sensors. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) owns implementation status; [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md) owns the tests; the [robot module](../src/ricebot/robot/README.md) owns the tested **rigid docking**, not a tether.

| Proposed modality | Sourced physical capability | What needs correcting or measuring |
| --- | --- | --- |
| UWB ranging | [Qorvo DWM3000](https://www.qorvo.com/products/p/DWM3000) uses DW3110; manufacturer describes object positioning precision around **10 cm** in relevant conditions | This is not guaranteed ±5 cm in wet paddy foliage. One range reading gives distance, **not** relative bearing or a 2D location; NLOS, radio clocks and anchors matter |
| UWB angle of arrival | [DW3110 specifications](https://www.qorvo.com/products/p/DW3110) show **one antenna port** and **no built-in PDoA**. [DW3220](https://www.qorvo.com/products/p/DW3220) supports two-port PDoA but has an end-of-life notice | Two DWM3000 modules do not magically provide calibrated AoA; a synchronized dual-radio system, different chipset or independent geometry/pose estimate must be validated |
| Peer packet transport | [Espressif ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html) uses Wi-Fi vendor-specific connectionless action frames. Wi-Fi HaLow is a **different sub-GHz IEEE 802.11ah system** | The old phrase “Sub-1GHz Wi-Fi HaLow / ESP-NOW” conflates incompatible radio families; do not claim real mesh deployment or link performance |
| mmWave motion sensing | [Infineon BGT60TR13C datasheet](https://www.infineon.com/assets/row/public/documents/24/49/infineon-bgt60tr13c-datasheet-en.pdf?fileId=8ac78c8c7d718a49017d94bac88e5d43) and [processing note](https://www.infineon.com/dgdl/Infineon-AN652_Digital_Beamforming_using_the_BGT60TR13C_Radar_Sensor-ApplicationNotes-v01_40-EN.pdf?fileId=8ac78c8c8779172a0187e2848e2d0f73) describe FMCW and target radial-velocity processing | Actual **ground velocity** over slow wet/muddy surfaces requires appropriate mounting, radar returns, filtering, testing and an error distribution; not a ready-made slip-invariant sensor output |
| Motor effort and docking force | [TI INA240](https://www.ti.com/product/INA240) is a shunt **current-sense amplifier** | It is not a load-cell amplifier or direct towing-force sensor. Actual contact-force instrumentation needs separate calibrated transducer/circuit and valid model thresholds |

## Sensor-to-action design, not a promised integrated stack

1. **Local deterministic safety:** commanded and actual motor/joint rates, obstacle/contact detection, tilt and safe HOLD. No model gets direct torque control.
2. **Local state and uncertainty:** wheel odometry/independent pose, sensor-valid flags, timestamp and bounded commanded-vs-observed-motion diagnosis. A static relative UWB distance by itself is insufficient to infer immobilisation.
3. **Cooperative belief:** per-robot measurements with provenance, local maps, explicit message packets and link blackouts. The existing simulated Policy B exercises this logic; no physical RF performance can be inferred from it.
4. **Rare strategic recovery event:** evaluate risk and value of intervention, choose investigate/self-recover/dispatch/defer/abstain via a bounded skill validated by safety and capability checks. Use a calibrated deterministic optimiser as mandatory baseline for any contextual LLM.
5. **Independent verification:** real simulator contact, latch actuation and extraction, risk to second robot, crop and time cost, and remaining fieldwork. Test false sensor dropouts separately from stuck rovers.

**Do not inherit the old PR's asserted numbers as specifications:** dual-DWM3000 ±5 cm AoA, 200 Hz controller, 20° tilt abort, 0.6 wheel-slip cut-off, 350–400 N tether loads, or guaranteed row/terrain performance. Derive actuator limits from installed robot geometry, forces and physics tests, and measure actual sensor timing if ever built.

**Physical architecture remains unchanged:** Main has a **rigid cam latch/pitch-lift mechanism** and scripted tandem extraction. A tether/winch is an unverified *alternative requiring a separately approved redesign*, not something to quietly include in this research merge.

**Evidence to obtain if real hardware is pursued:** clock calibration and RF NLOS packet errors, worst-case pose/velocity estimation, radar returns on actual wet ground, hitch force sensor calibration, contact geometry, physical second-casualty risk, host latency/RAM/energy, and safe operation without a disabled device's compute or radio.

**Provenance:** preserves PR #22's fusion, IMU, ToF, radar, peer-link and effort-monitoring ideas while correcting hardware capabilities and differentiating proposed apparatus from tested simulation.
