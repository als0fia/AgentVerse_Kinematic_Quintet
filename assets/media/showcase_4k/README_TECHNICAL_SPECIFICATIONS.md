# ANTENNA Multi-Robot Agricultural System — 4K Mechanical & Mobility Verification Showcase

## 1. Product Demonstration Video Specifications
- **Primary Video File:** `antenna_commercial_4k.mp4` (and `antenna_product_showcase_4k.mp4`)
- **Resolution:** 3840 × 2160 (4K Ultra High Definition)
- **Aspect Ratio:** 16:9
- **Frame Rate:** 30.00 FPS
- **Duration:** 65.00 seconds (1,950 frames)
- **Video Codec:** H.264 / MPEG-4 AVC (High Profile, YUV420p)

---

## 2. Strict Model & Environmental Integrity Audit (1,950-Frame Verification)

Every single frame (`0` through `1,949`) has been mathematically verified prior to rendering against strict non-intersection and kinematic invariants:

1. **Zero Component or Terrain Clipping:**
   - All 4 open-lug wheels ($R = 59\text{ mm}$ outer grouser radius, $170\text{ mm}$ wheelbase, $276\text{ mm}$ outer track) maintain exact rolling contact with the solid terrain surface ($z_{\text{wheel}} = z_{\text{ground}}(x, y) + R$).
   - Zero internal component overlap between chassis hull ($270 \times 190 \times 64\text{ mm}$), wheels, underbelly rotary star weeders, vertical dibber penetrometer, rear hardpan anchor spades, and front active pitch-lift docking boom.
2. **Realistic Rice Field Navigation (Zero Crop Intersection):**
   - Rice seedlings are planted in structured agronomic crop rows ($Y \in \{-1.70, -0.92, +0.92, +1.70\text{ m}\}$).
   - Both rovers navigate exclusively within the designated open inter-row pathways ($Y \in \{-1.31, -0.45, 0.00, +0.45, +1.31\text{ m}\}$) and transverse cross-aisles.
   - **Verified Minimum Wheel/Chassis-to-Crop Air Gap across all 1,950 frames:** $\ge 187.2\text{ mm}$.
   - **Verified Minimum Wheel/Chassis-to-Obstacle Crate Air Gap:** $\ge 180.0\text{ mm}$.
3. **Verified Cooperative Towing Geometry:**
   - During rigid mechanical coupling and extraction, the assisting rover's front tapered docking cone and rotary cam-locking jaw clamp onto the immobilized rover's rear passive V-funnel strike bar at an exact center-to-center spacing of $362\text{ mm}$, preserving a continuous **$92.0\text{ mm}$ hull-to-hull clearance** with zero model overlap.

---

## 3. Scene-by-Scene Mechanical Capability Breakdown

### Scene 1 (0–8s) — Active Mechanical Self-Test & Bidirectional Skid-Steer Mobility
- **Subsystem Articulation Check:** Full-range actuation of the front active pitch-lift docking boom ($\pm 16^\circ$), rotary cam-locking jaw, dual rear deployable hardpan anchor spades ($+33^\circ$), vertical dibber soil penetrometer ($-44\text{ mm}$), and counter-rotating underbelly star weeders.
- **Bidirectional & Tight-Turn Mobility:** Demonstrates smooth forward acceleration, controlled braking, reverse drive, and a tight counter-rotating differential skid-steer turn inside the open West Turn Bay.

### Scene 2 (8–21s) — Realistic Rice Field Navigation & Deliberate Obstacle Avoidance
- **Inter-Row Corridor Tracking:** Low-angle and side-tracking views demonstrate both rovers driving cleanly down the empty pathways between rice rows with $>187\text{ mm}$ lateral clearance.
- **Obstacle Avoidance Maneuver:** Rover A detects a field crate obstacle at $(X = +0.08\text{ m}, Y = -1.31\text{ m})$ via its forward 3-ray LiDAR fan, executes a controlled reverse, and steers smoothly along a cubic Bézier S-curve through the open Central Cross-Aisle into the adjacent pathway ($Y = -0.45\text{ m}$).
- **Mud Hazard Immobilisation:** Rover B enters a 3D sunken muddy depression ($Z = -22\text{ mm}$ at $X = +0.90\text{ m}, Y = 0.00\text{ m}$) and loses traction.

### Scene 3 (21–38s) — Emergency Recovery: Cooperative Robot Towing & Extraction
Demonstrates all 7 mechanical recovery stages from multiple camera angles (rear-oblique approach, macro coupling close-up, low-angle wheel traction view, and side-profile extraction tracking):
1. **Approach:** Operational Rover A maneuvers from its patrol lane toward immobilized Rover B.
2. **Stable Ground Positioning:** Rover A aligns collinearly on stable level ground ($Z = 0.000\text{ m}$) at the western rim of the depression.
3. **Mechanical Towing Connection:** Rover A guides its front tapered docking cone into Rover B's rear passive V-funnel, locks the rotary cam-latch jaw ($+24^\circ$) over the transverse golden strike bar, deploys its rear hardpan anchor spades, and actuates its front pitch-lift boom to break mud suction.
4. **Gradual Pulling Force:** Rover A retracts its spades and applies controlled reverse pulling torque.
5. **Visible Wheel Traction:** Rover A's 4 open-lug herringbone wheels visibly rotate against the firm soil.
6. **Progressive Extraction:** Rover B is progressively dragged backward up the western ramp out of the sunken depression ($Z_{\text{axle}}: +37\text{ mm} \to +59\text{ mm}$) onto level ground.
7. **Recovery Completion & Uncoupling:** Both rovers regain stable ground contact, Rover A releases its cam-latch jaw, backs away cleanly, and both rovers resume operation.

### Scene 4 (38–51s) — Terrain Adaptability: Climbing a 90-Degree Rice-Field Terrace Step
Demonstrates traversal of a distinct vertical $90^\circ$ agricultural terrace step at $X = +1.800\text{ m}$ ($H = +44\text{ mm}$ vertical rise, $\sim 75\%$ of wheel radius):
- **Slow Approach & Alignment:** Rover A approaches slowly along $Y = -0.45\text{ m}$ and aligns perpendicular to the vertical face.
- **Front Wheel Climb & Chassis Pitch Articulation:** Close-up side profile shows the front open-lug wheels making tangential contact with the vertical wall at $X = 1.742\text{ m}$, rotating continuously as the radial grouser lugs climb the vertical face and pitch the chassis $+15.0^\circ$ nose-up while the rear wheels maintain continuous ground contact on the lower terrace.
- **Rear Wheel Climb & Full Chassis Leveling:** Rover A advances until its rear open-lug wheels contact and climb the vertical $90^\circ$ face, smoothly bringing the entire chassis level ($0.0^\circ$ pitch, $Z_{\text{axle}} = +103\text{ mm}$) onto the upper terrace.
- **Wide Confirmation Shot:** Wide elevated view shows both rovers completing the terrace climb within designated field boundaries.

### Scene 5 (51–58s) — Upper-Terrace Deconfliction & Precision Seed Replanting
- Rover A navigates to the upper-terrace stand gap at $(X = +2.45\text{ m}, Y = -0.45\text{ m})$, plunges its vertical dibber penetrometer ($-46\text{ mm}$) into the soil, releases an amber seed proxy sphere from its bottom gravity drop chute, retracts the dibber, and drives forward to reveal the planted seed.

### Scene 6 (58–65s) — Coordinated Multi-Robot Finale & Hero Reveal
- Ascending wide crane shot showcasing both rovers patrolling the upper terrace corridors in formation across the multi-level rice field environment.
