/**
 * @file src/types/antenna.ts
 * @description Domain Type Definitions & Versioned Contracts (`rice.v1` & `antenna.dashboard.v1`) for Antenna.
 *
 * This file defines the data structures across the Antenna Swarm Control Console, aligned with:
 * - `src/ricebot/viewer/README.md` & `dashboard.py`:
 *   - `BeliefPerspective`: `'planter'` | `'scout'` | `'divergence'` | `'truth'`
 *   - `ContinuousHeatmapLayer`: `'none'` | `'moisture'` | `'firmness'` | `'plant_density'` | `'stand_gap'` | `'uncertainty'` | `'traversability_risk'` | `'elevation_m'`
 *   - `BekkerSinkageState`: Analytical Bekker–Wong wheel–mud sinkage (`z_mm`), belly clearance (`belly_clearance_mm`),
 *     mud drag resistance (`drag_force_n`), and traversability verdict.
 *   - `MycelialGossipPacketLog`: Peer-to-peer UWB/HaLow gossip packet provenance and blackout status.
 * - `src/ricebot/robot/README.md`:
 *   - 100% homogeneous "Medic-Ant" capabilities (`canSampleSoil`, `canPlantSeed`, `canRigidDockRescue`) and
 *     4-part rigid docking actuator telemetry (`dockPitchRad`, `camLatchRad`, `anchorSpadeRad`, `isRigidLatched`).
 * - `docs/IMMOBILIZATION_SCENARIOS.md` & `docs/DECISIONS.md` (`D-0011`, `D-0012`):
 *   - `FaultScenarioId`: S1–S8 physical immobilization hypotheses plus Negative Controls (`NEG_SENSOR_DROPOUT`, `NEG_INTENTIONAL_HOLD`).
 *   - `DynamicObstacle`: Mid-run injected obstacles that trigger automatic trajectory replanning around blocked cells.
 *
 * Use Cases:
 * 1. Type-safe rendering of robot dots, continuous 2D RBF belief heatmaps, and rigid docking vectors on the map.
 * 2. Expressing clear, natural-language robot plans (e.g., "Robot 1 is going to retrieve Robot 3 via rigid dock & pitch-lift").
 * 3. Managing emergency human teleoperation, rigid docking actuators, and Pareto rescue weight sliders.
 */

export type RobotStatusCode =
  | 'NOMINAL'
  | 'RETRIEVING'
  | 'WARNING_SLIP'
  | 'IMMOBILIZED'
  | 'HUMAN_CONTROL';

export type TaskMode =
  | 'PLANT'
  | 'SAMPLE'
  | 'RETRIEVE_PEER'
  | 'NAVIGATE'
  | 'BEACON_RELAY'
  | 'HOLD';

export type BeliefPerspective = 'planter' | 'scout' | 'divergence' | 'truth';

export type ContinuousHeatmapLayer =
  | 'moisture'
  | 'firmness'
  | 'plant_density'
  | 'stand_gap'
  | 'uncertainty'
  | 'traversability_risk'
  | 'elevation_m';

export type CommsScenarioMode =
  | 'NORMAL_P2P_GOSSIP'
  | 'RADIO_BLACKOUT_WINDOW'
  | 'S1_MUD_RESCUE_EVENT';

export type FaultScenarioId =
  | 'NONE'
  | 'S1_PLOW_PAN_BREACH'
  | 'S2_ASYMMETRIC_YAW_TRAP'
  | 'S3_BUND_HIGH_CENTER'
  | 'S4_TRENCH_DROP_IN'
  | 'S5_STRAW_ENTANGLEMENT'
  | 'S6_CLAY_SUCTION_LOCK'
  | 'S7_RIGID_DOCK_OVERLOAD'
  | 'S8_ROW_PINCH_OCCLUSION'
  | 'NEG_MISSING_LIDAR_FAULT'
  | 'NEG_INTENTIONAL_SEED_HOLD';

export interface MetricCoordinate {
  x: number;
  y: number;
}

export interface RigidDockMechanismState {
  /** Front active pitch-lift docking boom angle in radians (-0.20 to +0.45 rad) */
  dockPitchRad: number;
  /** Internal rotary cam-locking jaw angle in radians (0.00 to 0.65 rad) */
  camLatchRad: number;
  /** Rear deployable Danforth hardpan anchor spade angle in radians (0.00 to 0.85 rad) */
  anchorSpadeRad: number;
  /** True when front cone is seated <= 0.065m and cam-latch >= 0.30 rad */
  isRigidLatched: boolean;
}

export interface OnboardSensorTelemetry {
  /** Odometry / ground velocity estimate (m/s) */
  trueVelocityMs: number;
  /** 4-wheel differential skid-steer linear wheel speed (m/s) */
  wheelSpeedMs: number;
  /** Kinematic slip ratio s = 1 - (v_true / v_wheel), range [0, 1] */
  slipRatio: number;
  /** IMU pitch angle in degrees */
  pitchDeg: number;
  /** IMU roll angle in degrees */
  rollDeg: number;
  /** Motor phase current effort in Amps */
  motorCurrentAmps: number;
  /** Forward 3-ray LiDAR center rangefinder distance in meters (or null if sensor dropout) */
  frontLidarRangeM: number | null;
  /** Row cross-track error in cm (fits within 300mm inter-row lane) */
  furrowErrorCm: number;
  /** Bekker-Wong instantaneous wheel sinkage z in mm */
  wheelSinkageMm: number;
  /** Remaining belly ground clearance in mm (<= 0 means bellied out) */
  bellyClearanceMm: number;
  /** Rigid docking & cam-latch actuator state */
  rigidDock: RigidDockMechanismState;
}

export interface RobotPlanStep {
  id: string;
  order: number;
  title: string;
  targetCoord: MetricCoordinate;
  status: 'COMPLETED' | 'IN_PROGRESS' | 'QUEUED';
  expectedPostcondition: string;
}

export interface RobotCapabilities {
  canNavigate: boolean;
  canSampleSoil: boolean;
  canPlantSeed: boolean;
  hasRigidDockMechanism: boolean;
  maxHopperCapacity: number;
}

export interface RobotState {
  id: string;
  callsign: string;
  roleTitle: string;
  status: RobotStatusCode;
  taskMode: TaskMode;
  /** Current metric position on the 24m x 16m paddy field */
  position: MetricCoordinate;
  /** Heading in degrees (0 = East, 90 = South, etc.) */
  headingDeg: number;
  /** Current final target waypoint in meters */
  targetWaypoint: MetricCoordinate;
  /** Multi-waypoint route (bends around dynamic obstacles injected via D-0012) */
  plannedRoute: MetricCoordinate[];
  /** Optional ID of peer robot being retrieved via rigid docking */
  targetPeerId: string | null;
  /** Optional ID of target crop gap site */
  targetSiteId: string | null;
  /** Human-readable natural language plan summary (e.g., "Robot 1 is going to retrieve Robot 3") */
  planSummary: string;
  /** Detailed step-by-step plan sequence */
  planSteps: RobotPlanStep[];
  /** Remaining pre-germinated rice seed pellets in conical hopper */
  seedHopperCount: number;
  /** Battery state of charge (%) */
  batteryPct: number;
  /** Active paddy fault or negative-control scenario */
  faultScenario: FaultScenarioId;
  /** Structured diagnostic message */
  faultReason: string | null;
  /** Whether human operator has engaged direct manual control on this robot */
  humanControlActive: boolean;
  /** 100% homogeneous Medic-Ant capabilities */
  capabilities: RobotCapabilities;
  /** Live onboard sensor & rigid-docking telemetry */
  telemetry: OnboardSensorTelemetry;
  /** Recent breadcrumb positions for path visualization */
  breadcrumbs: MetricCoordinate[];
}

export interface CropGapSite {
  id: string;
  label: string;
  section: 'SECTION_A' | 'SECTION_B';
  position: MetricCoordinate;
  rowNumber: number;
  soilMoistureProxy: number; // 0.0 to 1.0
  soilFirmnessKpa: number;
  standGapScore: number; // 0.0 to 1.0
  status: 'MISSING_CROP' | 'ASSIGNED' | 'PLANTED_VERIFIED' | 'HAZARD_SKIPPED';
  assignedRobotId: string | null;
  sampledByRobotId: string | null;
}

export interface FieldHazardZone {
  id: string;
  label: string;
  type: 'DEEP_MUD_SINKHOLE' | 'EARTHEN_BUND' | 'IRRIGATION_TRENCH' | 'STRAW_MAT';
  position: MetricCoordinate;
  radiusMeters: number;
  scenarioCode: FaultScenarioId;
  description: string;
}

export interface DynamicObstacle {
  id: string;
  label: string;
  position: MetricCoordinate;
  radiusMeters: number;
  injectedAtTick: number;
}

export interface ParetoWeights {
  timeWeight: number;
  energyWeight: number;
  cropDamageWeight: number;
  soilDisturbanceWeight: number;
  recoveryRiskWeight: number;
}

export type RescuePolicyCode =
  | 'SELF_RECOVER_BOUNDED'
  | 'DISPATCH_RIGID_DOCK_DIRECT'
  | 'DISPATCH_RIGID_DOCK_DETOUR'
  | 'REALLOCATE_AND_BEACON'
  | 'SAFE_ABSTAIN_DEFER_HUMAN'
  | 'NEGATIVE_CONTROL_NO_RESCUE';

export interface RescueCandidateEvaluation {
  policyCode: RescuePolicyCode;
  title: string;
  recommended: boolean;
  netUtilityScore: number;
  paretoCostJ: number;
  coverageDeltaPct: number;
  cropDamageRiskPct: number;
  energyCostWh: number;
  secondaryImmobilizationProbPct: number;
  assignedHelperId: string | null;
  rationale: string;
  actionPlanText: string;
}

export interface FaultScenarioDefinition {
  id: FaultScenarioId;
  code: string;
  title: string;
  category: 'IMMOBILIZATION_HYPOTHESIS' | 'NEGATIVE_CONTROL';
  physicalMechanism: string;
  sensorSignature: string;
  recommendedPolicy: RescuePolicyCode;
  defaultTelemetryOverride: Partial<OnboardSensorTelemetry>;
}

export interface BekkerTerramechanicsInput {
  massKg: number;
  wheelRadiusMm: number;
  wheelWidthMm: number;
  mudMoisture: number;
}

export interface BekkerTerramechanicsOutput {
  sinkageMm: number;
  bellyClearanceMm: number;
  mudDragForceN: number;
  belliedOut: boolean;
  verdictCode: 'TRAVERSABLE' | 'MARGINAL_SLIP' | 'BELLY_OUT_STUCK';
  verdictLabel: string;
}

export interface MycelialGossipPacketLog {
  id: string;
  tick: number;
  timestampSec: number;
  senderId: string;
  receiverId: string;
  distanceM: number;
  rssiDbm: number;
  sharedSamplesCount: number;
  sharedHazardCells: number;
  status: 'DELIVERED' | 'BLACKOUT_DROPPED';
  summary: string;
}
