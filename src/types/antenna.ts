/**
 * @file src/types/antenna.ts
 * @description Domain Type Definitions & Versioned Contracts (`rice.v1`) for the Antenna Multi-Robot System.
 *
 * This file defines the data structures used across the Antenna Swarm Control Console, directly aligned
 * with the `AgentVerse_Kinematic_Quintet` architecture:
 * - `RobotStatusCode`: Color-coded operational states (`NOMINAL`, `RETRIEVING`, `WARNING_SLIP`, `IMMOBILIZED`, `HUMAN_CONTROL`).
 * - `TaskMode`: Contract-level mission intents (`PLANT`, `SAMPLE`, `RETRIEVE_PEER`, `NAVIGATE`, `BEACON_RELAY`, `HOLD`).
 * - `FaultScenarioId`: The 8 physical paddy immobilization scenarios (`S1` through `S8`) diagnosed by the 200Hz STM32H7 loop.
 * - `RobotState`: Complete robot observation, physical capability, active plan, waypoint trajectory, and sensor fusion telemetry.
 * - `CropGapSite`: Target coordinates where rice seedlings/seeds are missing and require targeted replanting.
 * - `FieldHazardZone`: Submerged mud traps, earthen bunds, or drainage ditches mapped by the swarm.
 * - `RescueCandidateEvaluation`: Multi-objective scoring breakdown J(action) for cooperative recovery decisions.
 *
 * Use Cases:
 * 1. Type-safe rendering of robot dots, color-coded status indicators, and tether vectors on the Paddy Field Map.
 * 2. Expressing clear, human-readable robot plans (e.g., "Robot 1 is going to retrieve Robot 3").
 * 3. Managing emergency human teleoperation state and safety gate overrides.
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

export type FaultScenarioId =
  | 'NONE'
  | 'S1_PLOW_PAN_BREACH'
  | 'S2_ASYMMETRIC_YAW_TRAP'
  | 'S3_BUND_HIGH_CENTER'
  | 'S4_TRENCH_DROP_IN'
  | 'S5_STRAW_ENTANGLEMENT'
  | 'S6_CLAY_SUCTION_LOCK'
  | 'S7_TETHER_SNAG_OVERLOAD'
  | 'S8_FURROW_PINCH';

export interface MetricCoordinate {
  x: number;
  y: number;
}

export interface Stm32Telemetry {
  /** 60GHz Doppler mmWave Radar true ground speed (m/s) */
  trueVelocityMs: number;
  /** Wheel encoder linear equivalent speed omega * r (m/s) */
  wheelSpeedMs: number;
  /** Computed kinematic slip ratio s = 1 - (v_true / (omega * r)), range [0, 1] */
  slipRatio: number;
  /** IMU pitch angle in degrees */
  pitchDeg: number;
  /** IMU roll angle in degrees */
  rollDeg: number;
  /** Average brushless hub motor phase current in Amps */
  motorCurrentAmps: number;
  /** INA240 strain-gauge tow hitch pulling tension in Newtons */
  tetherTensionN: number;
  /** Hall-effect whisker + VL53L8CX ToF lateral cross-track error in cm */
  furrowErrorCm: number;
  /** Dual-Qorvo DWM3000 UWB link quality percentage */
  uwbLinkQualityPct: number;
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
  hasWinchTether: boolean;
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
  /** Current target waypoint in meters */
  targetWaypoint: MetricCoordinate;
  /** Optional ID of peer robot being retrieved or tethered */
  targetPeerId: string | null;
  /** Optional ID of target crop gap site */
  targetSiteId: string | null;
  /** Human-readable natural language plan summary (e.g., "Robot 1 is going to retrieve Robot 3") */
  planSummary: string;
  /** Detailed step-by-step plan sequence */
  planSteps: RobotPlanStep[];
  /** Remaining rice seed/seedling proxies in actuated hopper */
  seedHopperCount: number;
  /** Battery state of charge (%) */
  batteryPct: number;
  /** Active paddy fault scenario if immobilized or degraded */
  faultScenario: FaultScenarioId;
  /** Structured fault diagnostic message from 200Hz STM32H7 loop */
  faultReason: string | null;
  /** Whether human operator has engaged direct manual control on this robot */
  humanControlActive: boolean;
  /** Physical capabilities of this rover unit */
  capabilities: RobotCapabilities;
  /** Live 200Hz sensor fusion telemetry */
  telemetry: Stm32Telemetry;
  /** Recent breadcrumb positions for path visualization */
  breadcrumbs: MetricCoordinate[];
}

export interface CropGapSite {
  id: string;
  label: string;
  position: MetricCoordinate;
  rowNumber: number;
  soilMoistureProxy: number; // 0.0 to 1.0
  soilFirmnessKpa: number;
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

export type RescuePolicyCode =
  | 'SELF_RECOVER_BOUNDED'
  | 'DISPATCH_RESCUE_DIRECT'
  | 'DISPATCH_RESCUE_DETOUR'
  | 'REALLOCATE_AND_BEACON'
  | 'SAFE_ABSTAIN_DEFER_HUMAN';

export interface RescueCandidateEvaluation {
  policyCode: RescuePolicyCode;
  title: string;
  recommended: boolean;
  netUtilityScore: number;
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
  chineseTerm: string;
  physicalMechanism: string;
  sensorSignature: string;
  recommendedPolicy: RescuePolicyCode;
  defaultTelemetryOverride: Partial<Stm32Telemetry>;
}
