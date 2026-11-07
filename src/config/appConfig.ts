/**
 * @file src/config/appConfig.ts
 * @description Centralized Configuration Module for the Antenna Swarm Control Console.
 *
 * This file aggregates all configurable parameters across the application, aligned with:
 * - `src/ricebot/viewer/README.md` & `dashboard.py` (`antenna.dashboard.v1` continuous 2D RBF belief heatmaps,
 *   robot belief vs. ground-truth perspective switcher, and Bekker–Wong terramechanics sizing calculator).
 * - `src/ricebot/robot/README.md` (100% homogeneous "Medic-Ant" rover specs: 270×190×64 mm sealed hull,
 *   276 mm outer wheel width for <300 mm paddy rows, Ø116×40 mm wheels, 3-ray LiDAR turret, BeiDou RTK GNSS,
 *   and 4-part Rigid Docking Cone + Rotary Cam-Latch + Pitch-Lift Boom + Rear Hardpan Anchor Spades).
 * - `docs/DECISIONS.md` (`D-0011` 3-tier cognitive fallback & Pareto multi-objective weights; `D-0012` dynamic
 *   mid-sequence obstacle injection & offline edge compute budget telemetry).
 * - `docs/IMMOBILIZATION_SCENARIOS.md` (S1–S8 physical immobilization hypotheses and negative controls).
 *
 * Use Cases:
 * 1. Configuring field dimensions, staggered Section A / Section B batch geometry, and heatmap grid resolution.
 * 2. Setting default Bekker–Wong vehicle sizing parameters and rigid-docking actuator limits.
 * 3. Defining semantic status colors and accessible glyphs across all UI panels.
 */

export const APP_CONFIG = {
  /** Application branding and schema contract identifiers */
  appTitle: 'Antenna',
  fleetName: 'Kinematic Quintet · Homogeneous Medic-Ant Swarm (rice.v1)',
  dashboardSchemaVersion: 'antenna.dashboard.v1',

  /** D-0012 Edge Compute Budget Telemetry (100% Offline Standalone) */
  edgeComputeBudget: {
    decisionLatencyMs: 0.4,
    powerWatts: 2.4,
    computeNodes: 'STM32H7 (Tier-0) + RPi5/Orin (Tier-1)',
    cloudGpuCostUsd: 0,
    summaryBadge:
      'Edge Compute Budget: ~0.4ms / decision · Power: ~2.4W (STM32/Pi5) · Cloud GPU: $0 (100% Offline Standalone)',
  },

  /** Centralized AI Model Names (Tier-2 Optional Deliberative Supervisor) */
  models: {
    defaultAdvisorModel: 'gemini-3-flash-preview',
    fallbackAdvisorModel: 'gemini-3.1-pro-preview',
  },

  /** Flooded Paddy Field Metric Geometry (24.0m x 16.0m with Section A & Section B Staggered Batches) */
  fieldGeometry: {
    widthMeters: 24.0,
    heightMeters: 16.0,
    sectionSplitXMeters: 12.0,
    heatmapGridCols: 24,
    heatmapGridRows: 16,
    cropRowCount: 7,
    furrowSpacingMeters: 2.0,
    interRowLaneWidthMm: 300,
    depotPosition: { x: 2.0, y: 2.0 },
    relayTowerPosition: { x: 22.0, y: 2.0 },
  },

  /** Homogeneous "Medic-Ant" Rover Physical Spec (`src/ricebot/robot/README.md`) */
  roverHomogeneousSpec: {
    hullDimensionsMm: '270 × 190 × 64 mm',
    outerWheelWidthMm: 276,
    wheelDiameterMm: 116,
    wheelRadiusMm: 58,
    wheelWidthMm: 40,
    defaultMassKg: 4.2,
    maxLinearVelocityMs: 0.45,
    maxAngularVelocityRadS: 1.4,
    lidarRayAnglesDeg: [-18, 0, 18],
    lidarCutoffMeters: 4.0,
    probeRadiusMeters: 0.28,
    dockPitchMinRad: -0.2,
    dockPitchMaxRad: 0.45,
    camLatchLockedThresholdRad: 0.3,
    camLatchMaxRad: 0.65,
    anchorSpadeMaxRad: 0.85,
  },

  /** Simulation & Telemetry Loop Timing */
  simulation: {
    defaultTickMs: 600,
    manualStepDistanceMeters: 0.65,
    seedReleaseConsumption: 1,
  },

  /** Physical & Kinematic Diagnostic Thresholds */
  safetyThresholds: {
    slipWarningRatio: 0.42,
    slipCriticalRatio: 0.65,
    maxSafePitchDeg: 18.0,
    maxSafeRollDeg: 18.0,
    lidarStopDistanceM: 0.35,
    dockContactDistanceM: 0.065,
  },

  /** Default Pareto Multi-Objective Rescue Weights (D-0011: J = w_t*T + w_e*E + w_c*C + w_s*S + w_r*R) */
  defaultParetoWeights: {
    timeWeight: 0.25,
    energyWeight: 0.20,
    cropDamageWeight: 0.30,
    soilDisturbanceWeight: 0.15,
    recoveryRiskWeight: 0.10,
  },

  /** Semantic Color-Coding & Accessible Symbols for Robot Statuses */
  statusTheme: {
    NOMINAL: {
      code: 'NOMINAL',
      label: 'Nominal Operation',
      shortLabel: 'Nominal',
      glyph: '●',
      hex: '#10b981', // Emerald 500
      ringHex: 'rgba(16, 185, 129, 0.28)',
      textClass: 'text-emerald-400',
      dotClass: 'bg-emerald-400',
      borderClass: 'border-emerald-500/40',
    },
    RETRIEVING: {
      code: 'RETRIEVING',
      label: 'Rigid-Dock Tandem Rescue',
      shortLabel: 'Retrieving',
      glyph: '◆',
      hex: '#06b6d4', // Cyan 500
      ringHex: 'rgba(6, 182, 212, 0.30)',
      textClass: 'text-cyan-400',
      dotClass: 'bg-cyan-400',
      borderClass: 'border-cyan-500/40',
    },
    WARNING_SLIP: {
      code: 'WARNING_SLIP',
      label: 'Elevated Slip / Sensor Alert',
      shortLabel: 'Alert / Slip',
      glyph: '▲',
      hex: '#f59e0b', // Amber 500
      ringHex: 'rgba(245, 158, 11, 0.30)',
      textClass: 'text-amber-400',
      dotClass: 'bg-amber-400',
      borderClass: 'border-amber-500/40',
    },
    IMMOBILIZED: {
      code: 'IMMOBILIZED',
      label: 'Immobilized / Fault Trap',
      shortLabel: 'Immobilized',
      glyph: '✖',
      hex: '#ef4444', // Red 500
      ringHex: 'rgba(239, 68, 68, 0.35)',
      textClass: 'text-rose-400',
      dotClass: 'bg-rose-500',
      borderClass: 'border-rose-500/50',
    },
    HUMAN_CONTROL: {
      code: 'HUMAN_CONTROL',
      label: 'Human Teleop Override',
      shortLabel: 'Human Control',
      glyph: '■',
      hex: '#a855f7', // Purple 500
      ringHex: 'rgba(168, 85, 247, 0.35)',
      textClass: 'text-purple-400',
      dotClass: 'bg-purple-400',
      borderClass: 'border-purple-500/50',
    },
  },
} as const;
