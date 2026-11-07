/**
 * @file src/config/appConfig.ts
 * @description Centralized Configuration Module for the Antenna Swarm Control Console.
 *
 * This file aggregates all configurable parameters across the application, including:
 * - Flooded paddy terrace physical geometry (metric dimensions, grid resolution, crop row spacing).
 * - Low-level STM32H7 kinematic & sensor thresholds (wheel slip ratio gates, INA240 winch cable tension limits,
 *   IMU pitch/roll rollover safety limits, UWB proximity envelopes).
 * - Color-coded robot operational status visual mappings and accessible text symbols.
 * - Multi-objective rescue evaluation weights for the Antenna AI Coordinator.
 * - Optional AI advisor model configuration (centralized model identifiers and generation limits).
 *
 * Use Cases:
 * 1. Adjusting paddy field dimensions or simulation tick frequency from a single source of truth.
 * 2. Referencing semantic color codes and accessible glyphs across the Map Canvas, Fleet Roster, and Telemetry Inspector.
 * 3. Configuring safety cutoff bounds for the Emergency Human Teleoperation mode.
 */

export const APP_CONFIG = {
  /** Application branding and version metadata */
  appTitle: 'Antenna',
  fleetName: 'Kinematic Quintet (rice.v1)',

  /** Centralized AI Model Names (for optional swarm incident advisor) */
  models: {
    defaultAdvisorModel: 'gemini-3-flash-preview',
    fallbackAdvisorModel: 'gemini-3.1-pro-preview',
  },

  /** Flooded Paddy Field Metric Geometry (in meters) */
  fieldGeometry: {
    widthMeters: 24.0,
    heightMeters: 16.0,
    gridCols: 12,
    gridRows: 8,
    cropRowCount: 7,
    furrowSpacingMeters: 2.0,
    depotPosition: { x: 2.0, y: 2.0 },
    relayTowerPosition: { x: 22.0, y: 2.0 },
  },

  /** Simulation & Telemetry Loop Timing */
  simulation: {
    defaultTickMs: 600,
    stm32LoopHz: 200,
    manualStepDistanceMeters: 0.65,
    manualRotationDegrees: 15,
    seedReleaseConsumption: 1,
  },

  /** STM32H7 200Hz Physical & Safety Thresholds */
  safetyThresholds: {
    slipWarningRatio: 0.42,
    slipCriticalRatio: 0.60,
    maxSafePitchDeg: 18.0,
    maxSafeRollDeg: 18.0,
    minTetherTensionN: 20,
    maxNominalTetherTensionN: 350,
    emergencyReleaseTensionN: 400,
    minUwbPeerDistanceM: 1.0,
    optimalConvoySpacingM: 2.5,
    wheelRadiusMeters: 0.14,
  },

  /** Multi-Objective Cost Function Weights J(action) */
  rescueWeights: {
    coverageWeight: 0.35,
    cropProtectionWeight: 0.30,
    energyConservationWeight: 0.15,
    secondaryRiskWeight: 0.20,
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
      label: 'Retrieving / Assisting Peer',
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
      label: 'Elevated Slip / Traction Alert',
      shortLabel: 'Slip Alert',
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
