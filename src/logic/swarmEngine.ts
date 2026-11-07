/**
 * @file src/logic/swarmEngine.ts
 * @description Pure Kinematic, Continuous RBF Belief Field, Bekker–Wong Terramechanics, Dynamic Obstacle Replanning,
 * Rigid Docking Rescue & Pareto Multi-Objective Engine for the Antenna Swarm Control Console.
 *
 * This module implements the core mathematical and algorithmic specifications from the repository:
 * 1. Analytical Bekker–Wong Wheel–Mud Sinkage (`src/ricebot/viewer/dashboard.py`):
 *    Computes sinkage $z$, belly clearance, mud drag $R_c$, and belly-out immobilization verdict from
 *    vehicle mass $m$, wheel radius $r$, wheel width $b$, and local mud moisture.
 * 2. Continuous 2D RBF Belief Field Evaluator (`src/ricebot/viewer/README.md`):
 *    Evaluates the 7 continuous layers (`moisture`, `firmness`, `plant_density`, `stand_gap`,
 *    `uncertainty`, `traversability_risk`, `elevation_m`) across the 4 perspectives
 *    (`planter`, `scout`, `divergence`, `truth`) without leaking hidden ground truth to planners.
 * 3. Dynamic Mid-Sequence Obstacle Injection & Route Bending (`docs/DECISIONS.md` `D-0012`):
 *    Inserts a sudden field obstacle and automatically recalculates a collision-free multi-waypoint
 *    detour trajectory around the obstacle.
 * 4. Homogeneous "Medic-Ant" Role Switching & Rigid Docking Actuation (`src/ricebot/robot/README.md`):
 *    Automatically switches a planter with `0` remaining seeds to `SAMPLE` (Scout) mode, and models
 *    the 4-part Rigid Docking Cone + Rotary Cam-Latch + Pitch-Lift Boom + Hardpan Anchor Spades.
 * 5. Multi-Objective Pareto Rescue Evaluator & Negative Control Gate (`docs/IMMOBILIZATION_SCENARIOS.md`):
 *    Evaluates $J = w_t T + w_e E + w_c C + w_s S + w_r R$ and blocks false-positive rescue dispatches on
 *    Negative Controls (`NEG_MISSING_LIDAR_FAULT`, `NEG_INTENTIONAL_SEED_HOLD`).
 */

import { APP_CONFIG } from '../config/appConfig';
import { FAULT_SCENARIO_CATALOG } from '../data/initialSwarmState';
import {
  BekkerTerramechanicsInput,
  BekkerTerramechanicsOutput,
  BeliefPerspective,
  ContinuousHeatmapLayer,
  CropGapSite,
  DynamicObstacle,
  FaultScenarioId,
  MetricCoordinate,
  ParetoWeights,
  RescueCandidateEvaluation,
  RescuePolicyCode,
  RobotState,
  RobotStatusCode,
  TaskMode,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

/**
 * Clamps a numeric value within an inclusive [min, max] interval.
 * @param value - Input number to clamp.
 * @param min - Lower bound.
 * @param max - Upper bound.
 * @returns Clamped number rounded to 2 decimal places.
 */
export function clampMetric(value: number, min: number, max: number): number {
  logFunctionCall('clampMetric', { value, min, max });
  const bounded = Math.max(min, Math.min(max, value));
  return Math.round(bounded * 100) / 100;
}

/**
 * Computes the Euclidean metric distance between two coordinates on the paddy field.
 * @param a - First metric coordinate (x, y).
 * @param b - Second metric coordinate (x, y).
 * @returns Euclidean distance in meters rounded to 2 decimal places.
 */
export function computeMetricDistance(a: MetricCoordinate, b: MetricCoordinate): number {
  logFunctionCall('computeMetricDistance', { a, b });
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  return Math.round(Math.hypot(dx, dy) * 100) / 100;
}

/**
 * Computes the kinematic wheel slip ratio s = 1 - (v_true / v_wheel).
 * @param trueVelocityMs - Ground velocity in m/s.
 * @param wheelSpeedMs - Wheel linear speed in m/s.
 * @returns Slip ratio in [0, 1] rounded to 2 decimal places.
 */
export function calculateSlipRatio(trueVelocityMs: number, wheelSpeedMs: number): number {
  logFunctionCall('calculateSlipRatio', { trueVelocityMs, wheelSpeedMs });
  if (wheelSpeedMs <= 0.005) {
    return 0;
  }
  const rawSlip = 1 - trueVelocityMs / wheelSpeedMs;
  return clampMetric(rawSlip, 0, 1);
}

/**
 * Evaluates the analytical Bekker–Wong wheel–mud sinkage, belly clearance, and compaction drag
 * using the exact equations from `src/ricebot/viewer/dashboard.py`.
 * @param input - BekkerTerramechanicsInput (massKg, wheelRadiusMm, wheelWidthMm, mudMoisture).
 * @returns BekkerTerramechanicsOutput with sinkage (mm), belly clearance (mm), drag (N), and verdict.
 */
export function evaluateBekkerTerramechanics(
  input: BekkerTerramechanicsInput
): BekkerTerramechanicsOutput {
  logFunctionCall('evaluateBekkerTerramechanics', { ...input });

  const { massKg, wheelRadiusMm, wheelWidthMm, mudMoisture } = input;
  const r_m = wheelRadiusMm / 1000.0;
  const b_m = wheelWidthMm / 1000.0;
  const m = Math.max(0.05, Math.min(0.99, mudMoisture));
  const f = Math.max(0.1, 1.0 - 0.85 * m);
  const mudDepth = 0.04 + 0.3 * Math.pow(m, 1.6) * Math.pow(1.0 - f, 1.2);
  const clearance0 = Math.max(0.02, 0.044 + (r_m - 0.058));
  const kc = 1.5 + 24.0 * Math.pow(f, 1.4) * (1.0 - 0.55 * m);
  const kphi = 45.0 + 920.0 * Math.pow(f, 1.5) * (1.0 - 0.6 * m);
  const n = 0.52 + 0.58 * f;
  const k_eq = Math.max(28000.0, (kc / Math.max(0.015, b_m) + kphi) * 950.0);
  const wWheel = (massKg * 9.81) / 4.0;
  const chord = Math.sqrt(2.0 * r_m * 0.035);
  const z0 = Math.pow(wWheel / (b_m * chord * k_eq), 1.0 / n);
  const z = Math.min(mudDepth, z0 + 0.008 * (1.0 - f));
  const bellyClear = clearance0 - z;
  const belliedOut = bellyClear <= 0.0;
  const rc = (4.0 * b_m * k_eq * Math.pow(z, n + 1.0)) / (n + 1.0);

  const sinkageMm = Math.round(z * 1000 * 10) / 10;
  const bellyClearanceMm = Math.round(bellyClear * 1000 * 10) / 10;
  const mudDragForceN = Math.round(rc * 10) / 10;

  let verdictCode: BekkerTerramechanicsOutput['verdictCode'] = 'TRAVERSABLE';
  let verdictLabel = 'TRAVERSABLE (SAFE CLEARANCE)';
  if (belliedOut) {
    verdictCode = 'BELLY_OUT_STUCK';
    verdictLabel = 'S1 BELLY-OUT / STUCK (RESCUE REQ)';
  } else if (bellyClear < 0.012) {
    verdictCode = 'MARGINAL_SLIP';
    verdictLabel = 'HIGH SLIP / MARGINAL CLEARANCE';
  }

  return {
    sinkageMm,
    bellyClearanceMm,
    mudDragForceN,
    belliedOut,
    verdictCode,
    verdictLabel,
  };
}

/**
 * Evaluates the continuous 2D RBF field value at metric coordinate (x, y) for a selected layer
 * and belief perspective (`planter`, `scout`, `divergence`, `truth`), mirroring `dashboard.py`.
 * Note: This function is purposefully high-frequency for grid rendering, so it avoids per-cell console spam.
 * @param x - Metric X coordinate in [0, 24].
 * @param y - Metric Y coordinate in [0, 16].
 * @param layer - ContinuousHeatmapLayer to evaluate.
 * @param perspective - BeliefPerspective ('planter' | 'scout' | 'divergence' | 'truth').
 * @returns Normalized continuous scalar value in [0, 1].
 */
export function evaluateContinuousBeliefField(
  x: number,
  y: number,
  layer: ContinuousHeatmapLayer,
  perspective: BeliefPerspective
): number {
  // Distance to S1 mud sinkhole at (14.6, 8.4) and S3 bund at (18.2, 10.2)
  const dSinkhole = Math.hypot(x - 14.6, y - 8.4);
  const dBund = Math.hypot(x - 18.2, y - 10.2);
  const dTrench = Math.hypot(x - 22.4, y - 9.0);

  // Underlying ground-truth scalar for the requested layer
  let truthVal = 0.5;
  switch (layer) {
    case 'moisture':
      truthVal =
        0.58 +
        0.34 * Math.exp(-(dSinkhole * dSinkhole) / 9.5) +
        0.08 * Math.sin(x * 0.35) * Math.cos(y * 0.4);
      break;
    case 'firmness':
      truthVal =
        0.74 -
        0.58 * Math.exp(-(dSinkhole * dSinkhole) / 8.5) +
        0.14 * Math.exp(-(dBund * dBund) / 5.0);
      break;
    case 'plant_density':
      truthVal =
        0.76 -
        0.45 * Math.exp(-Math.pow(x - 11.2, 2) / 6.0 - Math.pow(y - 4.0, 2) / 4.0) -
        0.42 * Math.exp(-Math.pow(x - 16.5, 2) / 6.0 - Math.pow(y - 4.0, 2) / 4.0);
      break;
    case 'stand_gap':
      truthVal =
        0.18 +
        0.72 * Math.exp(-Math.pow(x - 11.2, 2) / 5.5 - Math.pow(y - 4.0, 2) / 3.5) +
        0.68 * Math.exp(-Math.pow(x - 16.5, 2) / 5.5 - Math.pow(y - 4.0, 2) / 3.5) +
        0.64 * Math.exp(-Math.pow(x - 9.6, 2) / 5.5 - Math.pow(y - 14.0, 2) / 3.5);
      break;
    case 'uncertainty':
      // Uncertainty is lowest near sampled Section A & Scout trajectories
      truthVal =
        0.65 -
        0.48 * Math.exp(-Math.pow(x - 9.0, 2) / 28.0 - Math.pow(y - 7.0, 2) / 18.0) -
        0.35 * Math.exp(-Math.pow(x - 16.0, 2) / 22.0 - Math.pow(y - 12.0, 2) / 14.0);
      break;
    case 'traversability_risk':
      truthVal =
        0.12 +
        0.82 * Math.exp(-(dSinkhole * dSinkhole) / 6.5) +
        0.65 * Math.exp(-(dBund * dBund) / 4.5) +
        0.75 * Math.exp(-(dTrench * dTrench) / 4.5);
      break;
    case 'elevation_m':
      truthVal =
        0.45 +
        0.38 * Math.exp(-(dBund * dBund) / 4.0) -
        0.32 * Math.exp(-(dSinkhole * dSinkhole) / 6.0) -
        0.35 * Math.exp(-(dTrench * dTrench) / 4.0);
      break;
  }

  const clampedTruth = Math.max(0.02, Math.min(0.98, truthVal));
  if (perspective === 'truth') {
    return Math.round(clampedTruth * 100) / 100;
  }

  // Planter has denser observations in Section A (x < 13); Scout has denser observations in Section B (x >= 12)
  const planterBias =
    x > 15 ? 0.14 * Math.sin(x * 0.5 + y * 0.3) : 0.02 * Math.cos(x * 0.4);
  const scoutBias =
    x < 10 ? -0.13 * Math.cos(x * 0.45 - y * 0.25) : 0.02 * Math.sin(y * 0.4);

  const planterEst = Math.max(0.02, Math.min(0.98, clampedTruth + planterBias));
  const scoutEst = Math.max(0.02, Math.min(0.98, clampedTruth + scoutBias));

  if (perspective === 'planter') {
    return Math.round(planterEst * 100) / 100;
  }
  if (perspective === 'scout') {
    return Math.round(scoutEst * 100) / 100;
  }
  // 'divergence' = |Planter - Scout| Gap amplified for visual inspection
  const gap = Math.min(1.0, Math.abs(planterEst - scoutEst) * 3.2);
  return Math.round(gap * 100) / 100;
}

/**
 * Builds a clear, natural-language plan statement for a robot based on its task mode and targets.
 * @param robotNumberLabel - Short robot identifier such as "Robot 1" or callsign.
 * @param taskMode - Active mission mode (`PLANT`, `SAMPLE`, `RETRIEVE_PEER`, `NAVIGATE`, `BEACON_RELAY`, `HOLD`).
 * @param targetCoord - Target waypoint coordinate in meters.
 * @param targetPeerId - Optional peer robot ID (e.g., "ANT-03") when retrieving.
 * @param targetSiteId - Optional crop gap site ID (e.g., "C-04") when planting or sampling.
 * @param humanControlActive - Whether manual human teleoperation override is active.
 * @returns Human-readable plan description string.
 */
export function formatRobotPlanSummary(
  robotNumberLabel: string,
  taskMode: TaskMode,
  targetCoord: MetricCoordinate,
  targetPeerId: string | null,
  targetSiteId: string | null,
  humanControlActive: boolean
): string {
  logFunctionCall('formatRobotPlanSummary', {
    robotNumberLabel,
    taskMode,
    targetCoord,
    targetPeerId,
    targetSiteId,
    humanControlActive,
  });

  const coordText = `(${targetCoord.x.toFixed(1)}m, ${targetCoord.y.toFixed(1)}m)`;
  const shortName = robotNumberLabel.split('·')[0].trim();

  if (humanControlActive) {
    return `${shortName} is under direct human teleoperation override at ${coordText}`;
  }

  switch (taskMode) {
    case 'RETRIEVE_PEER': {
      const peerLabel = targetPeerId
        ? targetPeerId.replace('ANT-0', 'Robot ')
        : 'stuck peer';
      return `${shortName} is going to retrieve ${peerLabel} via collinear rigid docking, cam-latch & pitch-lift at ${coordText}`;
    }
    case 'PLANT': {
      const siteLabel = targetSiteId ? `missing crop gap ${targetSiteId}` : 'target furrow';
      return `${shortName} is going to plant ${siteLabel} at ${coordText}`;
    }
    case 'SAMPLE': {
      const siteLabel = targetSiteId ? `site ${targetSiteId}` : 'paddy sector';
      return `${shortName} is going to sample soil moisture & firmness at ${siteLabel} ${coordText}`;
    }
    case 'BEACON_RELAY':
      return `${shortName} is operating as a stationary RTK / Mycelial gossip relay node at ${coordText}`;
    case 'HOLD':
      return `${shortName} is holding position in deterministic Tier-0 safety lock at ${coordText}`;
    case 'NAVIGATE':
    default:
      return `${shortName} is going to navigate to waypoint ${coordText}`;
  }
}

/**
 * Derives the color-coded `RobotStatusCode` from a robot's current operational state and telemetry.
 * Strictly respects Negative Controls (`NEG_MISSING_LIDAR_FAULT`, `NEG_INTENTIONAL_SEED_HOLD`) so
 * they never show as `IMMOBILIZED`.
 * @param robot - RobotState to evaluate.
 * @returns Appropriate semantic RobotStatusCode.
 */
export function deriveRobotStatusCode(robot: RobotState): RobotStatusCode {
  logFunctionCall('deriveRobotStatusCode', { robotId: robot.id });

  if (robot.humanControlActive) {
    return 'HUMAN_CONTROL';
  }
  if (robot.faultScenario === 'NEG_INTENTIONAL_SEED_HOLD') {
    return 'NOMINAL';
  }
  if (robot.faultScenario === 'NEG_MISSING_LIDAR_FAULT') {
    return 'WARNING_SLIP';
  }
  if (robot.faultScenario !== 'NONE') {
    return 'IMMOBILIZED';
  }
  if (robot.taskMode === 'RETRIEVE_PEER') {
    return 'RETRIEVING';
  }
  if (robot.telemetry.slipRatio >= APP_CONFIG.safetyThresholds.slipWarningRatio) {
    return 'WARNING_SLIP';
  }
  return 'NOMINAL';
}

/**
 * Computes a collision-free multi-waypoint route from `start` to `end` that detours around
 * any active `DynamicObstacle` (`D-0012`).
 * @param start - Starting metric coordinate.
 * @param end - Target metric coordinate.
 * @param obstacles - List of injected DynamicObstacle objects.
 * @returns Ordered array of waypoints from start to end.
 */
export function computeDetourRoute(
  start: MetricCoordinate,
  end: MetricCoordinate,
  obstacles: DynamicObstacle[]
): MetricCoordinate[] {
  logFunctionCall('computeDetourRoute', {
    start,
    end,
    obstacleCount: obstacles.length,
  });

  if (obstacles.length === 0) {
    return [start, end];
  }

  // Check if any obstacle lies near the line segment from start to end
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const segLenSq = dx * dx + dy * dy;
  if (segLenSq < 0.05) {
    return [start, end];
  }

  for (const obs of obstacles) {
    const t = Math.max(
      0,
      Math.min(1, ((obs.position.x - start.x) * dx + (obs.position.y - start.y) * dy) / segLenSq)
    );
    const projX = start.x + t * dx;
    const projY = start.y + t * dy;
    const distToSeg = Math.hypot(obs.position.x - projX, obs.position.y - projY);

    if (distToSeg <= obs.radiusMeters + 0.75) {
      // Bend the trajectory around the obstacle (North or South depending on room)
      const offsetY = obs.position.y > 4.0 ? -(obs.radiusMeters + 1.3) : obs.radiusMeters + 1.3;
      const wp1: MetricCoordinate = {
        x: clampMetric(obs.position.x - 1.2, 1.0, APP_CONFIG.fieldGeometry.widthMeters - 1.0),
        y: clampMetric(obs.position.y + offsetY, 1.0, APP_CONFIG.fieldGeometry.heightMeters - 1.0),
      };
      const wp2: MetricCoordinate = {
        x: clampMetric(obs.position.x + 1.2, 1.0, APP_CONFIG.fieldGeometry.widthMeters - 1.0),
        y: clampMetric(obs.position.y + offsetY, 1.0, APP_CONFIG.fieldGeometry.heightMeters - 1.0),
      };
      return [start, wp1, wp2, end];
    }
  }

  return [start, end];
}

/**
 * Injects a sudden mid-sequence environmental obstacle (`D-0012`) along the selected or active robot's
 * trajectory, invalidates intersecting routes, and immediately replans a bent detour path around it.
 * @param robots - Current array of RobotState objects.
 * @param existingObstacles - Current array of DynamicObstacle objects.
 * @param targetRobotId - ID of the robot whose path will encounter the sudden obstacle.
 * @param tick - Current simulation tick counter.
 * @returns Updated robots, obstacles, and status message.
 */
export function injectDynamicObstacleAndReplan(
  robots: RobotState[],
  existingObstacles: DynamicObstacle[],
  targetRobotId: string,
  tick = 1
): {
  robots: RobotState[];
  obstacles: DynamicObstacle[];
  message: string;
} {
  logFunctionCall('injectDynamicObstacleAndReplan', {
    targetRobotId,
    existingObstacleCount: existingObstacles.length,
    tick,
  });

  // Choose an active robot with a non-zero path, preferring targetRobotId
  const candidate =
    robots.find(
      (r) =>
        r.id === targetRobotId &&
        computeMetricDistance(r.position, r.targetWaypoint) > 1.2
    ) ??
    robots.find((r) => computeMetricDistance(r.position, r.targetWaypoint) > 1.2) ??
    robots[0];

  const midX = clampMetric(
    (candidate.position.x + candidate.targetWaypoint.x) / 2,
    2.0,
    APP_CONFIG.fieldGeometry.widthMeters - 2.0
  );
  const midY = clampMetric(
    (candidate.position.y + candidate.targetWaypoint.y) / 2,
    2.0,
    APP_CONFIG.fieldGeometry.heightMeters - 2.0
  );

  const newObstacle: DynamicObstacle = {
    id: `OBS-0${existingObstacles.length + 1}`,
    label: `Collapsed Bund Obstacle #${existingObstacles.length + 1}`,
    position: { x: midX, y: midY },
    radiusMeters: 0.95,
    injectedAtTick: tick,
  };

  const nextObstacles = [...existingObstacles, newObstacle];

  const nextRobots = robots.map((robot) => {
    const newRoute = computeDetourRoute(robot.position, robot.targetWaypoint, nextObstacles);
    const wasReplanned = newRoute.length > 2;
    return {
      ...robot,
      plannedRoute: newRoute,
      planSummary: wasReplanned
        ? `${robot.callsign.split('·')[0].trim()} sensed ${newObstacle.label} at (${midX.toFixed(
            1
          )}m, ${midY.toFixed(1)}m) — A* replanned detour trajectory`
        : robot.planSummary,
    };
  });

  return {
    robots: nextRobots,
    obstacles: nextObstacles,
    message: `D-0012 Dynamic Obstacle Injected at (${midX.toFixed(1)}m, ${midY.toFixed(
      1
    )}m): ${candidate.id} route invalidated and replanned around obstacle.`,
  };
}

/**
 * Assigns a new task plan (`TaskIntentV1`) to a specific robot while enforcing homogeneous
 * seed-inventory checks (units with 0 seeds remaining cannot execute PLANT until refilled or switched to SAMPLE).
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot to reassign.
 * @param taskMode - Desired TaskMode.
 * @param targetCoord - Target metric waypoint.
 * @param options - Optional targetPeerId, targetSiteId, or obstacles.
 * @returns Object containing updated robots array and a validation message.
 */
export function assignRobotTaskPlan(
  robots: RobotState[],
  robotId: string,
  taskMode: TaskMode,
  targetCoord: MetricCoordinate,
  options: {
    targetPeerId?: string | null;
    targetSiteId?: string | null;
    obstacles?: DynamicObstacle[];
  } = {}
): { robots: RobotState[]; accepted: boolean; message: string } {
  logFunctionCall('assignRobotTaskPlan', {
    robotId,
    taskMode,
    targetCoord,
    options,
  });

  const targetRobot = robots.find((r) => r.id === robotId);
  if (!targetRobot) {
    return {
      robots,
      accepted: false,
      message: `Robot ${robotId} not found in fleet registry.`,
    };
  }

  // Enforce seed-inventory gate: a rover with 0 seeds in its hopper cannot execute PLANT
  if (taskMode === 'PLANT' && targetRobot.seedHopperCount <= 0) {
    return {
      robots,
      accepted: false,
      message: `Safety Gate Rejected: ${targetRobot.callsign} has 0 seeds remaining in its hopper (switched to Scout SAMPLE role until refilled).`,
    };
  }

  const clampedTarget: MetricCoordinate = {
    x: clampMetric(targetCoord.x, 0.8, APP_CONFIG.fieldGeometry.widthMeters - 0.8),
    y: clampMetric(targetCoord.y, 0.8, APP_CONFIG.fieldGeometry.heightMeters - 0.8),
  };

  const targetPeerId = options.targetPeerId ?? null;
  const targetSiteId = options.targetSiteId ?? null;
  const obstacles = options.obstacles ?? [];

  const updatedRobots = robots.map((robot) => {
    if (robot.id !== robotId) {
      return robot;
    }

    const clearedFault: FaultScenarioId = taskMode === 'HOLD' ? robot.faultScenario : 'NONE';
    const nextPlanSummary = formatRobotPlanSummary(
      robot.callsign,
      taskMode,
      clampedTarget,
      targetPeerId,
      targetSiteId,
      false
    );

    const nextTelemetry = {
      ...robot.telemetry,
      trueVelocityMs: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.38,
      wheelSpeedMs: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.42,
      slipRatio: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.08,
      rigidDock: {
        ...robot.telemetry.rigidDock,
        dockPitchRad: taskMode === 'RETRIEVE_PEER' ? 0.35 : 0.0,
        camLatchRad: taskMode === 'RETRIEVE_PEER' ? 0.55 : 0.0,
        anchorSpadeRad: taskMode === 'RETRIEVE_PEER' ? 0.40 : 0.0,
        isRigidLatched: taskMode === 'RETRIEVE_PEER',
      },
    };

    const nextRoute = computeDetourRoute(robot.position, clampedTarget, obstacles);

    const updatedRobot: RobotState = {
      ...robot,
      taskMode,
      humanControlActive: false,
      faultScenario: clearedFault,
      faultReason: clearedFault === 'NONE' ? null : robot.faultReason,
      targetWaypoint: clampedTarget,
      plannedRoute: nextRoute,
      targetPeerId,
      targetSiteId,
      planSummary: nextPlanSummary,
      telemetry: nextTelemetry,
      planSteps: [
        {
          id: `step-${Date.now()}-1`,
          order: 1,
          title: nextPlanSummary,
          targetCoord: clampedTarget,
          status: 'IN_PROGRESS',
          expectedPostcondition:
            taskMode === 'RETRIEVE_PEER'
              ? `Rigid cone seated <= 0.065m, cam-latch >= 0.30 rad & victim extracted > 0.08m`
              : taskMode === 'PLANT'
              ? `Rigid-body seed pellet released at v=0 with verified soil contact`
              : `Target waypoint reached within 0.28m probe radius`,
        },
      ],
    };

    return {
      ...updatedRobot,
      status: deriveRobotStatusCode(updatedRobot),
    };
  });

  return {
    robots: updatedRobots,
    accepted: true,
    message: `Dispatched ${targetRobot.callsign}: ${formatRobotPlanSummary(
      targetRobot.callsign,
      taskMode,
      clampedTarget,
      targetPeerId,
      targetSiteId,
      false
    )}`,
  };
}

/**
 * Toggles Emergency Human Teleoperation Control Mode for a specific robot.
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot to switch into or out of human control mode.
 * @param enableHumanControl - True to engage human teleop override, false to return to autonomous mode.
 * @returns Updated array of RobotState objects.
 */
export function setRobotHumanControlMode(
  robots: RobotState[],
  robotId: string,
  enableHumanControl: boolean
): RobotState[] {
  logFunctionCall('setRobotHumanControlMode', { robotId, enableHumanControl });

  return robots.map((robot) => {
    if (robot.id !== robotId) {
      return robot;
    }

    const nextTaskMode: TaskMode = enableHumanControl
      ? 'HOLD'
      : robot.taskMode === 'HOLD'
      ? 'NAVIGATE'
      : robot.taskMode;
    const updatedPlanSummary = formatRobotPlanSummary(
      robot.callsign,
      nextTaskMode,
      robot.targetWaypoint,
      robot.targetPeerId,
      robot.targetSiteId,
      enableHumanControl
    );

    const updated: RobotState = {
      ...robot,
      humanControlActive: enableHumanControl,
      taskMode: nextTaskMode,
      planSummary: updatedPlanSummary,
      telemetry: {
        ...robot.telemetry,
        trueVelocityMs: enableHumanControl ? 0 : robot.telemetry.trueVelocityMs,
        wheelSpeedMs: enableHumanControl ? 0 : robot.telemetry.wheelSpeedMs,
      },
    };

    return {
      ...updated,
      status: deriveRobotStatusCode(updated),
    };
  });
}

/**
 * Triggers or releases Global Fleet Emergency Stop (E-Stop) across all robots in the swarm.
 * @param robots - Current list of fleet RobotState objects.
 * @param engageEmergencyStop - True to halt all robots in HUMAN_CONTROL safety lock; false to resume autonomous plans.
 * @returns Updated array of RobotState objects.
 */
export function setFleetEmergencyStop(
  robots: RobotState[],
  engageEmergencyStop: boolean
): RobotState[] {
  logFunctionCall('setFleetEmergencyStop', { engageEmergencyStop });

  return robots.map((robot) => {
    if (engageEmergencyStop) {
      const updated: RobotState = {
        ...robot,
        humanControlActive: true,
        taskMode: 'HOLD',
        targetWaypoint: { ...robot.position },
        plannedRoute: [{ ...robot.position }],
        planSummary: `${robot.callsign.split('·')[0].trim()} halted by Global Emergency Stop — awaiting human command`,
        telemetry: {
          ...robot.telemetry,
          trueVelocityMs: 0,
          wheelSpeedMs: 0,
          slipRatio: 0,
          motorCurrentAmps: 0.4,
        },
      };
      return {
        ...updated,
        status: 'HUMAN_CONTROL',
      };
    }

    const resumedMode: TaskMode =
      robot.faultScenario !== 'NONE' &&
      robot.faultScenario !== 'NEG_INTENTIONAL_SEED_HOLD' &&
      robot.faultScenario !== 'NEG_MISSING_LIDAR_FAULT'
        ? 'HOLD'
        : robot.targetPeerId
        ? 'RETRIEVE_PEER'
        : robot.seedHopperCount > 0
        ? 'PLANT'
        : 'SAMPLE';

    const resumedWaypoint =
      computeMetricDistance(robot.position, robot.targetWaypoint) < 0.2
        ? {
            x: clampMetric(robot.position.x + 3.0, 1, APP_CONFIG.fieldGeometry.widthMeters - 1),
            y: robot.position.y,
          }
        : robot.targetWaypoint;

    const updated: RobotState = {
      ...robot,
      humanControlActive: false,
      taskMode: resumedMode,
      targetWaypoint: resumedWaypoint,
      plannedRoute: [robot.position, resumedWaypoint],
      planSummary: formatRobotPlanSummary(
        robot.callsign,
        resumedMode,
        resumedWaypoint,
        robot.targetPeerId,
        robot.targetSiteId,
        false
      ),
    };

    return {
      ...updated,
      status: deriveRobotStatusCode(updated),
    };
  });
}

/**
 * Applies a manual human teleoperation drive command or rigid-docking recovery actuator pulse to a robot.
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot being controlled by the human operator.
 * @param command - Directional vector or physical recovery/rigid-docking actuator command.
 * @returns Updated array of RobotState objects.
 */
export function executeManualTeleopCommand(
  robots: RobotState[],
  robotId: string,
  command:
    | { type: 'DRIVE'; dx: number; dy: number; speedScale?: number }
    | { type: 'MICRO_ROCK_CLAY_BREAKER' }
    | { type: 'CLEAR_STRAW_AXLE' }
    | { type: 'TOGGLE_CAM_LATCH' }
    | { type: 'ADJUST_DOCK_PITCH'; deltaRad: number }
    | { type: 'TOGGLE_ANCHOR_SPADES' }
): RobotState[] {
  logFunctionCall('executeManualTeleopCommand', { robotId, command });

  return robots.map((robot) => {
    if (robot.id !== robotId) {
      return robot;
    }

    if (command.type === 'MICRO_ROCK_CLAY_BREAKER') {
      const clearedFault: FaultScenarioId =
        robot.faultScenario === 'S6_CLAY_SUCTION_LOCK' ||
        robot.faultScenario === 'S1_PLOW_PAN_BREACH' ||
        robot.faultScenario === 'S2_ASYMMETRIC_YAW_TRAP' ||
        robot.faultScenario === 'S8_ROW_PINCH_OCCLUSION'
          ? 'NONE'
          : robot.faultScenario;

      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        faultScenario: clearedFault,
        faultReason:
          clearedFault === 'NONE'
            ? null
            : 'Micro-rocking reduced stiction; rigid-dock pitch-lift assist recommended.',
        planSummary: `${robot.callsign.split('·')[0].trim()} executed bounded micro-rocking pulse — mud stiction relieved`,
        telemetry: {
          ...robot.telemetry,
          trueVelocityMs: 0.18,
          wheelSpeedMs: 0.22,
          slipRatio: 0.18,
          motorCurrentAmps: 6.4,
          wheelSinkageMm: 18.0,
          bellyClearanceMm: 26.0,
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    if (command.type === 'CLEAR_STRAW_AXLE') {
      const clearedFault: FaultScenarioId =
        robot.faultScenario === 'S5_STRAW_ENTANGLEMENT' ? 'NONE' : robot.faultScenario;
      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        faultScenario: clearedFault,
        faultReason: clearedFault === 'NONE' ? null : robot.faultReason,
        planSummary: `${robot.callsign.split('·')[0].trim()} executed bounded reverse-polarity clearing oscillation — straw shed`,
        telemetry: {
          ...robot.telemetry,
          motorCurrentAmps: 4.9,
          slipRatio: 0.08,
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    if (command.type === 'TOGGLE_CAM_LATCH') {
      const nextLatched = !robot.telemetry.rigidDock.isRigidLatched;
      const nextLatchRad = nextLatched ? 0.55 : 0.0;
      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        planSummary: nextLatched
          ? `${robot.callsign.split('·')[0].trim()} engaged rotary cam-locking jaw (0.55 rad) — 8-wheel rigid train locked`
          : `${robot.callsign.split('·')[0].trim()} released rotary cam-locking jaw (0.00 rad) — uncoupled`,
        telemetry: {
          ...robot.telemetry,
          rigidDock: {
            ...robot.telemetry.rigidDock,
            camLatchRad: nextLatchRad,
            isRigidLatched: nextLatched,
          },
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    if (command.type === 'ADJUST_DOCK_PITCH') {
      const nextPitch = clampMetric(
        robot.telemetry.rigidDock.dockPitchRad + command.deltaRad,
        APP_CONFIG.roverHomogeneousSpec.dockPitchMinRad,
        APP_CONFIG.roverHomogeneousSpec.dockPitchMaxRad
      );
      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        planSummary: `${robot.callsign.split('·')[0].trim()} set front pitch-lift docking boom to ${nextPitch.toFixed(
          2
        )} rad (+Z suction pry)`,
        telemetry: {
          ...robot.telemetry,
          rigidDock: {
            ...robot.telemetry.rigidDock,
            dockPitchRad: nextPitch,
          },
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    if (command.type === 'TOGGLE_ANCHOR_SPADES') {
      const deployed = robot.telemetry.rigidDock.anchorSpadeRad < 0.2;
      const nextSpadeRad = deployed ? 0.75 : 0.0;
      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        planSummary: deployed
          ? `${robot.callsign.split('·')[0].trim()} deployed rear Danforth hardpan anchor spades (0.75 rad)`
          : `${robot.callsign.split('·')[0].trim()} stowed rear hardpan anchor spades (0.00 rad)`,
        telemetry: {
          ...robot.telemetry,
          rigidDock: {
            ...robot.telemetry.rigidDock,
            anchorSpadeRad: nextSpadeRad,
          },
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    // DRIVE command
    const scale = command.speedScale ?? 1.0;
    const nextX = clampMetric(
      robot.position.x + command.dx * scale,
      0.8,
      APP_CONFIG.fieldGeometry.widthMeters - 0.8
    );
    const nextY = clampMetric(
      robot.position.y + command.dy * scale,
      0.8,
      APP_CONFIG.fieldGeometry.heightMeters - 0.8
    );
    const nextHeading =
      command.dx !== 0 || command.dy !== 0
        ? Math.round((Math.atan2(command.dy, command.dx) * 180) / Math.PI)
        : robot.headingDeg;

    const nextPos = { x: nextX, y: nextY };
    const updatedBreadcrumbs = [...robot.breadcrumbs.slice(-5), nextPos];

    const nextRobot: RobotState = {
      ...robot,
      humanControlActive: true,
      position: nextPos,
      targetWaypoint: nextPos,
      plannedRoute: [nextPos],
      headingDeg: nextHeading,
      planSummary: `${robot.callsign.split('·')[0].trim()} driven manually by human operator to (${nextX.toFixed(
        1
      )}m, ${nextY.toFixed(1)}m)`,
      breadcrumbs: updatedBreadcrumbs,
      telemetry: {
        ...robot.telemetry,
        trueVelocityMs: clampMetric(0.32 * scale, 0.05, 0.45),
        wheelSpeedMs: clampMetric(0.35 * scale, 0.06, 0.45),
        slipRatio: calculateSlipRatio(0.32 * scale, 0.35 * scale),
      },
    };

    return {
      ...nextRobot,
      status: deriveRobotStatusCode(nextRobot),
    };
  });
}

/**
 * Injects one of the S1–S8 physical paddy immobilization scenarios or Negative Controls onto a target robot.
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot to affect.
 * @param scenarioId - FaultScenarioId to inject (or 'NONE' to clear).
 * @returns Updated array of RobotState objects.
 */
export function injectFaultScenario(
  robots: RobotState[],
  robotId: string,
  scenarioId: FaultScenarioId
): RobotState[] {
  logFunctionCall('injectFaultScenario', { robotId, scenarioId });

  const scenarioDef = FAULT_SCENARIO_CATALOG.find((s) => s.id === scenarioId);

  return robots.map((robot) => {
    if (robot.id !== robotId) {
      return robot;
    }

    if (scenarioId === 'NONE' || !scenarioDef) {
      const restoredMode: TaskMode = robot.seedHopperCount > 0 ? 'PLANT' : 'SAMPLE';
      const restored: RobotState = {
        ...robot,
        faultScenario: 'NONE',
        faultReason: null,
        humanControlActive: false,
        taskMode: restoredMode,
        planSummary: formatRobotPlanSummary(
          robot.callsign,
          restoredMode,
          robot.targetWaypoint,
          null,
          robot.targetSiteId,
          false
        ),
        telemetry: {
          ...robot.telemetry,
          trueVelocityMs: 0.39,
          wheelSpeedMs: 0.42,
          slipRatio: 0.07,
          pitchDeg: 0.8,
          rollDeg: -0.4,
          motorCurrentAmps: 5.1,
          frontLidarRangeM: 3.4,
          wheelSinkageMm: 15.0,
          bellyClearanceMm: 29.0,
        },
      };
      return { ...restored, status: deriveRobotStatusCode(restored) };
    }

    const isNegativeControl = scenarioDef.category === 'NEGATIVE_CONTROL';
    const updated: RobotState = {
      ...robot,
      faultScenario: scenarioId,
      faultReason: `${scenarioDef.code} ${scenarioDef.title}: ${scenarioDef.sensorSignature}`,
      humanControlActive: false,
      taskMode: isNegativeControl ? robot.taskMode : 'HOLD',
      targetWaypoint: isNegativeControl ? robot.targetWaypoint : { ...robot.position },
      planSummary: isNegativeControl
        ? `${robot.callsign.split('·')[0].trim()} negative control (${scenarioDef.code}): ${scenarioDef.title} — NO rescue dispatched`
        : `${robot.callsign.split('·')[0].trim()} immobilized (${scenarioDef.code}: ${scenarioDef.title}) — awaiting rigid-dock rescue or human override`,
      telemetry: {
        ...robot.telemetry,
        ...scenarioDef.defaultTelemetryOverride,
      },
    };

    return {
      ...updated,
      status: deriveRobotStatusCode(updated),
    };
  });
}

/**
 * Evaluates candidate rescue policies against the Pareto Multi-Objective Cost Function
 * J = w_t*T + w_e*E + w_c*C + w_s*S + w_r*R (D-0011) and enforces Negative Control gating
 * (`docs/IMMOBILIZATION_SCENARIOS.md`).
 * @param stuckRobot - The target RobotState under evaluation.
 * @param allRobots - All robots in the Kinematic Quintet.
 * @param weights - Optional custom ParetoWeights from interactive UI sliders.
 * @returns Array of RescueCandidateEvaluation objects sorted by recommendation and utility score.
 */
export function evaluateRescueCandidates(
  stuckRobot: RobotState,
  allRobots: RobotState[],
  weights: ParetoWeights = APP_CONFIG.defaultParetoWeights
): RescueCandidateEvaluation[] {
  logFunctionCall('evaluateRescueCandidates', {
    stuckRobotId: stuckRobot.id,
    faultScenario: stuckRobot.faultScenario,
    weights,
  });

  // Normalize weights so sum = 1.0
  const totalW =
    weights.timeWeight +
    weights.energyWeight +
    weights.cropDamageWeight +
    weights.soilDisturbanceWeight +
    weights.recoveryRiskWeight || 1.0;

  const wt = weights.timeWeight / totalW;
  const we = weights.energyWeight / totalW;
  const wc = weights.cropDamageWeight / totalW;
  const ws = weights.soilDisturbanceWeight / totalW;
  const wr = weights.recoveryRiskWeight / totalW;

  // Find closest available homogeneous peer with rigid docking mechanism
  const availableHelpers = allRobots
    .filter(
      (r) =>
        r.id !== stuckRobot.id &&
        r.capabilities.hasRigidDockMechanism &&
        r.faultScenario === 'NONE' &&
        !r.humanControlActive
    )
    .sort(
      (a, b) =>
        computeMetricDistance(a.position, stuckRobot.position) -
        computeMetricDistance(b.position, stuckRobot.position)
    );

  const primaryHelper = availableHelpers[0] ?? allRobots.find((r) => r.id !== stuckRobot.id) ?? null;
  const helperCallsign = primaryHelper ? primaryHelper.callsign.split('·')[0].trim() : 'Robot 1';
  const stuckCallsign = stuckRobot.callsign.split('·')[0].trim();

  const scenarioDef = FAULT_SCENARIO_CATALOG.find((s) => s.id === stuckRobot.faultScenario);
  const recommendedPolicy: RescuePolicyCode =
    scenarioDef?.recommendedPolicy ?? 'DISPATCH_RIGID_DOCK_DIRECT';

  const rawCandidates: Array<
    Omit<RescueCandidateEvaluation, 'netUtilityScore' | 'paretoCostJ' | 'recommended'> & {
      normTime: number;
      normEnergy: number;
      normCrop: number;
      normSoil: number;
      normRisk: number;
    }
  > = [
    {
      policyCode: 'DISPATCH_RIGID_DOCK_DIRECT',
      title: 'Collinear Rigid-Dock, Cam-Latch & Pitch-Lift Extraction',
      coverageDeltaPct: 24.5,
      cropDamageRiskPct: 0.0,
      energyCostWh: 16.5,
      secondaryImmobilizationProbPct: 5.5,
      normTime: 0.28,
      normEnergy: 0.34,
      normCrop: 0.0,
      normSoil: 0.22,
      normRisk: 0.18,
      assignedHelperId: primaryHelper?.id ?? 'ANT-01',
      rationale:
        'Approaches single-file inside 300mm row, locks rotary cam-latch (0.55 rad) onto rear strike bar, pitch-lifts (+0.35 rad) to break suction, and extracts in 8-wheel tandem reverse.',
      actionPlanText: `${helperCallsign} is going to retrieve ${stuckCallsign} via collinear rigid docking & pitch-lift at (${stuckRobot.position.x.toFixed(
        1
      )}m, ${stuckRobot.position.y.toFixed(1)}m)`,
    },
    {
      policyCode: 'DISPATCH_RIGID_DOCK_DETOUR',
      title: 'Zero-Crop-Damage Headland Detour & Rigid-Dock Extraction',
      coverageDeltaPct: 23.0,
      cropDamageRiskPct: 0.0,
      energyCostWh: 24.0,
      secondaryImmobilizationProbPct: 3.0,
      normTime: 0.42,
      normEnergy: 0.48,
      normCrop: 0.0,
      normSoil: 0.12,
      normRisk: 0.10,
      assignedHelperId: primaryHelper?.id ?? 'ANT-01',
      rationale:
        'Enforces strict C=0 zero-crop-damage constraint by routing rescuer via perimeter headland before entering target row for rigid-dock latch extraction.',
      actionPlanText: `${helperCallsign} is going to retrieve ${stuckCallsign} via headland detour & rigid dock at (${stuckRobot.position.x.toFixed(
        1
      )}m, ${stuckRobot.position.y.toFixed(1)}m)`,
    },
    {
      policyCode: 'SELF_RECOVER_BOUNDED',
      title: 'Bounded Reverse-Arc / Micro-Rocking Self-Recovery',
      coverageDeltaPct: 26.0,
      cropDamageRiskPct: 0.0,
      energyCostWh: 5.8,
      secondaryImmobilizationProbPct: 0.0,
      normTime: 0.15,
      normEnergy: 0.12,
      normCrop: 0.0,
      normSoil: 0.18,
      normRisk: 0.0,
      assignedHelperId: stuckRobot.id,
      rationale:
        'Executes bounded low-torque reverse along verified incoming breadcrumbs without risking a second Medic-Ant unit.',
      actionPlanText: `${stuckCallsign} is going to execute bounded self-recovery along incoming breadcrumbs`,
    },
    {
      policyCode: 'REALLOCATE_AND_BEACON',
      title: 'Isolate Unit as Stationary RTK/Gossip Relay & Reallocate Tasks',
      coverageDeltaPct: 19.5,
      cropDamageRiskPct: 0.0,
      energyCostWh: 3.0,
      secondaryImmobilizationProbPct: 0.0,
      normTime: 0.20,
      normEnergy: 0.08,
      normCrop: 0.0,
      normSoil: 0.0,
      normRisk: 0.0,
      assignedHelperId: primaryHelper?.id ?? 'ANT-02',
      rationale:
        'Disables stalled wheel drive on S5 hub entanglement, keeps onboard radio active as a Mycelial gossip relay, and auctions remaining stand gaps to peers.',
      actionPlanText: `${stuckCallsign} converts to stationary gossip relay; ${helperCallsign} takes over remaining planting sites`,
    },
    {
      policyCode: 'SAFE_ABSTAIN_DEFER_HUMAN',
      title: 'Rational Safe Refusal — Mark Hazard & Defer to Human Operator',
      coverageDeltaPct: 18.0,
      cropDamageRiskPct: 0.0,
      energyCostWh: 1.0,
      secondaryImmobilizationProbPct: 0.0,
      normTime: 0.35,
      normEnergy: 0.04,
      normCrop: 0.0,
      normSoil: 0.0,
      normRisk: 0.0,
      assignedHelperId: null,
      rationale:
        'Refuses dangerous extraction (S4 ditch rollover risk or S7 second-rover slip overload) where intervention risks a second casualty; flags human teleop.',
      actionPlanText: `Mark 1.5m SLAM no-go hazard around ${stuckCallsign} and engage Human Control Override`,
    },
  ];

  // If a Negative Control is active, prepend the explicit Negative Control gate result
  if (recommendedPolicy === 'NEGATIVE_CONTROL_NO_RESCUE') {
    rawCandidates.unshift({
      policyCode: 'NEGATIVE_CONTROL_NO_RESCUE',
      title: 'Negative Control Gate — Do Not Dispatch Rescue (False Positive Blocked)',
      coverageDeltaPct: 28.0,
      cropDamageRiskPct: 0.0,
      energyCostWh: 0.0,
      secondaryImmobilizationProbPct: 0.0,
      normTime: 0.02,
      normEnergy: 0.0,
      normCrop: 0.0,
      normSoil: 0.0,
      normRisk: 0.0,
      assignedHelperId: null,
      rationale:
        'Missing forward LiDAR rangefinder (range=None) or intentional stationary seed drop is NOT physical immobilization. Rescue call blocked.',
      actionPlanText: `Negative control verified on ${stuckCallsign}: keep peers on normal planting/sampling tasks`,
    });
  }

  const evaluated: RescueCandidateEvaluation[] = rawCandidates.map((c) => {
    const costJ =
      wt * c.normTime +
      we * c.normEnergy +
      wc * c.normCrop +
      ws * c.normSoil +
      wr * c.normRisk;
    const isRec = c.policyCode === recommendedPolicy;
    const baseUtility = Math.max(40, Math.min(98, (1 - costJ) * 100 + (isRec ? 9.5 : 0)));
    return {
      policyCode: c.policyCode,
      title: c.title,
      recommended: isRec,
      paretoCostJ: Math.round(costJ * 1000) / 1000,
      netUtilityScore: Math.round(baseUtility * 10) / 10,
      coverageDeltaPct: c.coverageDeltaPct,
      cropDamageRiskPct: c.cropDamageRiskPct,
      energyCostWh: c.energyCostWh,
      secondaryImmobilizationProbPct: c.secondaryImmobilizationProbPct,
      assignedHelperId: c.assignedHelperId,
      rationale: c.rationale,
      actionPlanText: c.actionPlanText,
    };
  });

  return evaluated.sort(
    (a, b) => Number(b.recommended) - Number(a.recommended) || b.netUtilityScore - a.netUtilityScore
  );
}

/**
 * Advances the swarm simulation by one deterministic tick (moving active robots along their
 * multi-waypoint planned routes, performing dynamic seed-depletion role switching when a planter
 * runs out of seeds, and completing rigid-dock tandem peer extraction).
 * @param state - Current robots, cropGapSites, and optional dynamic obstacles.
 * @returns Updated robots and cropGapSites after one simulation step.
 */
export function stepSwarmSimulation(state: {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
  obstacles?: DynamicObstacle[];
}): {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
} {
  logFunctionCall('stepSwarmSimulation', {
    robotCount: state.robots.length,
    siteCount: state.cropGapSites.length,
  });

  const updatedSites = state.cropGapSites.map((site) => ({ ...site }));
  const rescuedRobotIds = new Set<string>();
  const obstacles = state.obstacles ?? [];

  const nextRobots = state.robots.map((robot) => {
    if (
      robot.humanControlActive ||
      (robot.faultScenario !== 'NONE' &&
        robot.faultScenario !== 'NEG_INTENTIONAL_SEED_HOLD') ||
      robot.taskMode === 'HOLD' ||
      robot.taskMode === 'BEACON_RELAY'
    ) {
      return robot;
    }

    // Determine next immediate waypoint from plannedRoute (if detouring around obstacle)
    const activeRoute =
      robot.plannedRoute && robot.plannedRoute.length > 1
        ? robot.plannedRoute
        : [robot.position, robot.targetWaypoint];
    const nextImmediateTarget = activeRoute[1] ?? robot.targetWaypoint;

    const distToImmediate = computeMetricDistance(robot.position, nextImmediateTarget);
    const distToFinal = computeMetricDistance(robot.position, robot.targetWaypoint);
    const stepSize = robot.status === 'WARNING_SLIP' ? 0.22 : 0.38;

    if (distToFinal <= stepSize + 0.15) {
      const arrivedPos = { ...robot.targetWaypoint };

      if (robot.taskMode === 'PLANT' && robot.targetSiteId && robot.seedHopperCount > 0) {
        const siteIdx = updatedSites.findIndex((s) => s.id === robot.targetSiteId);
        if (siteIdx !== -1 && updatedSites[siteIdx].status !== 'PLANTED_VERIFIED') {
          updatedSites[siteIdx] = {
            ...updatedSites[siteIdx],
            status: 'PLANTED_VERIFIED',
            assignedRobotId: robot.id,
          };
        }

        const remainingSeeds = Math.max(0, robot.seedHopperCount - 1);

        // Dynamic Seed-Depletion Role Switching (D-0011): If hopper reaches 0, switch to SAMPLE (Scout) role!
        if (remainingSeeds === 0) {
          const nextSampleSite =
            updatedSites.find((s) => s.status === 'MISSING_CROP') ?? updatedSites[0];
          const scoutSummary = `${robot.callsign.split('·')[0].trim()} (0 seeds -> dynamic Scout role) is going to sample soil firmness at ${nextSampleSite.id}`;
          return {
            ...robot,
            position: arrivedPos,
            taskMode: 'SAMPLE' as TaskMode,
            targetWaypoint: { ...nextSampleSite.position },
            plannedRoute: computeDetourRoute(arrivedPos, nextSampleSite.position, obstacles),
            targetSiteId: nextSampleSite.id,
            seedHopperCount: 0,
            planSummary: scoutSummary,
            breadcrumbs: [...robot.breadcrumbs.slice(-5), arrivedPos],
          };
        }

        const nextUnplanted = updatedSites.find(
          (s) => s.status === 'MISSING_CROP' && s.id !== robot.targetSiteId
        );

        if (nextUnplanted) {
          nextUnplanted.status = 'ASSIGNED';
          nextUnplanted.assignedRobotId = robot.id;
          const nextSummary = formatRobotPlanSummary(
            robot.callsign,
            'PLANT',
            nextUnplanted.position,
            null,
            nextUnplanted.id,
            false
          );
          return {
            ...robot,
            position: arrivedPos,
            targetWaypoint: { ...nextUnplanted.position },
            plannedRoute: computeDetourRoute(arrivedPos, nextUnplanted.position, obstacles),
            targetSiteId: nextUnplanted.id,
            seedHopperCount: remainingSeeds,
            planSummary: nextSummary,
            breadcrumbs: [...robot.breadcrumbs.slice(-5), arrivedPos],
          };
        }
      }

      if (robot.taskMode === 'RETRIEVE_PEER' && robot.targetPeerId) {
        rescuedRobotIds.add(robot.targetPeerId);
      }

      return {
        ...robot,
        position: arrivedPos,
        plannedRoute: [arrivedPos],
        breadcrumbs: [...robot.breadcrumbs.slice(-5), arrivedPos],
      };
    }

    // Step toward nextImmediateTarget
    if (distToImmediate <= stepSize + 0.1 && activeRoute.length > 2) {
      const remainingRoute = [nextImmediateTarget, ...activeRoute.slice(2)];
      return {
        ...robot,
        position: nextImmediateTarget,
        plannedRoute: remainingRoute,
        breadcrumbs: [...robot.breadcrumbs.slice(-5), nextImmediateTarget],
      };
    }

    const dx = nextImmediateTarget.x - robot.position.x;
    const dy = nextImmediateTarget.y - robot.position.y;
    const ratio = stepSize / Math.max(0.01, distToImmediate);
    const nextPos: MetricCoordinate = {
      x: clampMetric(robot.position.x + dx * ratio, 0.8, APP_CONFIG.fieldGeometry.widthMeters - 0.8),
      y: clampMetric(robot.position.y + dy * ratio, 0.8, APP_CONFIG.fieldGeometry.heightMeters - 0.8),
    };
    const nextHeading = Math.round((Math.atan2(dy, dx) * 180) / Math.PI);

    return {
      ...robot,
      position: nextPos,
      headingDeg: nextHeading,
      plannedRoute: [nextPos, ...activeRoute.slice(1)],
      breadcrumbs: [...robot.breadcrumbs.slice(-5), nextPos],
    };
  });

  // Complete rigid-dock tandem extraction for any rescued peer
  const finalRobots = nextRobots.map((robot) => {
    if (!rescuedRobotIds.has(robot.id)) {
      return robot;
    }
    const fallbackWaypoint = {
      x: clampMetric(robot.position.x - 1.4, 1.0, APP_CONFIG.fieldGeometry.widthMeters - 1.0),
      y: robot.position.y,
    };
    const recovered: RobotState = {
      ...robot,
      faultScenario: 'NONE',
      faultReason: null,
      taskMode: 'NAVIGATE',
      targetWaypoint: fallbackWaypoint,
      plannedRoute: [robot.position, fallbackWaypoint],
      planSummary: `${robot.callsign.split('·')[0].trim()} extracted >0.08m onto firm loam via rigid cam-latch & pitch-lift — resuming mission`,
      telemetry: {
        ...robot.telemetry,
        trueVelocityMs: 0.34,
        wheelSpeedMs: 0.38,
        slipRatio: 0.11,
        motorCurrentAmps: 5.6,
        wheelSinkageMm: 16.0,
        bellyClearanceMm: 28.0,
      },
    };
    return {
      ...recovered,
      status: deriveRobotStatusCode(recovered),
    };
  });

  return {
    robots: finalRobots,
    cropGapSites: updatedSites,
  };
}
