/**
 * @file src/logic/swarmEngine.ts
 * @description Pure Kinematic, Task Planning, Emergency Control & Multi-Objective Rescue Engine for Antenna.
 *
 * This module encapsulates all deterministic domain logic for the Antenna Multi-Robot Rice Farming System:
 * 1. Kinematic Slip Ratio Calculation: Computes $s = 1 - (v_{\text{true}} / (\omega \cdot r))$ from 60GHz Doppler
 *    radar velocity and wheel encoder speed, classifying traction state into Nominal, Slip Warning, or Critical Immobilization.
 * 2. Natural-Language Plan Formatting: Generates clear, human-readable plan descriptions (e.g.,
 *    "Robot 1 is going to retrieve Robot 3 via UWB winch tow at (14.4m, 8.4m)").
 * 3. Task Reassignment & Capability Enforcement: Validates `TaskIntentV1` dispatches (preventing scouts without seed
 *    hoppers from being assigned `PLANT` tasks, and updating waypoints and status colors accordingly).
 * 4. Emergency Human Control & Global Fleet E-Stop: Transitions individual robots or the entire 5-robot swarm into
 *    `HUMAN_CONTROL` or safe `HOLD` mode, and processes manual operator directional nudges, winch tension adjustments,
 *    micro-rocking clay suction relief, and manual seed releases.
 * 5. Multi-Objective Rescue Coordinator: Evaluates the 5 candidate rescue policies (`SELF_RECOVER_BOUNDED`,
 *    `DISPATCH_RESCUE_DIRECT`, `DISPATCH_RESCUE_DETOUR`, `REALLOCATE_AND_BEACON`, `SAFE_ABSTAIN_DEFER_HUMAN`)
 *    for any immobilized robot (`S1`–`S6`).
 *
 * Use Cases:
 * - Driving the real-time simulation loop and interactive control handlers in the React UI.
 * - Executing deterministic unit tests (`src/logic/swarmEngine.test.ts`) via `node:test`.
 */

import { APP_CONFIG } from '../config/appConfig';
import { FAULT_SCENARIO_CATALOG } from '../data/initialSwarmState';
import {
  CropGapSite,
  FaultScenarioId,
  MetricCoordinate,
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
 * Computes the 200Hz STM32H7 kinematic wheel slip ratio s = 1 - (v_true / v_wheel).
 * @param trueVelocityMs - Doppler radar ground speed in m/s.
 * @param wheelSpeedMs - Wheel encoder linear speed in m/s.
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
      return `${shortName} is going to retrieve ${peerLabel} via UWB winch tow at ${coordText}`;
    }
    case 'PLANT': {
      const siteLabel = targetSiteId ? `missing crop gap ${targetSiteId}` : 'target furrow';
      return `${shortName} is going to plant ${siteLabel} at ${coordText}`;
    }
    case 'SAMPLE': {
      const siteLabel = targetSiteId ? `site ${targetSiteId}` : 'paddy sector';
      return `${shortName} is going to sample soil moisture at ${siteLabel} ${coordText}`;
    }
    case 'BEACON_RELAY':
      return `${shortName} is operating as a stationary Dual-UWB & Wi-Fi HaLow relay beacon at ${coordText}`;
    case 'HOLD':
      return `${shortName} is holding position in deterministic safety lock at ${coordText}`;
    case 'NAVIGATE':
    default:
      return `${shortName} is going to navigate to waypoint ${coordText}`;
  }
}

/**
 * Derives the color-coded `RobotStatusCode` from a robot's current operational state and telemetry.
 * @param robot - RobotState to evaluate.
 * @returns Appropriate semantic RobotStatusCode.
 */
export function deriveRobotStatusCode(robot: RobotState): RobotStatusCode {
  logFunctionCall('deriveRobotStatusCode', { robotId: robot.id });

  if (robot.humanControlActive) {
    return 'HUMAN_CONTROL';
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
 * Assigns a new task plan (`TaskIntentV1`) to a specific robot while validating its hardware capabilities.
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot to reassign.
 * @param taskMode - Desired TaskMode.
 * @param targetCoord - Target metric waypoint.
 * @param options - Optional targetPeerId or targetSiteId.
 * @returns Object containing updated robots array and a validation message.
 */
export function assignRobotTaskPlan(
  robots: RobotState[],
  robotId: string,
  taskMode: TaskMode,
  targetCoord: MetricCoordinate,
  options: { targetPeerId?: string | null; targetSiteId?: string | null } = {}
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

  // Enforce rice.v1 contract: Scouts without seed hoppers cannot accept PLANT tasks
  if (taskMode === 'PLANT' && (!targetRobot.capabilities.canPlantSeed || targetRobot.seedHopperCount <= 0)) {
    return {
      robots,
      accepted: false,
      message: `Safety Gate Rejected: ${targetRobot.callsign} has no active seed dropper / hopper inventory for PLANT mode.`,
    };
  }

  if (taskMode === 'RETRIEVE_PEER' && !targetRobot.capabilities.hasWinchTether) {
    return {
      robots,
      accepted: false,
      message: `Safety Gate Rejected: ${targetRobot.callsign} is not equipped with an INA240 winch tether assembly.`,
    };
  }

  const clampedTarget: MetricCoordinate = {
    x: clampMetric(targetCoord.x, 0.8, APP_CONFIG.fieldGeometry.widthMeters - 0.8),
    y: clampMetric(targetCoord.y, 0.8, APP_CONFIG.fieldGeometry.heightMeters - 0.8),
  };

  const targetPeerId = options.targetPeerId ?? null;
  const targetSiteId = options.targetSiteId ?? null;

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
      trueVelocityMs: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.4,
      wheelSpeedMs: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.44,
      slipRatio: taskMode === 'HOLD' || taskMode === 'BEACON_RELAY' ? 0 : 0.09,
      tetherTensionN: taskMode === 'RETRIEVE_PEER' ? 190 : 0,
    };

    const updatedRobot: RobotState = {
      ...robot,
      taskMode,
      humanControlActive: false,
      faultScenario: clearedFault,
      faultReason: clearedFault === 'NONE' ? null : robot.faultReason,
      targetWaypoint: clampedTarget,
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
              ? `Tether tension nominal (20-350 N) & peer extracted`
              : taskMode === 'PLANT'
              ? `Seed proxy released with verified soil contact`
              : `Target waypoint reached within 0.15m`,
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

    const nextTaskMode: TaskMode = enableHumanControl ? 'HOLD' : robot.taskMode === 'HOLD' ? 'NAVIGATE' : robot.taskMode;
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

    // Resume autonomous operation for non-immobilized units
    const resumedMode: TaskMode =
      robot.faultScenario !== 'NONE'
        ? 'HOLD'
        : robot.targetPeerId
        ? 'RETRIEVE_PEER'
        : robot.capabilities.canPlantSeed
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
 * Applies a manual human teleoperation drive command or recovery actuator pulse to a robot.
 * @param robots - Current list of fleet RobotState objects.
 * @param robotId - ID of the robot being controlled by the human operator.
 * @param command - Directional vector or special recovery actuator command.
 * @returns Updated array of RobotState objects.
 */
export function executeManualTeleopCommand(
  robots: RobotState[],
  robotId: string,
  command:
    | { type: 'DRIVE'; dx: number; dy: number; speedScale?: number }
    | { type: 'MICRO_ROCK_CLAY_BREAKER' }
    | { type: 'CLEAR_STRAW_AXLE' }
    | { type: 'WINCH_TENSION'; deltaN: number }
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
        robot.faultScenario === 'S2_ASYMMETRIC_YAW_TRAP'
          ? 'NONE'
          : robot.faultScenario;

      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        faultScenario: clearedFault,
        faultReason:
          clearedFault === 'NONE'
            ? null
            : 'Micro-rocking reduced suction; external winch assist still recommended.',
        planSummary: `${robot.callsign.split('·')[0].trim()} executed +- micro-rocking torque pulse — clay suction broken`,
        telemetry: {
          ...robot.telemetry,
          trueVelocityMs: 0.18,
          wheelSpeedMs: 0.22,
          slipRatio: 0.18,
          motorCurrentAmps: 6.4,
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
        planSummary: `${robot.callsign.split('·')[0].trim()} executed reverse-polarity hub oscillation — straw cleared`,
        telemetry: {
          ...robot.telemetry,
          motorCurrentAmps: 4.9,
          slipRatio: 0.08,
        },
      };
      return { ...nextRobot, status: deriveRobotStatusCode(nextRobot) };
    }

    if (command.type === 'WINCH_TENSION') {
      const rawTension = robot.telemetry.tetherTensionN + command.deltaN;
      // Enforce INA240 emergency release if tension >= 400 N
      const safeTension =
        rawTension >= APP_CONFIG.safetyThresholds.emergencyReleaseTensionN
          ? 0
          : clampMetric(rawTension, 0, 395);

      const nextRobot: RobotState = {
        ...robot,
        humanControlActive: true,
        planSummary:
          rawTension >= APP_CONFIG.safetyThresholds.emergencyReleaseTensionN
            ? `${robot.callsign.split('·')[0].trim()} INA240 safety gate tripped (>= 400 N) — winch brake released!`
            : `${robot.callsign.split('·')[0].trim()} manual winch tension set to ${safeTension} N`,
        telemetry: {
          ...robot.telemetry,
          tetherTensionN: safeTension,
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
      headingDeg: nextHeading,
      planSummary: `${robot.callsign.split('·')[0].trim()} driven manually by human operator to (${nextX.toFixed(
        1
      )}m, ${nextY.toFixed(1)}m)`,
      breadcrumbs: updatedBreadcrumbs,
      telemetry: {
        ...robot.telemetry,
        trueVelocityMs: clampMetric(0.32 * scale, 0.05, 0.6),
        wheelSpeedMs: clampMetric(0.36 * scale, 0.06, 0.68),
        slipRatio: calculateSlipRatio(0.32 * scale, 0.36 * scale),
      },
    };

    return {
      ...nextRobot,
      status: deriveRobotStatusCode(nextRobot),
    };
  });
}

/**
 * Injects one of the S1–S6 physical paddy immobilization scenarios onto a target robot.
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
      const restoredMode: TaskMode = robot.capabilities.canPlantSeed ? 'PLANT' : 'SAMPLE';
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
          trueVelocityMs: 0.42,
          wheelSpeedMs: 0.46,
          slipRatio: 0.09,
          pitchDeg: 0.8,
          rollDeg: -0.4,
          motorCurrentAmps: 5.1,
        },
      };
      return { ...restored, status: deriveRobotStatusCode(restored) };
    }

    const updated: RobotState = {
      ...robot,
      faultScenario: scenarioId,
      faultReason: `${scenarioDef.code} ${scenarioDef.title}: ${scenarioDef.sensorSignature}`,
      humanControlActive: false,
      taskMode: 'HOLD',
      targetWaypoint: { ...robot.position },
      planSummary: `${robot.callsign.split('·')[0].trim()} immobilized (${scenarioDef.code}: ${scenarioDef.title}) — awaiting rescue or human override`,
      telemetry: {
        ...robot.telemetry,
        ...scenarioDef.defaultTelemetryOverride,
      },
    };

    return {
      ...updated,
      status: 'IMMOBILIZED',
    };
  });
}

/**
 * Evaluates the 5 candidate rescue policies against the Antenna Multi-Objective Cost Function J(action)
 * for a given immobilized robot and current fleet state.
 * @param stuckRobot - The immobilized RobotState.
 * @param allRobots - All robots in the Kinematic Quintet.
 * @returns Array of 5 RescueCandidateEvaluation objects sorted by recommendation and utility score.
 */
export function evaluateRescueCandidates(
  stuckRobot: RobotState,
  allRobots: RobotState[]
): RescueCandidateEvaluation[] {
  logFunctionCall('evaluateRescueCandidates', {
    stuckRobotId: stuckRobot.id,
    faultScenario: stuckRobot.faultScenario,
  });

  // Find closest available helper with a winch tether
  const availableHelpers = allRobots
    .filter(
      (r) =>
        r.id !== stuckRobot.id &&
        r.capabilities.hasWinchTether &&
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
    scenarioDef?.recommendedPolicy ?? 'DISPATCH_RESCUE_DIRECT';

  const candidates: RescueCandidateEvaluation[] = [
    {
      policyCode: 'DISPATCH_RESCUE_DIRECT',
      title: 'Direct UWB Winch Tether Extraction',
      recommended: recommendedPolicy === 'DISPATCH_RESCUE_DIRECT',
      netUtilityScore: recommendedPolicy === 'DISPATCH_RESCUE_DIRECT' ? 88.4 : 71.2,
      coverageDeltaPct: 24.5,
      cropDamageRiskPct: 1.4,
      energyCostWh: 18.0,
      secondaryImmobilizationProbPct: 6.5,
      assignedHelperId: primaryHelper?.id ?? 'ANT-01',
      rationale:
        'Uses Dual-UWB PDoA alignment and INA240 strain-gauge tension control (<= 280 N) along a firm unplanted furrow.',
      actionPlanText: `${helperCallsign} is going to retrieve ${stuckCallsign} via direct UWB winch tow at (${stuckRobot.position.x.toFixed(
        1
      )}m, ${stuckRobot.position.y.toFixed(1)}m)`,
    },
    {
      policyCode: 'DISPATCH_RESCUE_DETOUR',
      title: 'Crop-Safe Perimeter Detour & Angled Tow',
      recommended: recommendedPolicy === 'DISPATCH_RESCUE_DETOUR',
      netUtilityScore: recommendedPolicy === 'DISPATCH_RESCUE_DETOUR' ? 86.9 : 74.0,
      coverageDeltaPct: 23.0,
      cropDamageRiskPct: 0.2,
      energyCostWh: 26.5,
      secondaryImmobilizationProbPct: 4.0,
      assignedHelperId: primaryHelper?.id ?? 'ANT-01',
      rationale:
        'Routes rescuer around established rice hills to guarantee < 0.5% seedling trampling compliance before angled tether pull.',
      actionPlanText: `${helperCallsign} is going to retrieve ${stuckCallsign} via perimeter detour tow at (${stuckRobot.position.x.toFixed(
        1
      )}m, ${stuckRobot.position.y.toFixed(1)}m)`,
    },
    {
      policyCode: 'SELF_RECOVER_BOUNDED',
      title: 'Bounded <= 5s Micro-Rocking Self-Recovery',
      recommended: recommendedPolicy === 'SELF_RECOVER_BOUNDED',
      netUtilityScore: recommendedPolicy === 'SELF_RECOVER_BOUNDED' ? 91.0 : 58.5,
      coverageDeltaPct: 26.0,
      cropDamageRiskPct: 0.8,
      energyCostWh: 6.2,
      secondaryImmobilizationProbPct: 0.0,
      assignedHelperId: stuckRobot.id,
      rationale:
        'Applies alternating low-duty-cycle torque to break clay thixotropy or yaw pocket without risking a second rover.',
      actionPlanText: `${stuckCallsign} is going to execute bounded 5s reverse-arc self-recovery along incoming breadcrumbs`,
    },
    {
      policyCode: 'REALLOCATE_AND_BEACON',
      title: 'Convert to Stationary UWB Relay Beacon & Reallocate Queue',
      recommended: recommendedPolicy === 'REALLOCATE_AND_BEACON',
      netUtilityScore: recommendedPolicy === 'REALLOCATE_AND_BEACON' ? 84.2 : 64.8,
      coverageDeltaPct: 19.5,
      cropDamageRiskPct: 0.0,
      energyCostWh: 3.5,
      secondaryImmobilizationProbPct: 0.0,
      assignedHelperId: primaryHelper?.id ?? 'ANT-02',
      rationale:
        'Disables stalled propulsion, turns immobilized rover into a fixed Dual-UWB localization anchor, and redistributes its seed sites.',
      actionPlanText: `${stuckCallsign} converts to stationary UWB beacon; ${helperCallsign} takes over remaining planting sites`,
    },
    {
      policyCode: 'SAFE_ABSTAIN_DEFER_HUMAN',
      title: 'Safe Abstention — Isolate Hazard & Defer to Human Operator',
      recommended: recommendedPolicy === 'SAFE_ABSTAIN_DEFER_HUMAN',
      netUtilityScore: recommendedPolicy === 'SAFE_ABSTAIN_DEFER_HUMAN' ? 89.5 : 61.0,
      coverageDeltaPct: 18.0,
      cropDamageRiskPct: 0.0,
      energyCostWh: 1.0,
      secondaryImmobilizationProbPct: 0.0,
      assignedHelperId: null,
      rationale:
        'Refuses high-risk tow (> 350 N or deep drainage ditch rollover risk) to prevent trapping a second robot; flags human teleop/retrieval.',
      actionPlanText: `Mark 1.5m no-go hazard boundary around ${stuckCallsign} and engage Human Control Override`,
    },
  ];

  return candidates.sort((a, b) => Number(b.recommended) - Number(a.recommended) || b.netUtilityScore - a.netUtilityScore);
}

/**
 * Advances the swarm simulation by one deterministic tick (moving active robots toward their waypoints
 * and completing seed planting or peer retrieval when within target proximity).
 * @param state - Current robots and cropGapSites arrays.
 * @returns Updated robots and cropGapSites after one simulation step.
 */
export function stepSwarmSimulation(state: {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
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

  const nextRobots = state.robots.map((robot) => {
    // Do not auto-translate robots in HUMAN_CONTROL, HOLD, BEACON_RELAY, or IMMOBILIZED states
    if (
      robot.humanControlActive ||
      robot.faultScenario !== 'NONE' ||
      robot.taskMode === 'HOLD' ||
      robot.taskMode === 'BEACON_RELAY'
    ) {
      return robot;
    }

    const dist = computeMetricDistance(robot.position, robot.targetWaypoint);
    const stepSize = robot.status === 'WARNING_SLIP' ? 0.22 : 0.38;

    if (dist <= stepSize + 0.15) {
      // Arrived at target waypoint
      const arrivedPos = { ...robot.targetWaypoint };

      // Check if planting a missing crop site
      if (robot.taskMode === 'PLANT' && robot.targetSiteId && robot.seedHopperCount > 0) {
        const siteIdx = updatedSites.findIndex((s) => s.id === robot.targetSiteId);
        if (siteIdx !== -1 && updatedSites[siteIdx].status !== 'PLANTED_VERIFIED') {
          updatedSites[siteIdx] = {
            ...updatedSites[siteIdx],
            status: 'PLANTED_VERIFIED',
            assignedRobotId: robot.id,
          };
        }

        // Find next unplanted site
        const nextUnplanted = updatedSites.find(
          (s) => s.status === 'MISSING_CROP' && s.id !== robot.targetSiteId
        );

        if (nextUnplanted && robot.seedHopperCount > 1) {
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
            targetSiteId: nextUnplanted.id,
            seedHopperCount: Math.max(0, robot.seedHopperCount - 1),
            planSummary: nextSummary,
            breadcrumbs: [...robot.breadcrumbs.slice(-5), arrivedPos],
          };
        }
      }

      // Check if retrieving a peer robot
      if (robot.taskMode === 'RETRIEVE_PEER' && robot.targetPeerId) {
        rescuedRobotIds.add(robot.targetPeerId);
      }

      return {
        ...robot,
        position: arrivedPos,
        breadcrumbs: [...robot.breadcrumbs.slice(-5), arrivedPos],
      };
    }

    // Move along vector toward targetWaypoint
    const dx = robot.targetWaypoint.x - robot.position.x;
    const dy = robot.targetWaypoint.y - robot.position.y;
    const ratio = stepSize / dist;
    const nextPos: MetricCoordinate = {
      x: clampMetric(robot.position.x + dx * ratio, 0.8, APP_CONFIG.fieldGeometry.widthMeters - 0.8),
      y: clampMetric(robot.position.y + dy * ratio, 0.8, APP_CONFIG.fieldGeometry.heightMeters - 0.8),
    };
    const nextHeading = Math.round((Math.atan2(dy, dx) * 180) / Math.PI);

    return {
      ...robot,
      position: nextPos,
      headingDeg: nextHeading,
      breadcrumbs: [...robot.breadcrumbs.slice(-5), nextPos],
    };
  });

  // If any peer robot was rescued during this step, clear its immobilization fault
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
      planSummary: `${robot.callsign.split('·')[0].trim()} extracted onto firm furrow by winch tow — resuming mission`,
      telemetry: {
        ...robot.telemetry,
        trueVelocityMs: 0.34,
        wheelSpeedMs: 0.39,
        slipRatio: 0.13,
        motorCurrentAmps: 5.8,
        tetherTensionN: 110,
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
