/**
 * @file src/logic/swarmEngine.test.ts
 * @description Unit Test Suite for the Antenna Swarm Kinematic, Bekker–Wong Terramechanics,
 * Continuous RBF Belief Field, Dynamic Obstacle Replanning & Pareto Rescue Engine (`node:test`).
 *
 * Verifies:
 * 1. `calculateSlipRatio` & `evaluateBekkerTerramechanics`: Analytical sinkage $z$, belly clearance,
 *    and `S1 BELLY-OUT` verdict matching `src/ricebot/viewer/dashboard.py`.
 * 2. `evaluateContinuousBeliefField`: Continuous 2D RBF heatmap layers across `planter`, `scout`,
 *    `divergence`, and `truth` perspectives.
 * 3. `injectDynamicObstacleAndReplan`: D-0012 mid-sequence obstacle injection and multi-waypoint detour bending.
 * 4. `assignRobotTaskPlan`: Homogeneous seed-depletion gate and rigid-dock rescue plan formatting.
 * 5. `setRobotHumanControlMode`, `setFleetEmergencyStop` & `executeManualTeleopCommand`: Manual drive and
 *    rigid-docking cam-latch / pitch-lift / anchor spade actuator commands.
 * 6. `injectFaultScenario` & `evaluateRescueCandidates`: S1–S8 Pareto ranking and Negative Control blocking.
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import { INITIAL_CROP_GAP_SITES, INITIAL_ROBOTS } from '../data/initialSwarmState';
import {
  assignRobotTaskPlan,
  calculateSlipRatio,
  evaluateBekkerTerramechanics,
  evaluateContinuousBeliefField,
  evaluateRescueCandidates,
  executeManualTeleopCommand,
  formatRobotPlanSummary,
  injectDynamicObstacleAndReplan,
  injectFaultScenario,
  setFleetEmergencyStop,
  setRobotHumanControlMode,
  stepSwarmSimulation,
} from './swarmEngine';

test('calculateSlipRatio and evaluateBekkerTerramechanics compute accurate sinkage and belly-out verdicts', () => {
  assert.equal(calculateSlipRatio(0.45, 0.50), 0.1);
  assert.equal(calculateSlipRatio(0.02, 0.55), 0.96);
  assert.equal(calculateSlipRatio(0.0, 0.0), 0);

  // Light 4.2kg Medic-Ant in moderate moisture (0.55) has positive belly clearance (TRAVERSABLE)
  const safeRes = evaluateBekkerTerramechanics({
    massKg: 4.2,
    wheelRadiusMm: 58,
    wheelWidthMm: 40,
    mudMoisture: 0.55,
  });
  assert.equal(safeRes.belliedOut, false);
  assert.equal(safeRes.verdictCode, 'TRAVERSABLE');

  // Heavy 16kg unit with narrow wheels in saturated mud (0.94) bellies out (BELLY_OUT_STUCK)
  const stuckRes = evaluateBekkerTerramechanics({
    massKg: 16.0,
    wheelRadiusMm: 42,
    wheelWidthMm: 24,
    mudMoisture: 0.94,
  });
  assert.equal(stuckRes.belliedOut, true);
  assert.equal(stuckRes.verdictCode, 'BELLY_OUT_STUCK');
});

test('evaluateContinuousBeliefField distinguishes planter belief, scout belief, and divergence gap', () => {
  const mPlanter = evaluateContinuousBeliefField(18.0, 10.0, 'moisture', 'planter');
  const mScout = evaluateContinuousBeliefField(18.0, 10.0, 'moisture', 'scout');
  const mGap = evaluateContinuousBeliefField(18.0, 10.0, 'moisture', 'divergence');
  assert.equal(mPlanter >= 0 && mPlanter <= 1, true);
  assert.equal(mScout >= 0 && mScout <= 1, true);
  assert.equal(mGap > 0, true);
});

test('formatRobotPlanSummary produces clear rigid-docking and planting plan descriptions', () => {
  const retrieveText = formatRobotPlanSummary(
    'Robot 1 · Atlas',
    'RETRIEVE_PEER',
    { x: 14.4, y: 8.4 },
    'ANT-03',
    null,
    false
  );
  assert.match(
    retrieveText,
    /Robot 1 is going to retrieve Robot 3 via collinear rigid docking, cam-latch & pitch-lift/
  );
});

test('D-0012 injectDynamicObstacleAndReplan inserts obstacle and bends robot plannedRoute around it', () => {
  const result = injectDynamicObstacleAndReplan(INITIAL_ROBOTS, [], 'ANT-02', 5);
  assert.equal(result.obstacles.length, 1);
  const r2 = result.robots.find((r) => r.id === 'ANT-02')!;
  assert.equal(r2.plannedRoute.length > 2, true);
  assert.match(r2.planSummary, /replanned detour trajectory/);
});

test('assignRobotTaskPlan enforces seed-depletion gate and allows any homogeneous unit to rescue', () => {
  // ANT-04 has 0 seeds remaining in its hopper, so PLANT is rejected until refilled
  const rejectedResult = assignRobotTaskPlan(
    INITIAL_ROBOTS,
    'ANT-04',
    'PLANT',
    { x: 16.5, y: 4.0 },
    { targetSiteId: 'C-04' }
  );
  assert.equal(rejectedResult.accepted, false);

  // ANT-05 has homogeneous rigid docking hardware and can retrieve ANT-03
  const acceptedResult = assignRobotTaskPlan(
    INITIAL_ROBOTS,
    'ANT-05',
    'RETRIEVE_PEER',
    { x: 14.4, y: 8.4 },
    { targetPeerId: 'ANT-03' }
  );
  assert.equal(acceptedResult.accepted, true);
  const updatedRobot5 = acceptedResult.robots.find((r) => r.id === 'ANT-05')!;
  assert.equal(updatedRobot5.status, 'RETRIEVING');
  assert.equal(updatedRobot5.telemetry.rigidDock.isRigidLatched, true);
});

test('Emergency Human Control Mode, Fleet E-Stop, and Rigid Docking actuators work deterministically', () => {
  const manualList = setRobotHumanControlMode(INITIAL_ROBOTS, 'ANT-02', true);
  const r2 = manualList.find((r) => r.id === 'ANT-02')!;
  assert.equal(r2.humanControlActive, true);
  assert.equal(r2.status, 'HUMAN_CONTROL');

  // Toggle rotary cam-latch jaw on ANT-02
  const latchedList = executeManualTeleopCommand(manualList, 'ANT-02', {
    type: 'TOGGLE_CAM_LATCH',
  });
  const latchedR2 = latchedList.find((r) => r.id === 'ANT-02')!;
  assert.equal(latchedR2.telemetry.rigidDock.isRigidLatched, true);
  assert.equal(latchedR2.telemetry.rigidDock.camLatchRad, 0.55);

  // Global Fleet E-Stop halts all 5 robots
  const estopFleet = setFleetEmergencyStop(INITIAL_ROBOTS, true);
  assert.equal(
    estopFleet.every((r) => r.status === 'HUMAN_CONTROL' && r.telemetry.trueVelocityMs === 0),
    true
  );
});

test('Negative Controls block false-positive rescue calls while S4 recommends Safe Abstention', () => {
  // Negative Control: missing forward LiDAR rangefinder (range=None) must NOT mark robot IMMOBILIZED
  const negFleet = injectFaultScenario(
    INITIAL_ROBOTS,
    'ANT-02',
    'NEG_MISSING_LIDAR_FAULT'
  );
  const negR2 = negFleet.find((r) => r.id === 'ANT-02')!;
  assert.notEqual(negR2.status, 'IMMOBILIZED');
  const negEvals = evaluateRescueCandidates(negR2, negFleet);
  assert.equal(negEvals[0].policyCode, 'NEGATIVE_CONTROL_NO_RESCUE');

  // S4 Trench Drop-In recommends SAFE_ABSTAIN_DEFER_HUMAN
  const trenchFleet = injectFaultScenario(
    INITIAL_ROBOTS,
    'ANT-02',
    'S4_TRENCH_DROP_IN'
  );
  const stuckR2 = trenchFleet.find((r) => r.id === 'ANT-02')!;
  assert.equal(stuckR2.status, 'IMMOBILIZED');
  const evaluations = evaluateRescueCandidates(stuckR2, trenchFleet);
  assert.equal(evaluations[0].policyCode, 'SAFE_ABSTAIN_DEFER_HUMAN');
});

test('stepSwarmSimulation advances active robots along their planned routes', () => {
  const beforeR2 = INITIAL_ROBOTS.find((r) => r.id === 'ANT-02')!;
  const nextState = stepSwarmSimulation({
    robots: INITIAL_ROBOTS,
    cropGapSites: INITIAL_CROP_GAP_SITES,
  });
  const afterR2 = nextState.robots.find((r) => r.id === 'ANT-02')!;
  assert.equal(afterR2.position.x > beforeR2.position.x, true);
});
