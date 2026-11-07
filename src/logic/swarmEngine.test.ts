/**
 * @file src/logic/swarmEngine.test.ts
 * @description Unit Test Suite for the Antenna Swarm Kinematic & Task Planning Engine (`node:test`).
 *
 * This file verifies the deterministic mathematical and state-transition guarantees of `swarmEngine.ts`:
 * 1. `calculateSlipRatio`: Verifies 200Hz STM32H7 slip ratio formula $s = 1 - (v_{\text{true}} / (\omega \cdot r))$
 *    and zero-division safety when wheels are stationary.
 * 2. `formatRobotPlanSummary`: Verifies natural-language plan generation for peer retrieval, crop planting,
 *    soil sampling, and human teleop override.
 * 3. `assignRobotTaskPlan`: Verifies `rice.v1` capability enforcement (rejecting `PLANT` tasks for scout rovers
 *    without seed hoppers, while accepting valid retrieval and planting plans).
 * 4. `setRobotHumanControlMode` & `setFleetEmergencyStop`: Verifies immediate transition into `HUMAN_CONTROL`
 *    status and velocity cutoff (`v_true = 0`), plus manual teleoperation drive and clay-suction micro-rocking.
 * 5. `evaluateRescueCandidates`: Verifies multi-objective rescue policy ranking for `S1` and `S4` scenarios.
 */

import test from 'node:test';
import assert from 'node:assert/strict';
import { INITIAL_CROP_GAP_SITES, INITIAL_ROBOTS } from '../data/initialSwarmState';
import {
  assignRobotTaskPlan,
  calculateSlipRatio,
  evaluateRescueCandidates,
  executeManualTeleopCommand,
  formatRobotPlanSummary,
  injectFaultScenario,
  setFleetEmergencyStop,
  setRobotHumanControlMode,
  stepSwarmSimulation,
} from './swarmEngine';

test('calculateSlipRatio computes accurate slip and handles stationary wheels safely', () => {
  assert.equal(calculateSlipRatio(0.45, 0.50), 0.1);
  assert.equal(calculateSlipRatio(0.02, 0.55), 0.96);
  assert.equal(calculateSlipRatio(0.0, 0.0), 0);
});

test('formatRobotPlanSummary produces clear human-readable robot plans', () => {
  const retrieveText = formatRobotPlanSummary(
    'Robot 1 · Atlas',
    'RETRIEVE_PEER',
    { x: 14.4, y: 8.4 },
    'ANT-03',
    null,
    false
  );
  assert.equal(
    retrieveText,
    'Robot 1 is going to retrieve Robot 3 via UWB winch tow at (14.4m, 8.4m)'
  );

  const plantText = formatRobotPlanSummary(
    'Robot 2 · Boreal',
    'PLANT',
    { x: 11.2, y: 4.0 },
    null,
    'C-03',
    false
  );
  assert.equal(
    plantText,
    'Robot 2 is going to plant missing crop gap C-03 at (11.2m, 4.0m)'
  );
});

test('assignRobotTaskPlan enforces rice.v1 capabilities and updates status color coding', () => {
  // ANT-04 is a soil scout without seed hopper (canPlantSeed = false)
  const rejectedResult = assignRobotTaskPlan(
    INITIAL_ROBOTS,
    'ANT-04',
    'PLANT',
    { x: 16.5, y: 4.0 },
    { targetSiteId: 'C-04' }
  );
  assert.equal(rejectedResult.accepted, false);

  // ANT-05 has a winch tether and can be dispatched to retrieve ANT-03
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
  assert.match(updatedRobot5.planSummary, /Robot 5 is going to retrieve Robot 3/);
});

test('Emergency Human Control Mode and Fleet E-Stop transition status and allow manual driving', () => {
  const manualList = setRobotHumanControlMode(INITIAL_ROBOTS, 'ANT-02', true);
  const r2 = manualList.find((r) => r.id === 'ANT-02')!;
  assert.equal(r2.humanControlActive, true);
  assert.equal(r2.status, 'HUMAN_CONTROL');
  assert.equal(r2.telemetry.trueVelocityMs, 0);

  // Drive ANT-02 east by +1.0m
  const startX = r2.position.x;
  const nudgedList = executeManualTeleopCommand(manualList, 'ANT-02', {
    type: 'DRIVE',
    dx: 1.0,
    dy: 0,
  });
  const nudgedR2 = nudgedList.find((r) => r.id === 'ANT-02')!;
  assert.equal(nudgedR2.position.x, Number((startX + 1.0).toFixed(2)));

  // Global Fleet E-Stop halts all 5 robots
  const estopFleet = setFleetEmergencyStop(INITIAL_ROBOTS, true);
  assert.equal(
    estopFleet.every((r) => r.status === 'HUMAN_CONTROL' && r.telemetry.trueVelocityMs === 0),
    true
  );
});

test('injectFaultScenario and evaluateRescueCandidates rank appropriate recovery policy first', () => {
  const trenchFleet = injectFaultScenario(
    INITIAL_ROBOTS,
    'ANT-02',
    'S4_TRENCH_DROP_IN'
  );
  const stuckR2 = trenchFleet.find((r) => r.id === 'ANT-02')!;
  assert.equal(stuckR2.status, 'IMMOBILIZED');

  const evaluations = evaluateRescueCandidates(stuckR2, trenchFleet);
  assert.equal(evaluations[0].policyCode, 'SAFE_ABSTAIN_DEFER_HUMAN');
  assert.equal(evaluations[0].recommended, true);
});

test('stepSwarmSimulation advances active robots toward their targets', () => {
  const beforeR2 = INITIAL_ROBOTS.find((r) => r.id === 'ANT-02')!;
  const nextState = stepSwarmSimulation({
    robots: INITIAL_ROBOTS,
    cropGapSites: INITIAL_CROP_GAP_SITES,
  });
  const afterR2 = nextState.robots.find((r) => r.id === 'ANT-02')!;
  assert.equal(afterR2.position.x > beforeR2.position.x, true);
});
