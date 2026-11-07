/**
 * @file src/App.tsx
 * @description Clean, User-Focused Dashboard Shell for the Antenna Multi-Robot System.
 *
 * Organizes the interface into 4 focused, uncluttered views so information is never crowded in one place:
 * 1. `FIELD_OPS` ("Live Dashboard"):
 *    - Left (3 cols): Compact Robot Roster showing where each robot is, its color-coded status, and its plain-English plan.
 *    - Center (6 cols): Clean Interactive Map with color-coded robot dots, dashed plan paths, and a single streamlined toolbar.
 *    - Right (3 cols): Unified Selected-Robot Card with a prominent "Take Human Control" button and a clean toggle between
 *      "Plan & Status" and "Manual Drive".
 * 2. `ROBOT_PLANS` ("Robot Plans"): Dedicated mission schedule and step-by-step queue for all 5 robots.
 * 3. `FAULT_TRIAGE` ("Faults & Rescue"): Dedicated fault simulator and rescue option comparison.
 * 4. `HUMAN_TELEOP` ("Terrain & Comms"): Dedicated wheel-mud sinkage calculator, P2P comms log, and JSON export.
 */

import React, { useEffect, useState } from 'react';
import { APP_CONFIG } from './config/appConfig';
import { BekkerTerramechanicsLab } from './components/BekkerTerramechanicsLab';
import { EmergencyControlPanel } from './components/EmergencyControlPanel';
import { FieldMapCanvas } from './components/FieldMapCanvas';
import { PlanTimelinePanel } from './components/PlanTimelinePanel';
import { RobotRosterPanel } from './components/RobotRosterPanel';
import { ScenarioTriagePanel } from './components/ScenarioTriagePanel';
import { TelemetryInspector } from './components/TelemetryInspector';
import { TopNavigation, WorkspaceTab } from './components/TopNavigation';
import {
  INITIAL_CROP_GAP_SITES,
  INITIAL_GOSSIP_LOGS,
  INITIAL_HAZARD_ZONES,
  INITIAL_ROBOTS,
} from './data/initialSwarmState';
import {
  assignRobotTaskPlan,
  executeManualTeleopCommand,
  injectDynamicObstacleAndReplan,
  injectFaultScenario,
  setFleetEmergencyStop,
  setRobotHumanControlMode,
  stepSwarmSimulation,
} from './logic/swarmEngine';
import {
  CropGapSite,
  DynamicObstacle,
  FaultScenarioId,
  MetricCoordinate,
  RescueCandidateEvaluation,
  RobotState,
  TaskMode,
} from './types/antenna';
import { logFunctionCall } from './utils/logger';

/**
 * Root application component for the Antenna Swarm Control Dashboard.
 */
export default function App() {
  const [activeTab, setActiveTab] = useState<WorkspaceTab>('FIELD_OPS');
  const [inspectorMode, setInspectorMode] = useState<'PLAN' | 'MANUAL'>('PLAN');
  const [robots, setRobots] = useState<RobotState[]>(INITIAL_ROBOTS);
  const [cropGapSites, setCropGapSites] = useState<CropGapSite[]>(INITIAL_CROP_GAP_SITES);
  const [obstacles, setObstacles] = useState<DynamicObstacle[]>([]);
  const [selectedRobotId, setSelectedRobotId] = useState<string>('ANT-01');
  const [simulationRunning, setSimulationRunning] = useState<boolean>(false);
  const [fleetEmergencyStopActive, setFleetEmergencyStopActive] = useState<boolean>(false);

  // Run periodic simulation steps when simulationRunning is true
  useEffect(() => {
    if (!simulationRunning || fleetEmergencyStopActive) {
      return;
    }

    const timer = window.setInterval(() => {
      setRobots((prevRobots) => {
        const result = stepSwarmSimulation({
          robots: prevRobots,
          cropGapSites,
        });
        setCropGapSites(result.cropGapSites);
        return result.robots;
      });
    }, APP_CONFIG.simulation.defaultTickMs);

    return () => window.clearInterval(timer);
  }, [simulationRunning, fleetEmergencyStopActive, cropGapSites]);

  /**
   * Advances the simulation by a single deterministic step.
   */
  function handleStepSimulationOnce(): void {
    logFunctionCall('App.handleStepSimulationOnce', {});
    const next = stepSwarmSimulation({ robots, cropGapSites });
    setRobots(next.robots);
    setCropGapSites(next.cropGapSites);
  }

  /**
   * Resets all robots, obstacles, and crop gap sites back to the initial baseline.
   */
  function handleResetSwarm(): void {
    logFunctionCall('App.handleResetSwarm', {});
    setSimulationRunning(false);
    setFleetEmergencyStopActive(false);
    setRobots(INITIAL_ROBOTS);
    setCropGapSites(INITIAL_CROP_GAP_SITES);
    setObstacles([]);
    setSelectedRobotId('ANT-01');
    setInspectorMode('PLAN');
  }

  /**
   * Injects a dynamic obstacle along an active robot's route and replans around it.
   */
  function handleInjectDynamicObstacle(): void {
    logFunctionCall('App.handleInjectDynamicObstacle', {
      existingObstacleCount: obstacles.length,
    });
    const result = injectDynamicObstacleAndReplan(robots, obstacles, selectedRobotId);
    setRobots(result.robots);
    setObstacles(result.obstacles);
  }

  /**
   * Exports the current `antenna.dashboard.v1` snapshot as a downloadable JSON file.
   */
  function handleExportDashboardJson(): void {
    logFunctionCall('App.handleExportDashboardJson', {
      schemaVersion: APP_CONFIG.dashboardSchemaVersion,
    });
    const payload = {
      schema_version: APP_CONFIG.dashboardSchemaVersion,
      exported_at: new Date().toISOString(),
      edge_compute_budget: APP_CONFIG.edgeComputeBudget,
      homogeneous_rover_spec: APP_CONFIG.roverHomogeneousSpec,
      robots,
      crop_gap_sites: cropGapSites,
      dynamic_obstacles: obstacles,
      gossip_packets: INITIAL_GOSSIP_LOGS,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], {
      type: 'application/json',
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `antenna.dashboard.v1.snapshot.json`;
    a.click();
    URL.revokeObjectURL(url);
  }

  /**
   * Assigns a new mission plan to a robot.
   */
  function handleAssignTaskPlan(
    robotId: string,
    taskMode: TaskMode,
    targetCoord: MetricCoordinate,
    options?: { targetPeerId?: string | null; targetSiteId?: string | null }
  ): void {
    logFunctionCall('App.handleAssignTaskPlan', {
      robotId,
      taskMode,
      targetCoord,
      options,
    });
    const result = assignRobotTaskPlan(robots, robotId, taskMode, targetCoord, {
      ...options,
      obstacles,
    });
    setRobots(result.robots);
  }

  /**
   * Engages or releases Emergency Human Control Mode on a single robot.
   */
  function handleToggleRobotHumanControl(robotId: string, enable: boolean): void {
    logFunctionCall('App.handleToggleRobotHumanControl', { robotId, enable });
    const nextRobots = setRobotHumanControlMode(robots, robotId, enable);
    setRobots(nextRobots);
    if (enable) {
      setInspectorMode('MANUAL');
    }
  }

  /**
   * Engages or releases Global Fleet Emergency Stop across all 5 robots.
   */
  function handleToggleFleetEmergencyStop(): void {
    const nextStopState = !fleetEmergencyStopActive;
    logFunctionCall('App.handleToggleFleetEmergencyStop', { nextStopState });
    setFleetEmergencyStopActive(nextStopState);
    if (nextStopState) {
      setSimulationRunning(false);
    }
    const nextRobots = setFleetEmergencyStop(robots, nextStopState);
    setRobots(nextRobots);
  }

  /**
   * Executes a manual directional drive command for a robot under human control.
   */
  function handleManualDrive(
    robotId: string,
    dx: number,
    dy: number,
    speedScale: number
  ): void {
    logFunctionCall('App.handleManualDrive', { robotId, dx, dy, speedScale });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'DRIVE',
      dx,
      dy,
      speedScale,
    });
    setRobots(nextRobots);
  }

  /**
   * Executes the micro-rocking clay suction breaker pulse on the specified robot.
   */
  function handleTriggerClayBreaker(robotId: string): void {
    logFunctionCall('App.handleTriggerClayBreaker', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'MICRO_ROCK_CLAY_BREAKER',
    });
    setRobots(nextRobots);
  }

  /**
   * Executes reverse-polarity straw unwrapping on the specified robot.
   */
  function handleTriggerStrawClearing(robotId: string): void {
    logFunctionCall('App.handleTriggerStrawClearing', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'CLEAR_STRAW_AXLE',
    });
    setRobots(nextRobots);
  }

  /**
   * Toggles the internal rotary cam-locking jaw.
   */
  function handleToggleCamLatch(robotId: string): void {
    logFunctionCall('App.handleToggleCamLatch', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'TOGGLE_CAM_LATCH',
    });
    setRobots(nextRobots);
  }

  /**
   * Adjusts the front active pitch-lift docking boom angle in radians.
   */
  function handleAdjustDockPitch(robotId: string, deltaRad: number): void {
    logFunctionCall('App.handleAdjustDockPitch', { robotId, deltaRad });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'ADJUST_DOCK_PITCH',
      deltaRad,
    });
    setRobots(nextRobots);
  }

  /**
   * Toggles the rear deployable hardpan anchor spades.
   */
  function handleToggleAnchorSpades(robotId: string): void {
    logFunctionCall('App.handleToggleAnchorSpades', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'TOGGLE_ANCHOR_SPADES',
    });
    setRobots(nextRobots);
  }

  /**
   * Injects an S1–S8 or Negative Control immobilization fault scenario onto a robot.
   */
  function handleInjectFaultScenario(
    robotId: string,
    scenarioId: FaultScenarioId
  ): void {
    logFunctionCall('App.handleInjectFaultScenario', { robotId, scenarioId });
    const nextRobots = injectFaultScenario(robots, robotId, scenarioId);
    setRobots(nextRobots);
  }

  /**
   * Applies a selected Multi-Objective Rescue Candidate policy to the fleet.
   */
  function handleApplyRescueCandidate(
    stuckRobot: RobotState,
    candidate: RescueCandidateEvaluation
  ): void {
    logFunctionCall('App.handleApplyRescueCandidate', {
      stuckRobotId: stuckRobot.id,
      policyCode: candidate.policyCode,
    });

    if (
      (candidate.policyCode === 'DISPATCH_RIGID_DOCK_DIRECT' ||
        candidate.policyCode === 'DISPATCH_RIGID_DOCK_DETOUR') &&
      candidate.assignedHelperId
    ) {
      const result = assignRobotTaskPlan(
        robots,
        candidate.assignedHelperId,
        'RETRIEVE_PEER',
        stuckRobot.position,
        { targetPeerId: stuckRobot.id, obstacles }
      );
      setRobots(result.robots);
      setSelectedRobotId(candidate.assignedHelperId);
      setActiveTab('FIELD_OPS');
      return;
    }

    if (candidate.policyCode === 'SELF_RECOVER_BOUNDED') {
      handleTriggerClayBreaker(stuckRobot.id);
      return;
    }

    if (candidate.policyCode === 'REALLOCATE_AND_BEACON') {
      const result = assignRobotTaskPlan(
        robots,
        stuckRobot.id,
        'BEACON_RELAY',
        stuckRobot.position,
        { obstacles }
      );
      setRobots(result.robots);
      return;
    }

    if (candidate.policyCode === 'SAFE_ABSTAIN_DEFER_HUMAN') {
      const nextRobots = setRobotHumanControlMode(robots, stuckRobot.id, true);
      setRobots(nextRobots);
      setSelectedRobotId(stuckRobot.id);
      setInspectorMode('MANUAL');
      setActiveTab('FIELD_OPS');
    }
  }

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];
  const selectedTheme = APP_CONFIG.statusTheme[selectedRobot.status];

  return (
    <div id="top" className="min-h-screen flex flex-col bg-slate-950 text-slate-100">
      {/* Clean 3-Zone Top Bar */}
      <TopNavigation
        activeTab={activeTab}
        onSelectTab={setActiveTab}
        fleetEmergencyStopActive={fleetEmergencyStopActive}
        onToggleFleetEmergencyStop={handleToggleFleetEmergencyStop}
      />

      {/* Main Content Area */}
      <main className="flex-1 max-w-[1600px] w-full mx-auto px-5 py-4">
        {activeTab === 'FIELD_OPS' && (
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 items-start">
            {/* Left Sidebar (3 cols): Clean Robot Status & Plan Roster */}
            <div className="lg:col-span-3">
              <RobotRosterPanel
                robots={robots}
                selectedRobotId={selectedRobot.id}
                onSelectRobot={setSelectedRobotId}
                onToggleRobotHumanControl={handleToggleRobotHumanControl}
              />
            </div>

            {/* Center Stage (6 cols): Uncluttered Multi-Robot Map */}
            <div className="lg:col-span-6">
              <FieldMapCanvas
                robots={robots}
                cropGapSites={cropGapSites}
                hazardZones={INITIAL_HAZARD_ZONES}
                obstacles={obstacles}
                selectedRobotId={selectedRobot.id}
                onSelectRobot={setSelectedRobotId}
                onDispatchRobotToPoint={handleAssignTaskPlan}
                onInjectDynamicObstacle={handleInjectDynamicObstacle}
                simulationRunning={simulationRunning}
                onToggleSimulation={() => setSimulationRunning((v) => !v)}
                onStepSimulationOnce={handleStepSimulationOnce}
                onResetSwarm={handleResetSwarm}
              />
            </div>

            {/* Right Inspector (3 cols): Single Focused Selected-Robot Card */}
            <div className="lg:col-span-3 bg-slate-900 border border-slate-800 rounded-lg overflow-hidden">
              {/* Selected Robot Header & Prominent Emergency Human Control Toggle */}
              <div className="p-3.5 bg-slate-950 border-b border-slate-800 space-y-3">
                <div className="flex items-center justify-between gap-2">
                  <div>
                    <h2 className="text-sm font-semibold text-slate-100">
                      {selectedRobot.callsign.split('(')[0].trim()}
                    </h2>
                    <span className="text-xs text-slate-400">
                      Position ({selectedRobot.position.x.toFixed(1)}m, {selectedRobot.position.y.toFixed(1)}m)
                    </span>
                  </div>
                  <span className={`text-xs font-mono font-semibold ${selectedTheme.textClass}`}>
                    {selectedTheme.glyph} {selectedTheme.shortLabel}
                  </span>
                </div>

                {/* Primary Emergency Human Control Button */}
                <button
                  type="button"
                  onClick={() => {
                    const nextState = !selectedRobot.humanControlActive;
                    handleToggleRobotHumanControl(selectedRobot.id, nextState);
                  }}
                  className={`w-full py-2 px-3 text-xs font-semibold rounded-md border transition-colors cursor-pointer ${
                    selectedRobot.humanControlActive
                      ? 'bg-purple-600 border-purple-400 text-white hover:bg-purple-500'
                      : 'bg-purple-950/60 border-purple-500/50 text-purple-200 hover:bg-purple-900/70'
                  }`}
                >
                  {selectedRobot.humanControlActive
                    ? `Exit Human Control (${selectedRobot.id})`
                    : `Take Human Control of ${selectedRobot.id}`}
                </button>

                {/* Sub-Tab Switcher: Plan & Status vs Manual Drive */}
                <div className="flex items-center gap-1 p-1 bg-slate-900 border border-slate-800 rounded-md">
                  <button
                    type="button"
                    onClick={() => setInspectorMode('PLAN')}
                    className={`flex-1 py-1 text-xs font-medium rounded transition-colors cursor-pointer ${
                      inspectorMode === 'PLAN'
                        ? 'bg-slate-800 text-slate-100'
                        : 'text-slate-400 hover:text-slate-200'
                    }`}
                  >
                    Plan &amp; Status
                  </button>
                  <button
                    type="button"
                    onClick={() => setInspectorMode('MANUAL')}
                    className={`flex-1 py-1 text-xs font-medium rounded transition-colors cursor-pointer ${
                      inspectorMode === 'MANUAL'
                        ? 'bg-slate-800 text-purple-300'
                        : 'text-slate-400 hover:text-slate-200'
                    }`}
                  >
                    Manual Drive
                  </button>
                </div>
              </div>

              {/* Render only one focused inspector view at a time */}
              {inspectorMode === 'PLAN' ? (
                <TelemetryInspector
                  selectedRobot={selectedRobot}
                  allRobots={robots}
                  cropGapSites={cropGapSites}
                  onAssignTaskPlan={handleAssignTaskPlan}
                />
              ) : (
                <EmergencyControlPanel
                  selectedRobot={selectedRobot}
                  fleetEmergencyStopActive={fleetEmergencyStopActive}
                  onToggleRobotHumanControl={handleToggleRobotHumanControl}
                  onToggleFleetEmergencyStop={handleToggleFleetEmergencyStop}
                  onManualDrive={handleManualDrive}
                  onTriggerClayBreaker={handleTriggerClayBreaker}
                  onTriggerStrawClearing={handleTriggerStrawClearing}
                  onToggleCamLatch={handleToggleCamLatch}
                  onAdjustDockPitch={handleAdjustDockPitch}
                  onToggleAnchorSpades={handleToggleAnchorSpades}
                />
              )}
            </div>
          </div>
        )}

        {activeTab === 'ROBOT_PLANS' && (
          <PlanTimelinePanel
            robots={robots}
            cropGapSites={cropGapSites}
            selectedRobotId={selectedRobot.id}
            onSelectRobot={setSelectedRobotId}
            onAssignTaskPlan={handleAssignTaskPlan}
          />
        )}

        {activeTab === 'FAULT_TRIAGE' && (
          <ScenarioTriagePanel
            robots={robots}
            selectedRobotId={selectedRobot.id}
            onSelectRobot={setSelectedRobotId}
            onInjectFaultScenario={handleInjectFaultScenario}
            onApplyRescueCandidate={handleApplyRescueCandidate}
          />
        )}

        {activeTab === 'HUMAN_TELEOP' && (
          <BekkerTerramechanicsLab
            gossipLogs={INITIAL_GOSSIP_LOGS}
            onExportDashboardJson={handleExportDashboardJson}
          />
        )}
      </main>
    </div>
  );
}
