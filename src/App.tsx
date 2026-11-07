/**
 * @file src/App.tsx
 * @description Root Application Shell for the Antenna Swarm Control Console (`AgentVerse_Kinematic_Quintet`).
 *
 * This component orchestrates the state and user workflows of the multi-robot rice farming UI,
 * refined against the canonical repository specifications (`src/ricebot/viewer/README.md`,
 * `src/ricebot/robot/README.md`, `docs/IMMOBILIZATION_SCENARIOS.md`, `docs/DECISIONS.md` `D-0011` & `D-0012`):
 * 1. Maintains the live state of the 5 Homogeneous "Medic-Ant" Rovers (`ANT-01` through `ANT-05`),
 *    missing rice crop gap sites (`C-01`–`C-08` across Section A & Section B), paddy hazard zones,
 *    and dynamically injected mid-run obstacles (`D-0012`).
 * 2. Runs a deterministic, pausable simulation loop that updates robot positions along multi-waypoint
 *    A* routes, seed hopper counts (with automatic 0-seed Scout role transition), and collinear rigid
 *    docking / cam-latch / pitch-lift peer extraction.
 * 3. Integrates the 4 primary operational views plus the `BekkerTerramechanicsLab` (`antenna.dashboard.v1`).
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
 * Root application component for the Antenna Swarm Control Console.
 */
export default function App() {
  const [activeTab, setActiveTab] = useState<WorkspaceTab>('FIELD_OPS');
  const [robots, setRobots] = useState<RobotState[]>(INITIAL_ROBOTS);
  const [cropGapSites, setCropGapSites] = useState<CropGapSite[]>(INITIAL_CROP_GAP_SITES);
  const [obstacles, setObstacles] = useState<DynamicObstacle[]>([]);
  const [selectedRobotId, setSelectedRobotId] = useState<string>('ANT-01');
  const [simulationRunning, setSimulationRunning] = useState<boolean>(false);
  const [fleetEmergencyStopActive, setFleetEmergencyStopActive] = useState<boolean>(false);
  const [statusBannerText, setStatusBannerText] = useState<string>(
    'Swarm Telemetry Nominal · Robot 1 is going to retrieve Robot 3 via collinear rigid docking, cam-latch & pitch-lift at (14.4m, 8.4m)'
  );

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
    setStatusBannerText('Executed +1 deterministic simulation tick (stepSwarmSimulation).');
  }

  /**
   * Resets all robots, obstacles, and crop gap sites back to the initial deterministic baseline.
   */
  function handleResetSwarm(): void {
    logFunctionCall('App.handleResetSwarm', {});
    setSimulationRunning(false);
    setFleetEmergencyStopActive(false);
    setRobots(INITIAL_ROBOTS);
    setCropGapSites(INITIAL_CROP_GAP_SITES);
    setObstacles([]);
    setSelectedRobotId('ANT-01');
    setStatusBannerText('Swarm state reset to deterministic Terrace Sector B-04 baseline.');
  }

  /**
   * Injects a sudden dynamic obstacle along an active robot's route and triggers A* multi-waypoint replanning (`D-0012`).
   */
  function handleInjectDynamicObstacle(): void {
    logFunctionCall('App.handleInjectDynamicObstacle', {
      existingObstacleCount: obstacles.length,
    });
    const result = injectDynamicObstacleAndReplan(robots, obstacles, selectedRobotId);
    setRobots(result.robots);
    setObstacles(result.obstacles);
    setStatusBannerText(result.message);
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
    setStatusBannerText('Exported antenna.dashboard.v1 JSON snapshot.');
  }

  /**
   * Assigns a new mission plan (`TaskIntentV1`) to a robot and updates the status banner.
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
    setStatusBannerText(result.message);
  }

  /**
   * Engages or releases Emergency Human Control Mode on a single robot.
   */
  function handleToggleRobotHumanControl(robotId: string, enable: boolean): void {
    logFunctionCall('App.handleToggleRobotHumanControl', { robotId, enable });
    const nextRobots = setRobotHumanControlMode(robots, robotId, enable);
    setRobots(nextRobots);
    const target = nextRobots.find((r) => r.id === robotId);
    if (target) {
      setStatusBannerText(target.planSummary);
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
    setStatusBannerText(
      nextStopState
        ? 'EMERGENCY FLEET STOP ACTIVE: All 5 robots transitioned to Human Control Safety Lock (v = 0 m/s).'
        : 'Global Emergency Stop released: Autonomous robot plans resumed.'
    );
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
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
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
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
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
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
  }

  /**
   * Toggles the internal rotary cam-locking jaw (`paddy_ant_rigid_rescue.xml`).
   */
  function handleToggleCamLatch(robotId: string): void {
    logFunctionCall('App.handleToggleCamLatch', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'TOGGLE_CAM_LATCH',
    });
    setRobots(nextRobots);
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
  }

  /**
   * Adjusts the front active pitch-lift docking boom angle in radians (`-0.20` to `+0.45 rad`).
   */
  function handleAdjustDockPitch(robotId: string, deltaRad: number): void {
    logFunctionCall('App.handleAdjustDockPitch', { robotId, deltaRad });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'ADJUST_DOCK_PITCH',
      deltaRad,
    });
    setRobots(nextRobots);
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
  }

  /**
   * Toggles the rear deployable hardpan anchor spades (`0.85 rad` deployed / `0.00 rad` stowed).
   */
  function handleToggleAnchorSpades(robotId: string): void {
    logFunctionCall('App.handleToggleAnchorSpades', { robotId });
    const nextRobots = executeManualTeleopCommand(robots, robotId, {
      type: 'TOGGLE_ANCHOR_SPADES',
    });
    setRobots(nextRobots);
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
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
    const updated = nextRobots.find((r) => r.id === robotId);
    if (updated) {
      setStatusBannerText(updated.planSummary);
    }
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
      setStatusBannerText(`Rescue Dispatched: ${candidate.actionPlanText}`);
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
      setStatusBannerText(candidate.actionPlanText);
      return;
    }

    if (candidate.policyCode === 'NEGATIVE_CONTROL_NO_RESCUE') {
      setStatusBannerText(
        `Negative Control Verified: No rescue dispatched for ${stuckRobot.id} (${candidate.actionPlanText})`
      );
      return;
    }

    if (candidate.policyCode === 'SAFE_ABSTAIN_DEFER_HUMAN') {
      const nextRobots = setRobotHumanControlMode(robots, stuckRobot.id, true);
      setRobots(nextRobots);
      setSelectedRobotId(stuckRobot.id);
      setStatusBannerText(
        `Safe Abstention: Refused risky extraction for ${stuckRobot.id}; switched to Human Teleop Override.`
      );
    }
  }

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];
  const plantedCount = cropGapSites.filter((s) => s.status === 'PLANTED_VERIFIED').length;
  const totalSeedsRemaining = robots.reduce((acc, r) => acc + r.seedHopperCount, 0);
  const humanControlledCount = robots.filter((r) => r.humanControlActive).length;

  return (
    <div id="top" className="min-h-screen flex flex-col bg-slate-950 text-slate-100">
      {/* Top Bar Contract Header */}
      <TopNavigation
        activeTab={activeTab}
        onSelectTab={setActiveTab}
        fleetEmergencyStopActive={fleetEmergencyStopActive}
        onToggleFleetEmergencyStop={handleToggleFleetEmergencyStop}
      />

      {/* Contextual Telemetry & D-0012 Edge Compute Budget Ribbon */}
      <div className="bg-slate-900/90 border-b border-slate-800 px-6 py-2.5 flex flex-wrap items-center justify-between gap-4 text-xs">
        <div className="flex items-center gap-2">
          <span className="font-mono font-semibold text-cyan-400">
            SWARM DISPATCH:
          </span>
          <span className="text-slate-100 font-medium" data-testid="swarm-status-banner">
            {statusBannerText}
          </span>
        </div>

        {/* Unboxed Tabular Summary Metrics + D-0012 Edge Compute Budget (Zero-Pill Rule) */}
        <div className="flex flex-wrap items-center gap-2 font-mono text-slate-400 tabular-nums">
          <span className="text-emerald-300">
            {APP_CONFIG.edgeComputeBudget.summaryBadge}
          </span>
          <span aria-hidden="true">·</span>
          <span>
            Replanted:{' '}
            <strong className="text-emerald-400">
              {plantedCount}/{cropGapSites.length}
            </strong>
          </span>
          <span aria-hidden="true">·</span>
          <span>
            Seeds: <strong className="text-slate-200">{totalSeedsRemaining}</strong>
          </span>
          <span aria-hidden="true">·</span>
          <span>
            Manual:{' '}
            <strong className={humanControlledCount > 0 ? 'text-purple-400' : 'text-slate-200'}>
              {humanControlledCount}/5
            </strong>
          </span>
        </div>
      </div>

      {/* Main Content Viewport */}
      <main className="flex-1 max-w-[1600px] w-full mx-auto px-6 py-5 space-y-5">
        {activeTab === 'FIELD_OPS' && (
          <>
            <div className="grid grid-cols-1 lg:grid-cols-12 gap-5 items-start">
              {/* Left Column (3 cols): Multi-Robot Status Roster & Plan Summaries */}
              <div className="lg:col-span-3 space-y-5">
                <RobotRosterPanel
                  robots={robots}
                  selectedRobotId={selectedRobot.id}
                  onSelectRobot={setSelectedRobotId}
                  onToggleRobotHumanControl={handleToggleRobotHumanControl}
                />
              </div>

              {/* Center Stage (5 cols): Interactive Paddy Field Map with Continuous RBF Heatmap, Dots & Routes */}
              <div className="lg:col-span-5 space-y-5">
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

                {/* Compact Multi-Robot Plan Overview Strip directly under Map */}
                <section
                  aria-label="Active Fleet Plans Summary"
                  className="p-4 bg-slate-900 border border-slate-800 rounded-lg"
                >
                  <div className="flex items-center justify-between mb-2.5">
                    <h2 className="text-xs font-semibold text-slate-200">
                      Active Swarm Execution Plans (All 5 Homogeneous Medic-Ant Units)
                    </h2>
                    <button
                      type="button"
                      onClick={() => setActiveTab('ROBOT_PLANS')}
                      className="text-xs text-cyan-400 hover:underline font-medium cursor-pointer"
                    >
                      Open Full Plan Matrix →
                    </button>
                  </div>
                  <div className="space-y-1.5 text-xs">
                    {robots.map((r) => {
                      const theme = APP_CONFIG.statusTheme[r.status];
                      return (
                        <div
                          key={`mini-plan-${r.id}`}
                          onClick={() => setSelectedRobotId(r.id)}
                          className={`flex items-center justify-between gap-3 px-2.5 py-1.5 rounded cursor-pointer transition-colors ${
                            r.id === selectedRobot.id
                              ? 'bg-slate-800 text-slate-100'
                              : 'bg-slate-950/70 hover:bg-slate-800/50 text-slate-300'
                          }`}
                        >
                          <div className="flex items-center gap-2 min-w-0">
                            <span className={`font-mono font-semibold shrink-0 ${theme.textClass}`}>
                              {theme.glyph} {r.id}
                            </span>
                            <span className="truncate">{r.planSummary}</span>
                          </div>
                          <span className="font-mono text-[11px] text-slate-400 shrink-0 tabular-nums">
                            {r.taskMode}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                </section>
              </div>

              {/* Right Column (4 cols): Emergency Human Control Deck + 200Hz Sensor Fusion Telemetry */}
              <div className="lg:col-span-4 space-y-5">
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

                <TelemetryInspector
                  selectedRobot={selectedRobot}
                  allRobots={robots}
                  cropGapSites={cropGapSites}
                  onAssignTaskPlan={handleAssignTaskPlan}
                />
              </div>
            </div>

            {/* Bottom Interactive Bekker-Wong Terramechanics Lab & P2P Mycelial Gossip Inspector (`antenna.dashboard.v1`) */}
            <BekkerTerramechanicsLab
              gossipLogs={INITIAL_GOSSIP_LOGS}
              onExportDashboardJson={handleExportDashboardJson}
            />
          </>
        )}

        {activeTab === 'ROBOT_PLANS' && (
          <PlanTimelinePanel
            robots={robots}
            cropGapSites={cropGapSites}
            selectedRobotId={selectedRobot.id}
            onSelectRobot={(id) => {
              setSelectedRobotId(id);
            }}
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
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 items-start">
            <div className="lg:col-span-7">
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
            <div className="lg:col-span-5 space-y-5">
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
              <TelemetryInspector
                selectedRobot={selectedRobot}
                allRobots={robots}
                cropGapSites={cropGapSites}
                onAssignTaskPlan={handleAssignTaskPlan}
              />
            </div>
          </div>
        )}
      </main>

      {/* Quiet Functional Footer */}
      <footer className="mt-auto border-t border-slate-800/80 bg-slate-950 px-6 py-3 flex flex-wrap items-center justify-between gap-4 text-xs text-slate-400">
        <div>
          Antenna Multi-Robot Rice Farming System · Homogeneous Medic-Ant Swarm Console (`rice.v1` / `antenna.dashboard.v1`)
        </div>
        <div className="flex items-center gap-4">
          <button
            type="button"
            onClick={handleResetSwarm}
            className="hover:text-slate-200 transition-colors cursor-pointer"
          >
            Reset Terrace Baseline
          </button>
          <span aria-hidden="true">·</span>
          <button
            type="button"
            onClick={() => setActiveTab('FAULT_TRIAGE')}
            className="hover:text-slate-200 transition-colors cursor-pointer"
          >
            S1–S8 &amp; Negative Controls
          </button>
        </div>
      </footer>
    </div>
  );
}
