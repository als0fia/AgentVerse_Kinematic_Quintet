/**
 * @file src/components/TelemetryInspector.tsx
 * @description 200Hz STM32H7 Sensor-Fusion Telemetry Inspector & Plan Reassignment Panel.
 *
 * This component displays the real-time low-level sensor telemetry for the currently selected robot
 * and provides an interactive `TaskIntentV1` Plan Reassignment form:
 * - 60GHz Doppler mmWave Radar true ground speed ($v_{\text{true}}$) vs Wheel Encoder speed ($\omega \cdot r$).
 * - Real-time Kinematic Slip Ratio $s = 1 - (v_{\text{true}} / (\omega \cdot r))$ with anti-digging threshold indicator.
 * - TDK ICM-42688-P IMU Pitch/Roll angles ($\theta, \phi$) and INA240 Motor Phase Current ($I$) + Tow Hitch Tension ($F_{\text{pull}}$).
 * - Hall-Effect Whisker + VL53L8CX ToF lateral furrow cross-track error ($e_y$).
 * - Interactive Plan Dispatcher: Lets operators assign a new plan to the selected robot (e.g.,
 *   "Retrieve Peer Robot", "Plant Missing Crop Gap", "Sample Soil Moisture", "UWB Relay Beacon").
 *
 * Use Cases:
 * 1. Diagnosing why a robot is slipping or immobilized using physical sensor fusion metrics.
 * 2. Reassigning a robot's mission plan and immediately seeing its trajectory and plan summary update.
 */

import React, { useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import {
  CropGapSite,
  MetricCoordinate,
  RobotState,
  TaskMode,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface TelemetryInspectorProps {
  selectedRobot: RobotState;
  allRobots: RobotState[];
  cropGapSites: CropGapSite[];
  onAssignTaskPlan: (
    robotId: string,
    taskMode: TaskMode,
    targetCoord: MetricCoordinate,
    options?: { targetPeerId?: string | null; targetSiteId?: string | null }
  ) => void;
}

/**
 * Renders the 200Hz sensor fusion telemetry grid and interactive plan assignment form for the selected robot.
 * @param props - TelemetryInspectorProps state and task assignment callback.
 * @returns React JSX section element.
 */
export const TelemetryInspector: React.FC<TelemetryInspectorProps> = ({
  selectedRobot,
  allRobots,
  cropGapSites,
  onAssignTaskPlan,
}) => {
  const [selectedTaskMode, setSelectedTaskMode] = useState<TaskMode>('RETRIEVE_PEER');
  const [selectedPeerId, setSelectedPeerId] = useState<string>('ANT-03');
  const [selectedSiteId, setSelectedSiteId] = useState<string>('C-04');

  const theme = APP_CONFIG.statusTheme[selectedRobot.status];
  const { telemetry } = selectedRobot;

  /**
   * Handles submitting the new robot plan assignment form.
   * @param event - Form submit event.
   */
  function handlePlanFormSubmit(event: React.FormEvent): void {
    event.preventDefault();
    logFunctionCall('TelemetryInspector.handlePlanFormSubmit', {
      robotId: selectedRobot.id,
      selectedTaskMode,
      selectedPeerId,
      selectedSiteId,
    });

    if (selectedTaskMode === 'RETRIEVE_PEER') {
      const peer = allRobots.find((r) => r.id === selectedPeerId) ?? allRobots[0];
      onAssignTaskPlan(selectedRobot.id, 'RETRIEVE_PEER', peer.position, {
        targetPeerId: peer.id,
      });
      return;
    }

    if (selectedTaskMode === 'PLANT' || selectedTaskMode === 'SAMPLE') {
      const site = cropGapSites.find((s) => s.id === selectedSiteId) ?? cropGapSites[0];
      onAssignTaskPlan(selectedRobot.id, selectedTaskMode, site.position, {
        targetSiteId: site.id,
      });
      return;
    }

    if (selectedTaskMode === 'BEACON_RELAY' || selectedTaskMode === 'HOLD') {
      onAssignTaskPlan(selectedRobot.id, selectedTaskMode, selectedRobot.position);
      return;
    }

    // Default NAVIGATE to depot
    onAssignTaskPlan(
      selectedRobot.id,
      'NAVIGATE',
      APP_CONFIG.fieldGeometry.depotPosition
    );
  }

  return (
    <section
      aria-label="200Hz Sensor Fusion Telemetry and Plan Dispatcher"
      className="flex flex-col bg-slate-900 border border-slate-800 rounded-lg overflow-hidden"
    >
      {/* Header */}
      <div className="flex items-center justify-between gap-2 px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div>
          <h2 className="text-sm font-semibold text-slate-100">
            {selectedRobot.callsign} · Telemetry & Plan
          </h2>
          <p className="text-xs text-slate-400">
            STM32H7 200Hz Loop · Battery {selectedRobot.batteryPct}% · Hopper {selectedRobot.seedHopperCount}/{selectedRobot.capabilities.maxHopperCapacity} seeds
          </p>
        </div>
        <span className={`text-xs font-mono font-semibold whitespace-nowrap shrink-0 ${theme.textClass}`}>
          {theme.glyph} {theme.shortLabel}
        </span>
      </div>

      <div className="p-4 space-y-4">
        {/* Active Fault Diagnostic Banner (if immobilized or slipping) */}
        {selectedRobot.faultReason && (
          <div className="p-3 rounded bg-rose-950/50 border border-rose-500/40 text-xs text-rose-200">
            <div className="font-mono font-semibold text-rose-300 mb-0.5">
              DIAGNOSTIC ALERT ({selectedRobot.faultScenario})
            </div>
            <p>{selectedRobot.faultReason}</p>
          </div>
        )}

        {/* 200Hz STM32H7 Sensor-Fusion Metric Grid (Tabular Numerals + Unit Hierarchy) */}
        <div className="grid grid-cols-2 sm:grid-cols-3 gap-2.5">
          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">RADAR v_true</div>
            <div className="mt-1 font-mono tabular-nums">
              <span className="text-lg font-bold text-slate-100">
                {telemetry.trueVelocityMs.toFixed(2)}
              </span>
              <span className="text-xs text-slate-400 ml-1">m/s</span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              Wheel: {telemetry.wheelSpeedMs.toFixed(2)} m/s
            </div>
          </div>

          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">SLIP RATIO (s)</div>
            <div className="mt-1 font-mono tabular-nums">
              <span
                className={`text-lg font-bold ${
                  telemetry.slipRatio >= 0.6
                    ? 'text-rose-400'
                    : telemetry.slipRatio >= 0.42
                    ? 'text-amber-400'
                    : 'text-emerald-400'
                }`}
              >
                {(telemetry.slipRatio * 100).toFixed(0)}
              </span>
              <span className="text-xs text-slate-400 ml-1">%</span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              Gate: ≤ 60%
            </div>
          </div>

          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">MOTOR CURRENT</div>
            <div className="mt-1 font-mono tabular-nums">
              <span className="text-lg font-bold text-slate-100">
                {telemetry.motorCurrentAmps.toFixed(1)}
              </span>
              <span className="text-xs text-slate-400 ml-1">A</span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              Tether: {telemetry.tetherTensionN} N
            </div>
          </div>

          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">IMU ATTITUDE</div>
            <div className="mt-1 font-mono tabular-nums">
              <span className="text-base font-bold text-slate-100">
                {telemetry.pitchDeg.toFixed(1)}° / {telemetry.rollDeg.toFixed(1)}°
              </span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              Pitch / Roll (±18° limit)
            </div>
          </div>

          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">FURROW ERROR e_y</div>
            <div className="mt-1 font-mono tabular-nums">
              <span className="text-lg font-bold text-slate-100">
                {telemetry.furrowErrorCm.toFixed(1)}
              </span>
              <span className="text-xs text-slate-400 ml-1">cm</span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              Whisker + 8×8 ToF
            </div>
          </div>

          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
            <div className="text-[11px] font-mono text-slate-400">DUAL-UWB PDoA</div>
            <div className="mt-1 font-mono tabular-nums">
              <span className="text-lg font-bold text-cyan-300">
                {telemetry.uwbLinkQualityPct}
              </span>
              <span className="text-xs text-slate-400 ml-1">%</span>
            </div>
            <div className="text-[11px] font-mono text-slate-400 mt-0.5">
              HaLow Sub-1GHz OK
            </div>
          </div>
        </div>

        {/* Interactive Plan Reassignment Form (TaskIntentV1) */}
        <form
          onSubmit={handlePlanFormSubmit}
          className="p-3.5 bg-slate-950 border border-slate-800 rounded-md space-y-3"
        >
          <div className="flex items-center justify-between">
            <h3 className="text-xs font-semibold text-slate-200">
              Assign New Robot Plan (TaskIntentV1)
            </h3>
            <span className="text-[11px] font-mono text-slate-400">
              Target: {selectedRobot.id}
            </span>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 gap-2.5">
            <div>
              <label
                htmlFor="plan-mode-select"
                className="block text-xs text-slate-400 mb-1"
              >
                Task Mode
              </label>
              <select
                id="plan-mode-select"
                value={selectedTaskMode}
                onChange={(e) => setSelectedTaskMode(e.target.value as TaskMode)}
                className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400"
              >
                <option value="RETRIEVE_PEER">Retrieve Stuck Peer (Winch Tow)</option>
                <option value="PLANT">Plant Missing Crop Site</option>
                <option value="SAMPLE">Sample Soil Moisture & Firmness</option>
                <option value="NAVIGATE">Return to Seed Depot (2.0m, 2.0m)</option>
                <option value="BEACON_RELAY">Stationary UWB Relay Beacon</option>
                <option value="HOLD">Deterministic Safety HOLD</option>
              </select>
            </div>

            {selectedTaskMode === 'RETRIEVE_PEER' && (
              <div>
                <label
                  htmlFor="peer-target-select"
                  className="block text-xs text-slate-400 mb-1"
                >
                  Peer Robot to Retrieve
                </label>
                <select
                  id="peer-target-select"
                  value={selectedPeerId}
                  onChange={(e) => setSelectedPeerId(e.target.value)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400"
                >
                  {allRobots
                    .filter((r) => r.id !== selectedRobot.id)
                    .map((r) => (
                      <option key={r.id} value={r.id}>
                        {r.callsign} ({r.status})
                      </option>
                    ))}
                </select>
              </div>
            )}

            {(selectedTaskMode === 'PLANT' || selectedTaskMode === 'SAMPLE') && (
              <div>
                <label
                  htmlFor="site-target-select"
                  className="block text-xs text-slate-400 mb-1"
                >
                  Target Crop Gap Site
                </label>
                <select
                  id="site-target-select"
                  value={selectedSiteId}
                  onChange={(e) => setSelectedSiteId(e.target.value)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400"
                >
                  {cropGapSites.map((site) => (
                    <option key={site.id} value={site.id}>
                      {site.id} ({site.position.x}m, {site.position.y}m) · {site.status}
                    </option>
                  ))}
                </select>
              </div>
            )}
          </div>

          <button
            type="submit"
            className="w-full py-2 px-4 bg-cyan-600 hover:bg-cyan-500 text-white text-xs font-semibold rounded transition-colors"
          >
            Update {selectedRobot.callsign.split('·')[0].trim()} Plan
          </button>
        </form>
      </div>
    </section>
  );
};
