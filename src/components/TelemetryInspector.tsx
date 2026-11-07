/**
 * @file src/components/TelemetryInspector.tsx
 * @description Clean Selected-Robot Plan Dispatcher & Health Summary Card.
 *
 * Shows the selected robot's current plan, a simple task assignment form, and
 * 3 essential health numbers (Speed, Wheel Slip, Seeds), with an optional
 * expandable drawer for detailed onboard sensor readings.
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
 * Renders the clean robot task assignment and health summary panel.
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
  const [showDetailedSensors, setShowDetailedSensors] = useState<boolean>(false);

  const { telemetry } = selectedRobot;
  const shortName = selectedRobot.callsign.split('·')[0].trim();

  /**
   * Handles submitting the robot task assignment form.
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

    onAssignTaskPlan(
      selectedRobot.id,
      'NAVIGATE',
      APP_CONFIG.fieldGeometry.depotPosition
    );
  }

  return (
    <section
      aria-label="Selected Robot Plan and Health"
      className="p-4 space-y-4"
    >
      {/* Active Plan Box */}
      <div className="p-3 rounded-md bg-slate-950 border border-slate-800">
        <div className="text-[11px] font-mono uppercase text-slate-400 mb-1">
          Current Plan ({selectedRobot.taskMode})
        </div>
        <p className="text-xs text-slate-100 font-medium leading-relaxed">
          {selectedRobot.planSummary}
        </p>
      </div>

      {/* Fault Alert (only shown when there is an active issue) */}
      {selectedRobot.faultReason && (
        <div
          className={`p-3 rounded-md border text-xs ${
            selectedRobot.faultScenario.startsWith('NEG_')
              ? 'bg-emerald-950/40 border-emerald-500/40 text-emerald-200'
              : 'bg-rose-950/50 border-rose-500/40 text-rose-200'
          }`}
        >
          <div className="font-semibold mb-0.5">
            {selectedRobot.faultScenario.startsWith('NEG_')
              ? 'Sensor Notice'
              : 'Alert: Robot Stuck'}
          </div>
          <p>{selectedRobot.faultReason}</p>
        </div>
      )}

      {/* 3 Essential Health Metrics */}
      <div className="grid grid-cols-3 gap-2">
        <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
          <div className="text-[11px] text-slate-400">Speed</div>
          <div className="mt-0.5 font-mono tabular-nums">
            <span className="text-base font-bold text-slate-100">
              {telemetry.trueVelocityMs.toFixed(2)}
            </span>
            <span className="text-xs text-slate-400 ml-1">m/s</span>
          </div>
        </div>

        <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
          <div className="text-[11px] text-slate-400">Wheel Slip</div>
          <div className="mt-0.5 font-mono tabular-nums">
            <span
              className={`text-base font-bold ${
                telemetry.slipRatio >= 0.6
                  ? 'text-rose-400'
                  : telemetry.slipRatio >= 0.42
                    ? 'text-amber-400'
                    : 'text-emerald-400'
              }`}
            >
              {(telemetry.slipRatio * 100).toFixed(0)}%
            </span>
          </div>
        </div>

        <div className="p-2.5 bg-slate-950 border border-slate-800 rounded">
          <div className="text-[11px] text-slate-400">Seeds Left</div>
          <div className="mt-0.5 font-mono tabular-nums">
            <span className="text-base font-bold text-slate-100">
              {selectedRobot.seedHopperCount}
            </span>
            <span className="text-xs text-slate-400 ml-0.5">
              /{selectedRobot.capabilities.maxHopperCapacity}
            </span>
          </div>
        </div>
      </div>

      {/* Simple Plan Reassignment Form */}
      <form
        onSubmit={handlePlanFormSubmit}
        className="p-3.5 bg-slate-950 border border-slate-800 rounded-md space-y-3"
      >
        <h3 className="text-xs font-semibold text-slate-200">
          Assign New Plan to {shortName}
        </h3>

        <div className="space-y-2.5">
          <div>
            <label
              htmlFor="plan-mode-select"
              className="block text-xs text-slate-400 mb-1"
            >
              Task
            </label>
            <select
              id="plan-mode-select"
              value={selectedTaskMode}
              onChange={(e) => setSelectedTaskMode(e.target.value as TaskMode)}
              className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400 cursor-pointer"
            >
              <option value="RETRIEVE_PEER">Retrieve Stuck Peer Robot</option>
              <option value="PLANT">Plant Missing Crop Site</option>
              <option value="SAMPLE">Scout Soil Moisture &amp; Firmness</option>
              <option value="NAVIGATE">Return to Seed Depot</option>
              <option value="BEACON_RELAY">Act as Comms Relay Beacon</option>
              <option value="HOLD">Hold Position</option>
            </select>
          </div>

          {selectedTaskMode === 'RETRIEVE_PEER' && (
            <div>
              <label
                htmlFor="peer-target-select"
                className="block text-xs text-slate-400 mb-1"
              >
                Robot to Retrieve
              </label>
              <select
                id="peer-target-select"
                value={selectedPeerId}
                onChange={(e) => setSelectedPeerId(e.target.value)}
                className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400 cursor-pointer"
              >
                {allRobots
                  .filter((r) => r.id !== selectedRobot.id)
                  .map((r) => (
                    <option key={r.id} value={r.id}>
                      {r.callsign.split('(')[0].trim()} ({r.status})
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
                Target Crop Site
              </label>
              <select
                id="site-target-select"
                value={selectedSiteId}
                onChange={(e) => setSelectedSiteId(e.target.value)}
                className="w-full px-2.5 py-1.5 text-xs bg-slate-900 border border-slate-700 rounded text-slate-100 focus:outline-cyan-400 cursor-pointer"
              >
                {cropGapSites.map((site) => (
                  <option key={site.id} value={site.id}>
                    {site.id} ({site.position.x}m, {site.position.y}m)
                  </option>
                ))}
              </select>
            </div>
          )}
        </div>

        <button
          type="submit"
          className="w-full py-2 px-4 bg-cyan-600 hover:bg-cyan-500 text-white text-xs font-semibold rounded transition-colors cursor-pointer"
        >
          Update {shortName} Plan
        </button>
      </form>

      {/* Progressive Disclosure: Optional Detailed Sensor Drawer */}
      <div className="pt-1 border-t border-slate-800/80">
        <button
          type="button"
          onClick={() => setShowDetailedSensors((v) => !v)}
          className="text-xs text-slate-400 hover:text-slate-200 font-medium cursor-pointer"
        >
          {showDetailedSensors ? 'Hide Advanced Sensors ▲' : 'Show Advanced Sensors ▼'}
        </button>

        {showDetailedSensors && (
          <div className="mt-2.5 grid grid-cols-2 gap-2 text-xs font-mono bg-slate-950 p-2.5 rounded border border-slate-800 tabular-nums">
            <div>
              <span className="text-slate-400 block text-[10px]">Mud Sinkage</span>
              <span className="text-slate-200">{telemetry.wheelSinkageMm.toFixed(0)} mm</span>
            </div>
            <div>
              <span className="text-slate-400 block text-[10px]">Belly Clearance</span>
              <span className="text-slate-200">{telemetry.bellyClearanceMm.toFixed(0)} mm</span>
            </div>
            <div>
              <span className="text-slate-400 block text-[10px]">Front LiDAR</span>
              <span className="text-slate-200">
                {telemetry.frontLidarRangeM === null
                  ? 'Dropout'
                  : `${telemetry.frontLidarRangeM.toFixed(2)} m`}
              </span>
            </div>
            <div>
              <span className="text-slate-400 block text-[10px]">Docking Boom</span>
              <span className="text-cyan-300">
                {telemetry.rigidDock.isRigidLatched ? 'Latched' : 'Open'} (
                {telemetry.rigidDock.dockPitchRad.toFixed(2)} rad)
              </span>
            </div>
          </div>
        )}
      </div>
    </section>
  );
};
