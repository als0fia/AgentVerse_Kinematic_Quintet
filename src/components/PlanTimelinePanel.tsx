/**
 * @file src/components/PlanTimelinePanel.tsx
 * @description Fleet-Wide Robot Plan Matrix & Step-by-Step Execution Queue for Antenna.
 *
 * This component provides a comprehensive tabular and timeline breakdown of every robot's current
 * plan, upcoming waypoints, expected physical postconditions, and homogeneous hardware capabilities (`make_homogeneous_rover_spec`):
 * - Clearly shows natural-language plans (e.g., "Robot 1 is going to retrieve Robot 3 via collinear rigid docking, cam-latch & pitch-lift").
 * - Lists ordered plan steps (`COMPLETED`, `IN_PROGRESS`, `QUEUED`) with exact metric coordinates.
 * - Provides quick one-click preset buttons to dispatch cooperative maneuvers across the quintet.
 */

import React from 'react';
import { APP_CONFIG } from '../config/appConfig';
import {
  CropGapSite,
  MetricCoordinate,
  RobotState,
  TaskMode,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface PlanTimelinePanelProps {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  onAssignTaskPlan: (
    robotId: string,
    taskMode: TaskMode,
    targetCoord: MetricCoordinate,
    options?: { targetPeerId?: string | null; targetSiteId?: string | null }
  ) => void;
}

/**
 * Renders the Fleet Plan Matrix and step-by-step execution queues for all 5 homogeneous Medic-Ant robots.
 */
export const PlanTimelinePanel: React.FC<PlanTimelinePanelProps> = ({
  robots,
  cropGapSites,
  selectedRobotId,
  onSelectRobot,
  onAssignTaskPlan,
}) => {
  return (
    <section
      aria-label="Multi-Robot Execution Plans and Capability Matrix"
      className="space-y-6"
    >
      {/* Top Summary Header */}
      <div className="flex flex-wrap items-center justify-between gap-4 p-4 bg-slate-900 border border-slate-800 rounded-lg">
        <div>
          <h2 className="text-base font-semibold text-slate-100">
            Homogeneous Medic-Ant Swarm Plans &amp; Task Allocation (TaskIntentV1)
          </h2>
          <p className="text-xs text-slate-400">
            Decentralized Mycelial P2P Gossip (`MycelialGossipPacket`) · Automatic Seed-Depletion Role Switch (`0` seeds → Scout)
          </p>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <button
            type="button"
            onClick={() => {
              logFunctionCall('PlanTimelinePanel.presetRetrieveR3WithR1', {});
              const r3 = robots.find((r) => r.id === 'ANT-03');
              if (r3) {
                onAssignTaskPlan('ANT-01', 'RETRIEVE_PEER', r3.position, {
                  targetPeerId: 'ANT-03',
                });
              }
            }}
            className="px-3 py-1.5 text-xs font-medium bg-cyan-950/70 hover:bg-cyan-900 border border-cyan-500/40 text-cyan-200 rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Plan: Robot 1 Rigid-Dock Rescue Robot 3
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('PlanTimelinePanel.presetPlantC04WithR2', {});
              const c04 = cropGapSites.find((s) => s.id === 'C-04');
              if (c04) {
                onAssignTaskPlan('ANT-02', 'PLANT', c04.position, {
                  targetSiteId: 'C-04',
                });
              }
            }}
            className="px-3 py-1.5 text-xs font-medium bg-emerald-950/70 hover:bg-emerald-900 border border-emerald-500/40 text-emerald-200 rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Plan: Robot 2 Plant Site C-04
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('PlanTimelinePanel.presetBeaconR3', {});
              const r3 = robots.find((r) => r.id === 'ANT-03');
              if (r3) {
                onAssignTaskPlan('ANT-03', 'BEACON_RELAY', r3.position);
              }
            }}
            className="px-3 py-1.5 text-xs font-medium bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-200 rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Plan: Convert Robot 3 to P2P Gossip Beacon
          </button>
        </div>
      </div>

      {/* High-Density Robot Plan Table */}
      <div className="bg-slate-900 border border-slate-800 rounded-lg overflow-hidden">
        <div className="overflow-x-auto">
          <table className="w-full text-left border-collapse text-xs">
            <thead>
              <tr className="bg-slate-950 border-b border-slate-800 text-slate-400 font-mono">
                <th className="py-3 px-4">ROBOT UNIT</th>
                <th className="py-3 px-4">STATUS COLOR</th>
                <th className="py-3 px-4">TASK MODE</th>
                <th className="py-3 px-4">ACTIVE NATURAL-LANGUAGE PLAN</th>
                <th className="py-3 px-4 text-right">POSE (X, Y)</th>
                <th className="py-3 px-4 text-right">WAYPOINTS</th>
                <th className="py-3 px-4 text-right">HOPPER</th>
                <th className="py-3 px-4 text-right">ACTION</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-slate-800">
              {robots.map((robot) => {
                const theme = APP_CONFIG.statusTheme[robot.status];
                const isSelected = robot.id === selectedRobotId;
                return (
                  <tr
                    key={robot.id}
                    className={`transition-colors ${
                      isSelected ? 'bg-slate-800/60' : 'hover:bg-slate-800/30'
                    }`}
                  >
                    <td className="py-3 px-4 font-semibold text-slate-100 whitespace-nowrap">
                      {robot.callsign}
                      <span className="block text-[11px] font-normal text-slate-400">
                        {robot.roleTitle}
                      </span>
                    </td>
                    <td className={`py-3 px-4 font-mono font-semibold whitespace-nowrap ${theme.textClass}`}>
                      {theme.glyph} {theme.shortLabel}
                    </td>
                    <td className="py-3 px-4 font-mono text-slate-200 whitespace-nowrap">
                      {robot.taskMode}
                    </td>
                    <td className="py-3 px-4 text-slate-100 font-medium max-w-md">
                      {robot.planSummary}
                    </td>
                    <td className="py-3 px-4 text-right font-mono text-slate-300 tabular-nums whitespace-nowrap">
                      ({robot.position.x.toFixed(1)}m, {robot.position.y.toFixed(1)}m)
                    </td>
                    <td className="py-3 px-4 text-right font-mono text-cyan-300 tabular-nums whitespace-nowrap">
                      {robot.plannedRoute.length} pts → ({robot.targetWaypoint.x.toFixed(1)}m,{' '}
                      {robot.targetWaypoint.y.toFixed(1)}m)
                    </td>
                    <td className="py-3 px-4 text-right font-mono text-slate-300 tabular-nums whitespace-nowrap">
                      {robot.seedHopperCount}/{robot.capabilities.maxHopperCapacity}
                    </td>
                    <td className="py-3 px-4 text-right whitespace-nowrap">
                      <button
                        type="button"
                        onClick={() => onSelectRobot(robot.id)}
                        className="px-2.5 py-1 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-cyan-300 cursor-pointer"
                      >
                        Inspect Steps
                      </button>
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      </div>

      {/* Step-by-Step Plan Cards for Each Robot */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {robots.map((robot) => {
          const theme = APP_CONFIG.statusTheme[robot.status];
          return (
            <div
              key={`steps-card-${robot.id}`}
              className="p-4 bg-slate-900 border border-slate-800 rounded-lg flex flex-col justify-between"
            >
              <div>
                <div className="flex items-center justify-between gap-2 pb-2.5 mb-3 border-b border-slate-800">
                  <h3 className="text-sm font-semibold text-slate-100">
                    {robot.callsign}
                  </h3>
                  <span className={`text-xs font-mono font-semibold ${theme.textClass}`}>
                    {theme.glyph} {theme.shortLabel}
                  </span>
                </div>

                <p className="text-xs text-slate-200 font-medium mb-3">
                  {robot.planSummary}
                </p>

                <ol className="space-y-2.5 text-xs">
                  {robot.planSteps.map((step) => (
                    <li
                      key={step.id}
                      className="p-2.5 bg-slate-950 border border-slate-800/90 rounded"
                    >
                      <div className="flex items-center justify-between font-mono text-[11px] text-slate-400 mb-1">
                        <span>STEP 0{step.order}</span>
                        <span
                          className={
                            step.status === 'COMPLETED'
                              ? 'text-emerald-400'
                              : step.status === 'IN_PROGRESS'
                                ? 'text-cyan-300 font-semibold'
                                : 'text-slate-400'
                          }
                        >
                          {step.status}
                        </span>
                      </div>
                      <div className="text-slate-200 font-medium">{step.title}</div>
                      <div className="text-[11px] font-mono text-slate-400 mt-1">
                        Postcondition: {step.expectedPostcondition}
                      </div>
                    </li>
                  ))}
                </ol>
              </div>

              {/* Hardware Capabilities Footer (Unboxed Metadata) */}
              <div className="mt-4 pt-2.5 border-t border-slate-800 text-[11px] font-mono text-slate-400 flex flex-wrap items-center gap-1.5">
                <span>Homogeneous Spec:</span>
                <span>{robot.seedHopperCount > 0 ? 'Planter Active' : 'Depleted→Scout'}</span>
                <span aria-hidden="true">·</span>
                <span>{robot.capabilities.hasRigidDockMechanism ? 'Rigid Dock + Cam-Latch' : 'None'}</span>
                <span aria-hidden="true">·</span>
                <span>3-Ray LiDAR + Probe</span>
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
};
