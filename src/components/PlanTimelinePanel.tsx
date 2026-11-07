/**
 * @file src/components/PlanTimelinePanel.tsx
 * @description Clean Fleet Mission Plans & Selected Robot Step Queue.
 *
 * Shows a single concise table of all 5 robots' current plans on the left,
 * and the step-by-step execution queue for the selected robot on the right.
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
 * Renders the Fleet Plan table and the selected robot's step-by-step timeline.
 */
export const PlanTimelinePanel: React.FC<PlanTimelinePanelProps> = ({
  robots,
  cropGapSites,
  selectedRobotId,
  onSelectRobot,
  onAssignTaskPlan,
}) => {
  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];
  const selectedTheme = APP_CONFIG.statusTheme[selectedRobot.status];

  return (
    <section
      aria-label="Multi-Robot Execution Plans"
      className="space-y-4"
    >
      {/* Clean Header & Quick Team Presets */}
      <div className="flex flex-wrap items-center justify-between gap-4 p-4 bg-slate-900 border border-slate-800 rounded-lg">
        <div>
          <h2 className="text-base font-semibold text-slate-100">
            Robot Mission Plans
          </h2>
          <p className="text-xs text-slate-400">
            Click any robot in the table to view its step-by-step schedule.
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
            Preset: Robot 1 Retrieve Robot 3
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
            Preset: Robot 2 Plant Site C-04
          </button>
        </div>
      </div>

      {/* Split Layout: Left = Fleet Plan Table (8 cols), Right = Selected Robot Steps (4 cols) */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 items-start">
        <div className="lg:col-span-8 bg-slate-900 border border-slate-800 rounded-lg overflow-hidden">
          <div className="overflow-x-auto">
            <table className="w-full text-left border-collapse text-xs">
              <thead>
                <tr className="bg-slate-950 border-b border-slate-800 text-slate-400 font-mono">
                  <th className="py-3 px-4">ROBOT</th>
                  <th className="py-3 px-4">STATUS</th>
                  <th className="py-3 px-4">CURRENT PLAN</th>
                  <th className="py-3 px-4 text-right">TARGET</th>
                  <th className="py-3 px-4 text-right">SEEDS</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-slate-800">
                {robots.map((robot) => {
                  const theme = APP_CONFIG.statusTheme[robot.status];
                  const isSelected = robot.id === selectedRobotId;
                  return (
                    <tr
                      key={robot.id}
                      onClick={() => onSelectRobot(robot.id)}
                      className={`transition-colors cursor-pointer ${
                        isSelected ? 'bg-slate-800/70' : 'hover:bg-slate-800/30'
                      }`}
                    >
                      <td className="py-3 px-4 font-semibold text-slate-100 whitespace-nowrap">
                        {robot.callsign.split('(')[0].trim()}
                      </td>
                      <td className={`py-3 px-4 font-mono font-semibold whitespace-nowrap ${theme.textClass}`}>
                        {theme.glyph} {theme.shortLabel}
                      </td>
                      <td className="py-3 px-4 text-slate-200 max-w-md">
                        {robot.planSummary}
                      </td>
                      <td className="py-3 px-4 text-right font-mono text-cyan-300 tabular-nums whitespace-nowrap">
                        ({robot.targetWaypoint.x.toFixed(1)}m, {robot.targetWaypoint.y.toFixed(1)}m)
                      </td>
                      <td className="py-3 px-4 text-right font-mono text-slate-300 tabular-nums whitespace-nowrap">
                        {robot.seedHopperCount}/{robot.capabilities.maxHopperCapacity}
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        </div>

        {/* Selected Robot Step-by-Step Queue */}
        <div className="lg:col-span-4 p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-3">
          <div className="flex items-center justify-between gap-2 pb-2.5 border-b border-slate-800">
            <h3 className="text-sm font-semibold text-slate-100">
              {selectedRobot.callsign.split('(')[0].trim()} · Steps
            </h3>
            <span className={`text-xs font-mono font-semibold ${selectedTheme.textClass}`}>
              {selectedTheme.glyph} {selectedTheme.shortLabel}
            </span>
          </div>

          <p className="text-xs text-slate-300">
            {selectedRobot.planSummary}
          </p>

          <ol className="space-y-2 text-xs">
            {selectedRobot.planSteps.map((step) => (
              <li
                key={step.id}
                className="p-2.5 bg-slate-950 border border-slate-800 rounded"
              >
                <div className="flex items-center justify-between font-mono text-[11px] text-slate-400 mb-1">
                  <span>STEP {step.order}</span>
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
                <div className="text-slate-100 font-medium">{step.title}</div>
              </li>
            ))}
          </ol>
        </div>
      </div>
    </section>
  );
};
