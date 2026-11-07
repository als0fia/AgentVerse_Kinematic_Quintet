/**
 * @file src/components/RobotRosterPanel.tsx
 * @description Fleet Status Roster & Plan Summary Column for the Antenna Multi-Robot System.
 *
 * This component displays all homogeneous Medic-Ant rovers in the Kinematic Quintet (`ANT-01` through `ANT-05`) with:
 * - Color-coded status indicators paired with accessible glyphs (`● Nominal`, `◆ Retrieving`,
 *   `▲ Slip Alert`, `✖ Immobilized`, `■ Human Control`).
 * - Explicit natural-language plan statements showing what each robot is doing right now
 *   (e.g., "Robot 1 is going to retrieve Robot 3 via collinear rigid docking, cam-latch & pitch-lift at (14.4m, 8.4m)").
 * - Key tabular metrics (metric position, slip ratio %, wheel sinkage mm, seed hopper count).
 * - Direct one-click button on each robot row to engage or release Emergency Human Control Mode.
 */

import React, { useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import { RobotState } from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface RobotRosterPanelProps {
  robots: RobotState[];
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  onToggleRobotHumanControl: (robotId: string, enable: boolean) => void;
}

/**
 * Renders the multi-robot fleet roster with color-coded statuses, plan descriptions, and emergency human control triggers.
 */
export const RobotRosterPanel: React.FC<RobotRosterPanelProps> = ({
  robots,
  selectedRobotId,
  onSelectRobot,
  onToggleRobotHumanControl,
}) => {
  const [statusFilter, setStatusFilter] = useState<'ALL' | 'ACTIVE' | 'ATTENTION'>('ALL');

  /**
   * Filters the robot list according to the selected segmented filter button.
   */
  function getFilteredRobots(): RobotState[] {
    if (statusFilter === 'ACTIVE') {
      return robots.filter((r) => r.status === 'NOMINAL' || r.status === 'RETRIEVING');
    }
    if (statusFilter === 'ATTENTION') {
      return robots.filter(
        (r) =>
          r.status === 'IMMOBILIZED' ||
          r.status === 'WARNING_SLIP' ||
          r.status === 'HUMAN_CONTROL'
      );
    }
    return robots;
  }

  const filteredRobots = getFilteredRobots();

  return (
    <section
      aria-label="Multi-Robot Fleet Status and Plan Roster"
      className="flex flex-col bg-slate-900 border border-slate-800 rounded-lg overflow-hidden"
    >
      {/* Header & Segmented Status Filter */}
      <div className="px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div className="flex items-center justify-between gap-2 mb-2.5">
          <h2 className="text-sm font-semibold text-slate-100">
            Homogeneous Medic-Ant Roster ({robots.length} Units)
          </h2>
          <span className="text-xs font-mono text-slate-400 tabular-nums">
            276mm Row Fit
          </span>
        </div>

        {/* Interactive Filter Tabs */}
        <div className="flex items-center gap-1 p-1 bg-slate-900 border border-slate-800 rounded-md">
          <button
            type="button"
            onClick={() => {
              logFunctionCall('RobotRosterPanel.setStatusFilter', { filter: 'ALL' });
              setStatusFilter('ALL');
            }}
            className={`flex-1 px-2.5 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              statusFilter === 'ALL'
                ? 'bg-slate-800 text-slate-100'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            All ({robots.length})
          </button>
          <button
            type="button"
            onClick={() => {
              logFunctionCall('RobotRosterPanel.setStatusFilter', { filter: 'ACTIVE' });
              setStatusFilter('ACTIVE');
            }}
            className={`flex-1 px-2.5 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              statusFilter === 'ACTIVE'
                ? 'bg-slate-800 text-emerald-300'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Nominal / Dock
          </button>
          <button
            type="button"
            onClick={() => {
              logFunctionCall('RobotRosterPanel.setStatusFilter', { filter: 'ATTENTION' });
              setStatusFilter('ATTENTION');
            }}
            className={`flex-1 px-2.5 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              statusFilter === 'ATTENTION'
                ? 'bg-slate-800 text-rose-300'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Alerts / Manual
          </button>
        </div>
      </div>

      {/* Robot Cards List */}
      <div className="divide-y divide-slate-800">
        {filteredRobots.map((robot) => {
          const theme = APP_CONFIG.statusTheme[robot.status];
          const isSelected = robot.id === selectedRobotId;

          return (
            <div
              key={robot.id}
              className={`p-3.5 transition-colors ${
                isSelected ? 'bg-slate-800/75' : 'hover:bg-slate-800/35'
              }`}
            >
              {/* Top Row: Robot Callsign + Color-Coded Status Label */}
              <div className="flex items-center justify-between gap-2 mb-1.5">
                <button
                  type="button"
                  onClick={() => {
                    logFunctionCall('RobotRosterPanel.onSelectRobot', { robotId: robot.id });
                    onSelectRobot(robot.id);
                  }}
                  className="text-left group focus-visible:outline-2 focus-visible:outline-cyan-400 rounded cursor-pointer"
                >
                  <span className="text-sm font-semibold text-slate-100 group-hover:text-cyan-300 transition-colors">
                    {robot.callsign}
                  </span>
                  <span className="block text-xs text-slate-400">
                    {robot.roleTitle}
                  </span>
                </button>

                {/* Non-Hue-Only Status Text + Symbol */}
                <span className={`text-xs font-mono font-semibold whitespace-nowrap shrink-0 ${theme.textClass}`}>
                  {theme.glyph} {theme.shortLabel}
                </span>
              </div>

              {/* Explicit Natural-Language Robot Plan Box */}
              <div className="my-2 px-2.5 py-2 rounded bg-slate-950/90 border border-slate-800/90">
                <div className="text-[11px] font-mono text-slate-400 mb-0.5 flex items-center justify-between gap-2">
                  <span>PLAN · {robot.taskMode}</span>
                  <span className="shrink-0 tabular-nums">z={robot.telemetry.wheelSinkageMm.toFixed(0)}mm</span>
                </div>
                <p className="text-xs text-slate-200 leading-snug font-medium">
                  {robot.planSummary}
                </p>
              </div>

              {/* Unboxed Metadata Row with Typographic Separators (Zero-Pill Rule) */}
              <div className="flex flex-wrap items-center justify-between gap-2 text-xs font-mono text-slate-400 tabular-nums mt-2">
                <div className="flex items-center gap-1.5">
                  <span>
                    ({robot.position.x.toFixed(1)}m, {robot.position.y.toFixed(1)}m)
                  </span>
                  <span aria-hidden="true">·</span>
                  <span
                    className={
                      robot.telemetry.slipRatio >= 0.6
                        ? 'text-rose-400 font-semibold'
                        : robot.telemetry.slipRatio >= 0.42
                          ? 'text-amber-400'
                          : 'text-slate-300'
                    }
                  >
                    Slip {(robot.telemetry.slipRatio * 100).toFixed(0)}%
                  </span>
                  <span aria-hidden="true">·</span>
                  <span>
                    Seeds {robot.seedHopperCount}
                    {robot.seedHopperCount === 0 ? ' (Scout)' : ''}
                  </span>
                </div>

                {/* Quick Emergency Human Override Button */}
                <button
                  type="button"
                  onClick={() => {
                    logFunctionCall('RobotRosterPanel.onToggleRobotHumanControl', {
                      robotId: robot.id,
                      enable: !robot.humanControlActive,
                    });
                    onSelectRobot(robot.id);
                    onToggleRobotHumanControl(robot.id, !robot.humanControlActive);
                  }}
                  className={`px-2.5 py-1 text-xs font-medium rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                    robot.humanControlActive
                      ? 'bg-purple-600 border-purple-400 text-white hover:bg-purple-500'
                      : 'bg-slate-950 border-slate-700 text-slate-200 hover:border-purple-400 hover:text-purple-300'
                  }`}
                >
                  {robot.humanControlActive ? 'Exit Human Control' : 'Human Control'}
                </button>
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
};
