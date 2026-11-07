/**
 * @file src/components/RobotRosterPanel.tsx
 * @description Clean, Scannable Fleet Roster Sidebar for the Antenna Dashboard.
 *
 * Displays the 5 robots (`ANT-01` through `ANT-05`) with only the essential operator information:
 * - Robot name & callsign (`Robot 1 · Atlas`).
 * - Color-coded status indicator (`● Nominal`, `◆ Retrieving`, `▲ Alert`, `✖ Stuck`, `■ Manual`).
 * - Concise, plain-English current plan (e.g., "Robot 1 is going to retrieve Robot 3").
 * - Quick one-click "Human Control" override button.
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
 * Renders the streamlined multi-robot fleet roster sidebar.
 */
export const RobotRosterPanel: React.FC<RobotRosterPanelProps> = ({
  robots,
  selectedRobotId,
  onSelectRobot,
  onToggleRobotHumanControl,
}) => {
  const [statusFilter, setStatusFilter] = useState<'ALL' | 'ACTIVE' | 'ATTENTION'>('ALL');

  /**
   * Filters the robot list according to the selected filter tab.
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
      {/* Header & Compact Filter */}
      <div className="px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div className="flex items-center justify-between gap-2 mb-2">
          <h2 className="text-sm font-semibold text-slate-100">
            Robots ({robots.length})
          </h2>
          <span className="text-xs text-slate-400">
            Select a unit to control
          </span>
        </div>

        <div className="flex items-center gap-1 p-1 bg-slate-900 border border-slate-800 rounded-md">
          <button
            type="button"
            onClick={() => {
              logFunctionCall('RobotRosterPanel.setStatusFilter', { filter: 'ALL' });
              setStatusFilter('ALL');
            }}
            className={`flex-1 px-2 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
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
            className={`flex-1 px-2 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              statusFilter === 'ACTIVE'
                ? 'bg-slate-800 text-emerald-300'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Active
          </button>
          <button
            type="button"
            onClick={() => {
              logFunctionCall('RobotRosterPanel.setStatusFilter', { filter: 'ATTENTION' });
              setStatusFilter('ATTENTION');
            }}
            className={`flex-1 px-2 py-1 text-xs font-medium rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              statusFilter === 'ATTENTION'
                ? 'bg-slate-800 text-rose-300'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            Alerts
          </button>
        </div>
      </div>

      {/* Streamlined Robot Cards */}
      <div className="divide-y divide-slate-800/80">
        {filteredRobots.map((robot) => {
          const theme = APP_CONFIG.statusTheme[robot.status];
          const isSelected = robot.id === selectedRobotId;
          const shortTitle = robot.callsign.split('(')[0].trim();

          return (
            <div
              key={robot.id}
              onClick={() => {
                logFunctionCall('RobotRosterPanel.onSelectRobot', { robotId: robot.id });
                onSelectRobot(robot.id);
              }}
              className={`p-3 transition-colors cursor-pointer ${
                isSelected
                  ? 'bg-slate-800/80 border-l-2 border-l-cyan-400'
                  : 'hover:bg-slate-800/35'
              }`}
            >
              {/* Top Row: Robot Name + Color-Coded Status */}
              <div className="flex items-center justify-between gap-2">
                <span className="text-sm font-semibold text-slate-100">
                  {shortTitle}
                </span>
                <span className={`text-xs font-mono font-semibold whitespace-nowrap shrink-0 ${theme.textClass}`}>
                  {theme.glyph} {theme.shortLabel}
                </span>
              </div>

              {/* Plain-English Plan */}
              <p className="text-xs text-slate-300 mt-1 leading-snug line-clamp-2">
                {robot.planSummary}
              </p>

              {/* Compact Footer: Battery/Seeds + Quick Human Control Toggle */}
              <div className="flex items-center justify-between gap-2 mt-2 pt-1.5 border-t border-slate-800/60 text-xs text-slate-400 font-mono tabular-nums">
                <span>
                  Bat {robot.batteryPct}% · Seeds {robot.seedHopperCount}
                </span>

                <button
                  type="button"
                  onClick={(e) => {
                    e.stopPropagation();
                    logFunctionCall('RobotRosterPanel.onToggleRobotHumanControl', {
                      robotId: robot.id,
                      enable: !robot.humanControlActive,
                    });
                    onSelectRobot(robot.id);
                    onToggleRobotHumanControl(robot.id, !robot.humanControlActive);
                  }}
                  className={`px-2 py-0.5 text-[11px] font-sans font-medium rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                    robot.humanControlActive
                      ? 'bg-purple-600 border-purple-400 text-white hover:bg-purple-500'
                      : 'bg-slate-950 border-slate-700 text-slate-300 hover:border-purple-400 hover:text-purple-300'
                  }`}
                >
                  {robot.humanControlActive ? 'Exit Manual' : 'Human Control'}
                </button>
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
};
