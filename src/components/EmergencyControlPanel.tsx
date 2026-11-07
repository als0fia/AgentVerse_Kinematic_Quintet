/**
 * @file src/components/EmergencyControlPanel.tsx
 * @description Clean Manual Human Control & Recovery Pad for the Selected Robot.
 *
 * Provides:
 * - Directional Drive Pad (North, South, East, West, Hold + WASD / Arrow keys).
 * - Speed slider (`25%` to `100%`).
 * - Simple, user-friendly recovery buttons (`Unstick Mud`, `Latch / Unlatch Peer`, `Lift Peer`).
 */

import React, { useEffect, useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import { RobotState } from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface EmergencyControlPanelProps {
  selectedRobot: RobotState;
  fleetEmergencyStopActive: boolean;
  onToggleRobotHumanControl: (robotId: string, enable: boolean) => void;
  onToggleFleetEmergencyStop: () => void;
  onManualDrive: (robotId: string, dx: number, dy: number, speedScale: number) => void;
  onTriggerClayBreaker: (robotId: string) => void;
  onTriggerStrawClearing: (robotId: string) => void;
  onToggleCamLatch: (robotId: string) => void;
  onAdjustDockPitch: (robotId: string, deltaRad: number) => void;
  onToggleAnchorSpades: (robotId: string) => void;
}

/**
 * Renders the clean Manual Drive & Recovery Pad for the selected robot.
 */
export const EmergencyControlPanel: React.FC<EmergencyControlPanelProps> = ({
  selectedRobot,
  onToggleRobotHumanControl,
  onManualDrive,
  onTriggerClayBreaker,
  onTriggerStrawClearing,
  onToggleCamLatch,
  onAdjustDockPitch,
  onToggleAnchorSpades,
}) => {
  const [speedScale, setSpeedScale] = useState<number>(0.5);
  const [lastManualActionText, setLastManualActionText] = useState<string>(
    'Use the directional pad or arrow keys to drive manually.'
  );

  const stepM = APP_CONFIG.simulation.manualStepDistanceMeters;
  const { rigidDock } = selectedRobot.telemetry;

  /**
   * Dispatches a manual directional drive step for the selected robot.
   */
  function handleDirectionalStep(directionLabel: string, dx: number, dy: number): void {
    logFunctionCall('EmergencyControlPanel.handleDirectionalStep', {
      robotId: selectedRobot.id,
      directionLabel,
      dx,
      dy,
      speedScale,
    });
    onManualDrive(selectedRobot.id, dx, dy, speedScale);
    setLastManualActionText(`Moved ${selectedRobot.id} ${directionLabel}`);
  }

  useEffect(() => {
    if (!selectedRobot.humanControlActive) {
      return;
    }

    function handleKeyDown(e: KeyboardEvent): void {
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') {
        return;
      }

      if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') {
        e.preventDefault();
        handleDirectionalStep('North', 0, -stepM);
      } else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') {
        e.preventDefault();
        handleDirectionalStep('South', 0, stepM);
      } else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') {
        e.preventDefault();
        handleDirectionalStep('West', -stepM, 0);
      } else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') {
        e.preventDefault();
        handleDirectionalStep('East', stepM, 0);
      }
    }

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [selectedRobot.id, selectedRobot.humanControlActive, speedScale]);

  return (
    <section
      aria-label="Emergency Human Control Mode Console"
      className="p-4 space-y-4"
    >
      {/* Directional Teleoperation D-Pad */}
      <div className="p-3.5 bg-slate-950 border border-slate-800 rounded-md">
        <div className="flex items-center justify-between gap-2 mb-2">
          <span className="text-xs font-semibold text-slate-200">
            Manual Drive Pad (WASD / Arrow Keys)
          </span>
          <span className="text-[11px] font-mono text-slate-400 tabular-nums">
            ({selectedRobot.position.x.toFixed(1)}m, {selectedRobot.position.y.toFixed(1)}m)
          </span>
        </div>

        <div className="grid grid-cols-3 gap-2 max-w-[220px] mx-auto my-2">
          <div />
          <button
            type="button"
            aria-label={`Drive ${selectedRobot.id} North`}
            onClick={() => handleDirectionalStep('North', 0, -stepM)}
            className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
          >
            ▲ N
          </button>
          <div />

          <button
            type="button"
            aria-label={`Drive ${selectedRobot.id} West`}
            onClick={() => handleDirectionalStep('West', -stepM, 0)}
            className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
          >
            ◀ W
          </button>
          <button
            type="button"
            aria-label={`Halt ${selectedRobot.id} in place`}
            onClick={() => {
              onToggleRobotHumanControl(selectedRobot.id, true);
              setLastManualActionText(`${selectedRobot.id} stopped in place.`);
            }}
            className="py-2 px-2 bg-rose-950/80 hover:bg-rose-900 border border-rose-500/40 rounded text-[11px] font-mono font-bold text-rose-200 transition-colors cursor-pointer"
          >
            STOP
          </button>
          <button
            type="button"
            aria-label={`Drive ${selectedRobot.id} East`}
            onClick={() => handleDirectionalStep('East', stepM, 0)}
            className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
          >
            E ▶
          </button>

          <div />
          <button
            type="button"
            aria-label={`Drive ${selectedRobot.id} South`}
            onClick={() => handleDirectionalStep('South', 0, stepM)}
            className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
          >
            ▼ S
          </button>
          <div />
        </div>

        {/* Speed Slider */}
        <div className="mt-3 pt-2.5 border-t border-slate-800/80">
          <div className="flex items-center justify-between text-xs mb-1">
            <label htmlFor="teleop-speed-slider" className="text-slate-400">
              Drive Speed:
            </label>
            <span className="font-mono text-cyan-300 tabular-nums">
              {(speedScale * 100).toFixed(0)}%
            </span>
          </div>
          <input
            id="teleop-speed-slider"
            type="range"
            min="0.25"
            max="1.0"
            step="0.25"
            value={speedScale}
            onChange={(e) => setSpeedScale(parseFloat(e.target.value))}
            className="w-full accent-cyan-400 cursor-pointer"
          />
        </div>
      </div>

      {/* Simple Physical Recovery Actions */}
      <div className="space-y-2">
        <div className="text-xs font-semibold text-slate-300">
          Recovery &amp; Docking Tools
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => {
              onTriggerClayBreaker(selectedRobot.id);
              setLastManualActionText(`Applied mud-unstick rocking pulse on ${selectedRobot.id}.`);
            }}
            className="px-3 py-2 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-amber-300 transition-colors cursor-pointer"
          >
            Unstick Mud
          </button>

          <button
            type="button"
            onClick={() => {
              onTriggerStrawClearing(selectedRobot.id);
              setLastManualActionText(`Cleared wheel straw on ${selectedRobot.id}.`);
            }}
            className="px-3 py-2 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-cyan-300 transition-colors cursor-pointer"
          >
            Clear Wheel Straw
          </button>

          <button
            type="button"
            onClick={() => {
              onToggleCamLatch(selectedRobot.id);
              setLastManualActionText(`Toggled docking latch on ${selectedRobot.id}.`);
            }}
            className={`px-3 py-2 text-xs font-medium rounded border transition-colors cursor-pointer ${
              rigidDock.isRigidLatched
                ? 'bg-cyan-950/70 border-cyan-500/50 text-cyan-200'
                : 'bg-slate-950 border-slate-700 text-slate-300 hover:bg-slate-800'
            }`}
          >
            {rigidDock.isRigidLatched ? 'Dock: Latched' : 'Dock: Open'}
          </button>

          <button
            type="button"
            onClick={() => {
              onAdjustDockPitch(selectedRobot.id, 0.1);
              onToggleAnchorSpades(selectedRobot.id);
              setLastManualActionText(`Engaged pry-lift boom on ${selectedRobot.id}.`);
            }}
            className="px-3 py-2 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-slate-200 transition-colors cursor-pointer"
          >
            Pry-Lift Peer
          </button>
        </div>
      </div>

      <div className="text-xs text-slate-400 pt-1 border-t border-slate-800/80">
        {lastManualActionText}
      </div>
    </section>
  );
};
