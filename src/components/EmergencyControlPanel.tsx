/**
 * @file src/components/EmergencyControlPanel.tsx
 * @description Emergency Human Control Mode & Direct Teleoperation Override Deck for Antenna.
 *
 * This component provides the dedicated Human-in-the-Loop emergency override interface required
 * when a homogeneous Medic-Ant rover encounters severe mud sinkage (`S1`), perimeter drainage ditches (`S4`),
 * or rigid docking overload (`S7`), or whenever the human supervisor chooses to take manual control:
 * - Single-click Emergency Human Control Mode toggle for the selected robot.
 * - Global Fleet Emergency Stop (E-Stop) button that halts all 5 rovers simultaneously.
 * - Manual Directional Teleop Pad (North, South, East, West + Keyboard WASD / Arrow Keys) with
 *   configurable crawl speed scaling (`0.25x` Low-RPM Mud Crawl to `1.0x` Normal).
 * - Specialized Physical Recovery & Rigid Docking Actuators (`paddy_ant_rigid_rescue.xml`):
 *   1. `Micro-Rocking Clay Breaker`: Alternating low-duty-cycle torque to break clay suction (`S6`).
 *   2. `Reverse-Polarity Axle Clearing`: Sheds wrapped rice straw (`S5`).
 *   3. `Collinear Rigid Docking, Cam-Latch & Pitch-Lift Controller`:
 *      - Front Active Pitch-Lift Boom (`-0.20` to `+0.45 rad`) to pry victim belly out of suction.
 *      - Rotary Cam-Latch Jaw (`LOCKED 0.60 rad` / `OPEN 0.00 rad`).
 *      - Rear Deployable Hardpan Anchor Spades (`0.85 rad` deployed / `0.00 rad` stowed).
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
 * Renders the Emergency Human Control & Rigid Docking Teleoperation Console for the selected robot and fleet.
 */
export const EmergencyControlPanel: React.FC<EmergencyControlPanelProps> = ({
  selectedRobot,
  fleetEmergencyStopActive,
  onToggleRobotHumanControl,
  onToggleFleetEmergencyStop,
  onManualDrive,
  onTriggerClayBreaker,
  onTriggerStrawClearing,
  onToggleCamLatch,
  onAdjustDockPitch,
  onToggleAnchorSpades,
}) => {
  const [speedScale, setSpeedScale] = useState<number>(0.5);
  const [keyboardCaptureEnabled, setKeyboardCaptureEnabled] = useState<boolean>(true);
  const [lastManualActionText, setLastManualActionText] = useState<string>(
    'Ready for human teleoperation or rigid docking actuator input.'
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
    setLastManualActionText(
      `Manual drive ${directionLabel} (${(dx * speedScale).toFixed(2)}m, ${(dy * speedScale).toFixed(
        2
      )}m) applied to ${selectedRobot.id}`
    );
  }

  // Listen for WASD / Arrow keys when human control is active on the selected robot
  useEffect(() => {
    if (!selectedRobot.humanControlActive || !keyboardCaptureEnabled) {
      return;
    }

    function handleKeyDown(e: KeyboardEvent): void {
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') {
        return;
      }

      if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') {
        e.preventDefault();
        handleDirectionalStep('North (-Y)', 0, -stepM);
      } else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') {
        e.preventDefault();
        handleDirectionalStep('South (+Y)', 0, stepM);
      } else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') {
        e.preventDefault();
        handleDirectionalStep('West (-X)', -stepM, 0);
      } else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') {
        e.preventDefault();
        handleDirectionalStep('East (+X)', stepM, 0);
      }
    }

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [selectedRobot.id, selectedRobot.humanControlActive, keyboardCaptureEnabled, speedScale]);

  return (
    <section
      aria-label="Emergency Human Control Mode Console"
      className="flex flex-col bg-slate-900 border border-slate-800 rounded-lg overflow-hidden"
    >
      {/* Header Banner */}
      <div className="flex items-center justify-between gap-2 px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div>
          <h2 className="text-sm font-semibold text-slate-100">
            Emergency Human Control Mode
          </h2>
          <p className="text-xs text-slate-400">
            Target Unit: <span className="font-mono text-slate-200">{selectedRobot.callsign}</span>
          </p>
        </div>

        <span
          className={`text-xs font-mono font-semibold ${
            selectedRobot.humanControlActive ? 'text-purple-400' : 'text-slate-400'
          }`}
        >
          {selectedRobot.humanControlActive ? '■ MANUAL OVERRIDE ACTIVE' : '● AUTONOMOUS'}
        </span>
      </div>

      <div className="p-4 space-y-4">
        {/* Primary Emergency Mode Switches */}
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-2.5">
          <button
            type="button"
            onClick={() => {
              const nextState = !selectedRobot.humanControlActive;
              logFunctionCall('EmergencyControlPanel.toggleSingleRobotHumanMode', {
                robotId: selectedRobot.id,
                nextState,
              });
              onToggleRobotHumanControl(selectedRobot.id, nextState);
              setLastManualActionText(
                nextState
                  ? `Engaged direct human control on ${selectedRobot.callsign}`
                  : `Returned ${selectedRobot.callsign} to autonomous planner`
              );
            }}
            className={`px-3.5 py-2.5 text-xs font-semibold rounded-md border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              selectedRobot.humanControlActive
                ? 'bg-purple-600 border-purple-400 text-white hover:bg-purple-500'
                : 'bg-purple-950/50 border-purple-500/50 text-purple-200 hover:bg-purple-900/60'
            }`}
          >
            {selectedRobot.humanControlActive
              ? `Release ${selectedRobot.id} to Autonomous`
              : `Take Human Control of ${selectedRobot.id}`}
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('EmergencyControlPanel.toggleFleetEmergencyStop', {
                nextState: !fleetEmergencyStopActive,
              });
              onToggleFleetEmergencyStop();
              setLastManualActionText(
                !fleetEmergencyStopActive
                  ? 'GLOBAL E-STOP ENGAGED: All 5 rovers halted.'
                  : 'Global E-Stop released: Swarm resumed.'
              );
            }}
            className={`px-3.5 py-2.5 text-xs font-semibold rounded-md border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              fleetEmergencyStopActive
                ? 'bg-emerald-600 border-emerald-400 text-white hover:bg-emerald-500'
                : 'bg-rose-600 border-rose-400 text-white hover:bg-rose-500'
            }`}
          >
            {fleetEmergencyStopActive ? 'Release Global Fleet E-Stop' : 'Global Fleet E-Stop (All 5)'}
          </button>
        </div>

        {/* Directional Teleoperation D-Pad & Crawl Speed Selector */}
        <div className="p-3.5 bg-slate-950 border border-slate-800 rounded-md">
          <div className="flex items-center justify-between gap-2 mb-3">
            <span className="text-xs font-semibold text-slate-200">
              Manual Teleop Drive Pad (WASD / Arrows)
            </span>
            <label className="flex items-center gap-1.5 text-xs text-slate-400 cursor-pointer">
              <input
                type="checkbox"
                checked={keyboardCaptureEnabled}
                onChange={(e) => setKeyboardCaptureEnabled(e.target.checked)}
                className="rounded border-slate-700 bg-slate-900 text-cyan-500 focus:ring-cyan-500"
              />
              <span>Keyboard Active</span>
            </label>
          </div>

          <div className="grid grid-cols-3 gap-2 max-w-[240px] mx-auto my-2">
            <div />
            <button
              type="button"
              aria-label={`Drive ${selectedRobot.id} North`}
              onClick={() => handleDirectionalStep('North (-Y)', 0, -stepM)}
              className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
            >
              ▲ N
            </button>
            <div />

            <button
              type="button"
              aria-label={`Drive ${selectedRobot.id} West`}
              onClick={() => handleDirectionalStep('West (-X)', -stepM, 0)}
              className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
            >
              ◀ W
            </button>
            <button
              type="button"
              aria-label={`Halt ${selectedRobot.id} in place`}
              onClick={() => {
                onToggleRobotHumanControl(selectedRobot.id, true);
                setLastManualActionText(`${selectedRobot.id} wheel velocity zeroed (HOLD).`);
              }}
              className="py-2 px-2 bg-rose-950/80 hover:bg-rose-900 border border-rose-500/40 rounded text-[11px] font-mono font-bold text-rose-200 transition-colors cursor-pointer"
            >
              HOLD
            </button>
            <button
              type="button"
              aria-label={`Drive ${selectedRobot.id} East`}
              onClick={() => handleDirectionalStep('East (+X)', stepM, 0)}
              className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
            >
              E ▶
            </button>

            <div />
            <button
              type="button"
              aria-label={`Drive ${selectedRobot.id} South`}
              onClick={() => handleDirectionalStep('South (+Y)', 0, stepM)}
              className="py-2 px-3 bg-slate-900 hover:bg-slate-800 active:bg-purple-600 border border-slate-700 rounded text-xs font-mono font-semibold text-slate-100 transition-colors cursor-pointer"
            >
              ▼ S
            </button>
            <div />
          </div>

          {/* Crawl Torque / Speed Limiter Slider */}
          <div className="mt-3 pt-3 border-t border-slate-800/80">
            <div className="flex items-center justify-between text-xs mb-1">
              <label htmlFor="teleop-speed-slider" className="text-slate-400">
                Low-RPM Mud Crawl Limiter:
              </label>
              <span className="font-mono text-cyan-300 tabular-nums">
                {(speedScale * 100).toFixed(0)}% ({(0.45 * speedScale).toFixed(2)} m/s)
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

        {/* Physical Recovery & Rigid Docking Actuators (`paddy_ant_rigid_rescue.xml`) */}
        <div className="space-y-2.5">
          <div className="text-xs font-semibold text-slate-300">
            Rigid Docking, Cam-Latch &amp; Pry-Lift Actuators (`paddy_ant_rigid_rescue.xml`)
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
            <button
              type="button"
              onClick={() => {
                logFunctionCall('EmergencyControlPanel.onTriggerClayBreaker', {
                  robotId: selectedRobot.id,
                });
                onTriggerClayBreaker(selectedRobot.id);
                setLastManualActionText(
                  `Executed +- micro-rocking torque profile on ${selectedRobot.id} (clay suction relief).`
                );
              }}
              className="px-3 py-2 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-slate-200 text-left transition-colors cursor-pointer"
            >
              <span className="block font-semibold text-amber-300">Micro-Rock Clay Breaker</span>
              <span className="text-[11px] text-slate-400">Break S1/S6 mud suction (≤ 5s pulse)</span>
            </button>

            <button
              type="button"
              onClick={() => {
                logFunctionCall('EmergencyControlPanel.onTriggerStrawClearing', {
                  robotId: selectedRobot.id,
                });
                onTriggerStrawClearing(selectedRobot.id);
                setLastManualActionText(
                  `Executed reverse-polarity hub oscillation on ${selectedRobot.id} (straw cleared).`
                );
              }}
              className="px-3 py-2 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-slate-200 text-left transition-colors cursor-pointer"
            >
              <span className="block font-semibold text-cyan-300">Clear Hub Straw Stall</span>
              <span className="text-[11px] text-slate-400">Reverse-polarity S5 weed unwrapping</span>
            </button>
          </div>

          {/* Rigid Docking Cam-Latch, Pitch-Lift Boom, and Anchor Spades */}
          <div className="p-2.5 bg-slate-950 border border-slate-800 rounded space-y-2">
            <div className="flex items-center justify-between text-xs font-mono">
              <span className="text-slate-400">
                Pitch Boom:{' '}
                <strong className="text-cyan-300 tabular-nums">
                  {rigidDock.dockPitchRad >= 0 ? '+' : ''}
                  {rigidDock.dockPitchRad.toFixed(2)} rad
                </strong>
              </span>
              <span className="text-slate-400">
                Cam Jaw:{' '}
                <strong className="text-slate-200 tabular-nums">
                  {rigidDock.camLatchRad.toFixed(2)} rad
                </strong>
              </span>
            </div>

            <div className="flex flex-wrap items-center gap-1.5">
              <button
                type="button"
                onClick={() => {
                  onToggleCamLatch(selectedRobot.id);
                  setLastManualActionText(
                    `Toggled rotary cam-latch jaw on ${selectedRobot.id}`
                  );
                }}
                className={`px-2.5 py-1 text-xs font-mono rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  rigidDock.isRigidLatched
                    ? 'bg-cyan-950/70 border-cyan-500/50 text-cyan-200 font-semibold'
                    : 'bg-slate-900 border-slate-700 text-slate-300 hover:bg-slate-800'
                }`}
              >
                {rigidDock.isRigidLatched ? 'Cam-Latch: LOCKED' : 'Cam-Latch: OPEN'}
              </button>

              <button
                type="button"
                onClick={() => {
                  onAdjustDockPitch(selectedRobot.id, -0.1);
                  setLastManualActionText(
                    `Lowered front docking boom by -0.10 rad on ${selectedRobot.id}`
                  );
                }}
                className="px-2 py-1 text-xs font-mono bg-slate-900 hover:bg-slate-800 border border-slate-700 rounded text-slate-200 whitespace-nowrap shrink-0 cursor-pointer"
              >
                Pitch -0.10 rad
              </button>

              <button
                type="button"
                onClick={() => {
                  onAdjustDockPitch(selectedRobot.id, 0.1);
                  setLastManualActionText(
                    `Raised front pitch-lift boom by +0.10 rad on ${selectedRobot.id} (pry-lift)`
                  );
                }}
                className="px-2 py-1 text-xs font-mono bg-slate-900 hover:bg-slate-800 border border-slate-700 rounded text-cyan-300 whitespace-nowrap shrink-0 cursor-pointer"
              >
                Pry-Lift +0.10 rad
              </button>

              <button
                type="button"
                onClick={() => {
                  onToggleAnchorSpades(selectedRobot.id);
                  setLastManualActionText(
                    `Toggled rear hardpan anchor spades on ${selectedRobot.id}`
                  );
                }}
                className={`px-2.5 py-1 text-xs font-mono rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  rigidDock.anchorSpadeRad > 0.1
                    ? 'bg-amber-950/70 border-amber-500/50 text-amber-200 font-semibold'
                    : 'bg-slate-900 border-slate-700 text-slate-300 hover:bg-slate-800'
                }`}
              >
                {rigidDock.anchorSpadeRad > 0.1 ? 'Spades: PLANTED' : 'Spades: STOWED'}
              </button>
            </div>
          </div>
        </div>

        {/* Last Teleoperation Feedback Line */}
        <div className="text-xs font-mono text-slate-400 pt-1 border-t border-slate-800/80">
          Telemetry Ack: <span className="text-slate-200">{lastManualActionText}</span>
        </div>
      </div>
    </section>
  );
};
