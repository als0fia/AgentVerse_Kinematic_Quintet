/**
 * @file src/components/ScenarioTriagePanel.tsx
 * @description Clean Fault Simulator & Rescue Decision Assistant.
 *
 * Left column: Select a robot and simulate a field fault (or sensor check).
 * Right column: View recommended rescue options and apply a recovery plan with one click.
 */

import React, { useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import { FAULT_SCENARIO_CATALOG } from '../data/initialSwarmState';
import { evaluateRescueCandidates } from '../logic/swarmEngine';
import {
  FaultScenarioId,
  ParetoWeights,
  RescueCandidateEvaluation,
  RobotState,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface ScenarioTriagePanelProps {
  robots: RobotState[];
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  onInjectFaultScenario: (robotId: string, scenarioId: FaultScenarioId) => void;
  onApplyRescueCandidate: (
    stuckRobot: RobotState,
    candidate: RescueCandidateEvaluation
  ) => void;
}

/**
 * Renders a clean 2-column Fault Simulator and Rescue Option selector.
 */
export const ScenarioTriagePanel: React.FC<ScenarioTriagePanelProps> = ({
  robots,
  selectedRobotId,
  onSelectRobot,
  onInjectFaultScenario,
  onApplyRescueCandidate,
}) => {
  const [paretoWeights, setParetoWeights] = useState<ParetoWeights>({
    ...APP_CONFIG.defaultParetoWeights,
  });
  const [showWeightSliders, setShowWeightSliders] = useState<boolean>(false);

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];
  const immobilizedRobots = robots.filter((r) => r.faultScenario !== 'NONE');
  const triageTargetRobot =
    selectedRobot.faultScenario !== 'NONE'
      ? selectedRobot
      : immobilizedRobots[0] ?? selectedRobot;

  const rescueEvaluations = evaluateRescueCandidates(
    triageTargetRobot,
    robots,
    paretoWeights
  );

  /**
   * Updates one of the rescue priority weights.
   */
  function handleWeightChange(key: keyof ParetoWeights, value: number): void {
    logFunctionCall('ScenarioTriagePanel.handleWeightChange', { key, value });
    setParetoWeights((prev) => ({
      ...prev,
      [key]: value,
    }));
  }

  return (
    <section
      aria-label="Fault Simulator and Rescue Options"
      className="grid grid-cols-1 lg:grid-cols-12 gap-4 items-start"
    >
      {/* Left Column (5 cols): Compact Fault Scenario Selector */}
      <div className="lg:col-span-5 p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-3">
        <div className="flex items-center justify-between gap-2">
          <div>
            <h2 className="text-sm font-semibold text-slate-100">
              Simulate Field Fault
            </h2>
            <p className="text-xs text-slate-400">
              Test how the swarm responds when a robot gets stuck.
            </p>
          </div>

          {selectedRobot.faultScenario !== 'NONE' && (
            <button
              type="button"
              onClick={() => {
                logFunctionCall('ScenarioTriagePanel.clearFault', {
                  robotId: selectedRobot.id,
                });
                onInjectFaultScenario(selectedRobot.id, 'NONE');
              }}
              className="px-2.5 py-1 text-xs font-semibold bg-emerald-600 hover:bg-emerald-500 text-white rounded transition-colors cursor-pointer shrink-0"
            >
              Clear Fault
            </button>
          )}
        </div>

        {/* Target Robot Selector */}
        <div>
          <label htmlFor="triage-robot-select" className="block text-xs text-slate-400 mb-1">
            Target Robot
          </label>
          <select
            id="triage-robot-select"
            value={selectedRobot.id}
            onChange={(e) => onSelectRobot(e.target.value)}
            className="w-full px-3 py-1.5 text-xs bg-slate-950 border border-slate-700 rounded text-slate-100 cursor-pointer"
          >
            {robots.map((r) => (
              <option key={r.id} value={r.id}>
                {r.callsign.split('(')[0].trim()} ({r.faultScenario === 'NONE' ? 'Nominal' : r.faultScenario})
              </option>
            ))}
          </select>
        </div>

        {/* Compact Fault List */}
        <div className="space-y-2 max-h-[480px] overflow-y-auto pr-1">
          {FAULT_SCENARIO_CATALOG.map((scenario) => {
            const isActiveOnSelected = selectedRobot.faultScenario === scenario.id;
            const isNegControl = scenario.category === 'NEGATIVE_CONTROL';
            return (
              <div
                key={scenario.id}
                className={`p-3 rounded border flex items-center justify-between gap-3 transition-colors ${
                  isActiveOnSelected
                    ? isNegControl
                      ? 'bg-emerald-950/40 border-emerald-500/60'
                      : 'bg-rose-950/40 border-rose-500/60'
                    : 'bg-slate-950 border-slate-800'
                }`}
              >
                <div className="min-w-0">
                  <div className="flex items-center gap-2 text-xs">
                    <span
                      className={`font-mono font-bold ${
                        isNegControl ? 'text-emerald-400' : 'text-rose-400'
                      }`}
                    >
                      {scenario.code}
                    </span>
                    <span className="font-semibold text-slate-100 truncate">
                      {scenario.title}
                    </span>
                  </div>
                  <p className="text-xs text-slate-400 mt-0.5 line-clamp-1">
                    {scenario.physicalMechanism}
                  </p>
                </div>

                <button
                  type="button"
                  onClick={() => {
                    logFunctionCall('ScenarioTriagePanel.injectScenario', {
                      robotId: selectedRobot.id,
                      scenarioId: scenario.id,
                    });
                    onInjectFaultScenario(selectedRobot.id, scenario.id);
                  }}
                  className={`px-3 py-1.5 text-xs font-semibold rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                    isActiveOnSelected
                      ? isNegControl
                        ? 'bg-emerald-600 border-emerald-400 text-white'
                        : 'bg-rose-600 border-rose-400 text-white'
                      : 'bg-slate-900 hover:bg-slate-800 border-slate-700 text-slate-200'
                  }`}
                >
                  {isActiveOnSelected ? 'Active' : 'Trigger'}
                </button>
              </div>
            );
          })}
        </div>
      </div>

      {/* Right Column (7 cols): Recommended Rescue Options */}
      <div className="lg:col-span-7 p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-4">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <div>
            <h3 className="text-sm font-semibold text-slate-100">
              Rescue Options for {triageTargetRobot.callsign.split('(')[0].trim()}
            </h3>
            <p className="text-xs text-slate-400">
              Incident:{' '}
              <span className="font-mono text-rose-300">
                {triageTargetRobot.faultScenario === 'NONE'
                  ? 'None (Baseline Preview)'
                  : triageTargetRobot.faultScenario}
              </span>
            </p>
          </div>

          <button
            type="button"
            onClick={() => setShowWeightSliders((v) => !v)}
            className="px-2.5 py-1 text-xs font-medium bg-slate-950 hover:bg-slate-800 border border-slate-700 rounded text-slate-300 cursor-pointer"
          >
            {showWeightSliders ? 'Hide Priority Weights ▲' : 'Tune Priority Weights ▼'}
          </button>
        </div>

        {/* Optional Collapsible Priority Sliders */}
        {showWeightSliders && (
          <div className="grid grid-cols-2 sm:grid-cols-5 gap-2 p-3 bg-slate-950 border border-slate-800 rounded">
            {(
              [
                { key: 'timeWeight', label: 'Time' },
                { key: 'energyWeight', label: 'Energy' },
                { key: 'cropDamageWeight', label: 'Crop Safety' },
                { key: 'soilDisturbanceWeight', label: 'Soil Care' },
                { key: 'recoveryRiskWeight', label: '2nd Robot Risk' },
              ] as const
            ).map((item) => (
              <div key={item.key} className="bg-slate-900 border border-slate-800/80 rounded p-2">
                <div className="flex items-center justify-between text-[11px] mb-1">
                  <span className="text-slate-400">{item.label}</span>
                  <span className="font-mono text-cyan-300 font-bold tabular-nums">
                    {paretoWeights[item.key].toFixed(2)}
                  </span>
                </div>
                <input
                  type="range"
                  min={0.05}
                  max={0.60}
                  step={0.05}
                  value={paretoWeights[item.key]}
                  onChange={(e) => handleWeightChange(item.key, parseFloat(e.target.value))}
                  className="w-full accent-cyan-400 cursor-pointer h-1.5 bg-slate-800 rounded-lg"
                />
              </div>
            ))}
          </div>
        )}

        {/* Clean Rescue Options List */}
        <div className="space-y-2.5">
          {rescueEvaluations.map((candidate) => (
            <div
              key={candidate.policyCode}
              className={`p-3.5 rounded-md border flex flex-col sm:flex-row sm:items-center justify-between gap-3 ${
                candidate.recommended
                  ? 'bg-cyan-950/25 border-cyan-500/50'
                  : 'bg-slate-950 border-slate-800'
              }`}
            >
              <div className="space-y-1 max-w-xl">
                <div className="flex items-center gap-2">
                  <span
                    className={`text-xs font-semibold ${
                      candidate.recommended ? 'text-cyan-300' : 'text-slate-200'
                    }`}
                  >
                    {candidate.recommended ? '★ Recommended: ' : ''}
                    {candidate.title}
                  </span>
                  <span className="text-xs font-mono text-emerald-400 tabular-nums">
                    Score {candidate.netUtilityScore.toFixed(1)}
                  </span>
                </div>
                <p className="text-xs text-slate-200 font-medium">
                  {candidate.actionPlanText}
                </p>
                <div className="text-[11px] font-mono text-slate-400 tabular-nums">
                  Crop Risk: {candidate.cropDamageRiskPct.toFixed(0)}% · 2nd Robot Trap Risk:{' '}
                  {candidate.secondaryImmobilizationProbPct.toFixed(0)}%
                </div>
              </div>

              <button
                type="button"
                onClick={() => {
                  logFunctionCall('ScenarioTriagePanel.onApplyRescueCandidate', {
                    stuckRobotId: triageTargetRobot.id,
                    policyCode: candidate.policyCode,
                  });
                  onApplyRescueCandidate(triageTargetRobot, candidate);
                }}
                className={`px-3.5 py-2 text-xs font-semibold rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                  candidate.recommended
                    ? 'bg-cyan-600 hover:bg-cyan-500 text-white'
                    : 'bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700'
                }`}
              >
                Apply Plan
              </button>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};
