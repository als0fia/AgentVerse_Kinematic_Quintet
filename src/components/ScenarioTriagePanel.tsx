/**
 * @file src/components/ScenarioTriagePanel.tsx
 * @description Paddy Immobilization Scenario Simulator (`S1`–`S8` + Negative Controls) & Pareto Rescue Evaluator (`D-0011`).
 *
 * This component brings `docs/IMMOBILIZATION_SCENARIOS.md` and `docs/DECISIONS.md` (`D-0011`) from
 * the `AgentVerse_Kinematic_Quintet` repository directly into an interactive diagnostic console:
 * - Allows the operator to inject any of the 8 physical paddy field failure hypotheses (`S1`–`S8`)
 *   OR the 2 Negative Control scenarios (`NEG_MISSING_LIDAR_FAULT` where `front_range_m = None`,
 *   and `NEG_INTENTIONAL_SEED_HOLD`) that block false-positive rescue dispatches.
 * - Provides interactive sliders for the 5 Pareto Rescue Cost Weights (`D-0011`):
 *   $J = w_t T + w_e E + w_c C + w_s S + w_r R$
 *   (`timeWeight`, `energyWeight`, `cropDamageWeight`, `soilDisturbanceWeight`, `recoveryRiskWeight`).
 * - Provides a one-click button on each candidate policy to execute that recovery plan across the fleet.
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
 * Renders the S1–S8 + Negative Control immobilization scenario injector and D-0011 Pareto rescue comparison table.
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
   * Updates one of the 5 D-0011 Pareto rescue cost weights.
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
      aria-label="Paddy Immobilization Fault Triage and Multi-Objective Rescue Evaluator"
      className="space-y-6"
    >
      {/* Top Scenario Injection Control Deck */}
      <div className="p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-4">
        <div className="flex flex-wrap items-center justify-between gap-4">
          <div>
            <h2 className="text-base font-semibold text-slate-100">
              Paddy Field Immobilization Hypotheses (S1–S8) &amp; Negative Controls
            </h2>
            <p className="text-xs text-slate-400">
              200Hz STM32H7 slip ratio classification · Distinguishes true immobilization from missing range data (`front_range_m = None`)
            </p>
          </div>

          <div className="flex items-center gap-2">
            <label htmlFor="triage-robot-select" className="text-xs text-slate-400">
              Target Rover:
            </label>
            <select
              id="triage-robot-select"
              value={selectedRobot.id}
              onChange={(e) => onSelectRobot(e.target.value)}
              className="px-3 py-1.5 text-xs bg-slate-950 border border-slate-700 rounded text-slate-100 font-mono"
            >
              {robots.map((r) => (
                <option key={r.id} value={r.id}>
                  {r.callsign} ({r.faultScenario === 'NONE' ? 'Nominal' : r.faultScenario})
                </option>
              ))}
            </select>

            {selectedRobot.faultScenario !== 'NONE' && (
              <button
                type="button"
                onClick={() => {
                  logFunctionCall('ScenarioTriagePanel.clearFault', {
                    robotId: selectedRobot.id,
                  });
                  onInjectFaultScenario(selectedRobot.id, 'NONE');
                }}
                className="px-3 py-1.5 text-xs font-semibold bg-emerald-600 hover:bg-emerald-500 text-white rounded transition-colors cursor-pointer"
              >
                Clear Fault on {selectedRobot.id}
              </button>
            )}
          </div>
        </div>

        {/* S1 - S8 + Negative Control Scenario Cards */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
          {FAULT_SCENARIO_CATALOG.map((scenario) => {
            const isActiveOnSelected = selectedRobot.faultScenario === scenario.id;
            const isNegControl = scenario.category === 'NEGATIVE_CONTROL';
            return (
              <div
                key={scenario.id}
                className={`p-3.5 rounded border flex flex-col justify-between transition-colors ${
                  isActiveOnSelected
                    ? isNegControl
                      ? 'bg-emerald-950/40 border-emerald-500/60'
                      : 'bg-rose-950/40 border-rose-500/60'
                    : 'bg-slate-950 border-slate-800'
                }`}
              >
                <div>
                  <div className="flex items-center justify-between gap-2 text-xs font-mono mb-1">
                    <span
                      className={`font-bold ${
                        isNegControl ? 'text-emerald-400' : 'text-rose-400'
                      }`}
                    >
                      {scenario.code} · {isNegControl ? 'NEGATIVE CONTROL' : 'IMMOBILIZATION'}
                    </span>
                    <span className="text-slate-400">
                      {isNegControl ? 'NO RESCUE' : `Policy: ${scenario.recommendedPolicy.split('_')[0]}`}
                    </span>
                  </div>
                  <h3 className="text-sm font-semibold text-slate-100 mb-1">
                    {scenario.title}
                  </h3>
                  <p className="text-xs text-slate-400 leading-relaxed mb-2">
                    {scenario.physicalMechanism}
                  </p>
                  <div className="text-[11px] font-mono text-slate-300 bg-slate-900/90 p-2 rounded border border-slate-800">
                    Signature: {scenario.sensorSignature}
                  </div>
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
                  className={`mt-3 w-full py-1.5 px-3 text-xs font-semibold rounded border transition-colors cursor-pointer ${
                    isActiveOnSelected
                      ? isNegControl
                        ? 'bg-emerald-600 border-emerald-400 text-white'
                        : 'bg-rose-600 border-rose-400 text-white'
                      : 'bg-slate-900 hover:bg-slate-800 border-slate-700 text-slate-200'
                  }`}
                >
                  {isActiveOnSelected
                    ? `Active on ${selectedRobot.id}`
                    : `Simulate ${scenario.code} on ${selectedRobot.id}`}
                </button>
              </div>
            );
          })}
        </div>
      </div>

      {/* D-0011 Multi-Objective Pareto Rescue Cost Weight Sliders & Evaluation Table */}
      <div className="p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-4">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <div>
            <h3 className="text-sm font-semibold text-slate-100">
              D-0011 Pareto Multi-Objective Rescue Evaluator for {triageTargetRobot.callsign}
            </h3>
            <p className="text-xs text-slate-400">
              Active Incident:{' '}
              <span className="font-mono text-rose-300">
                {triageTargetRobot.faultScenario === 'NONE'
                  ? 'No active immobilization (showing baseline contingency matrix)'
                  : triageTargetRobot.faultScenario}
              </span>
            </p>
          </div>
          <div className="text-xs font-mono text-cyan-300">
            J = w_t·T + w_e·E + w_c·C + w_s·S + w_r·R
          </div>
        </div>

        {/* Interactive Pareto Weight Sliders (w_t, w_e, w_c, w_s, w_r) */}
        <div className="grid grid-cols-1 sm:grid-cols-5 gap-2.5 p-3 bg-slate-950 border border-slate-800 rounded">
          {(
            [
              { key: 'timeWeight', label: 'w_t (Time / Delay)' },
              { key: 'energyWeight', label: 'w_e (Energy Wh)' },
              { key: 'cropDamageWeight', label: 'w_c (Crop Damage)' },
              { key: 'soilDisturbanceWeight', label: 'w_s (Soil / Rutting)' },
              { key: 'recoveryRiskWeight', label: 'w_r (2nd Trap Risk)' },
            ] as const
          ).map((item) => (
            <div key={item.key} className="bg-slate-900 border border-slate-800/80 rounded p-2">
              <div className="flex items-center justify-between text-[11px] font-mono mb-1">
                <span className="text-slate-400">{item.label}</span>
                <span className="text-cyan-300 font-bold tabular-nums">
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

        <div className="overflow-x-auto">
          <table className="w-full text-left border-collapse text-xs">
            <thead>
              <tr className="bg-slate-950 border-b border-slate-800 text-slate-400 font-mono">
                <th className="py-2.5 px-3">CANDIDATE POLICY</th>
                <th className="py-2.5 px-3">RESULTING ROBOT PLAN</th>
                <th className="py-2.5 px-3 text-right">UTILITY J(a)</th>
                <th className="py-2.5 px-3 text-right">CROP DMG</th>
                <th className="py-2.5 px-3 text-right">2ND TRAP RISK</th>
                <th className="py-2.5 px-3 text-right">EXECUTE</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-slate-800">
              {rescueEvaluations.map((candidate) => (
                <tr
                  key={candidate.policyCode}
                  className={
                    candidate.recommended
                      ? 'bg-cyan-950/30 hover:bg-cyan-950/50'
                      : 'hover:bg-slate-800/40'
                  }
                >
                  <td className="py-3 px-3 font-mono whitespace-nowrap">
                    <span
                      className={`font-semibold ${
                        candidate.recommended ? 'text-cyan-300' : 'text-slate-200'
                      }`}
                    >
                      {candidate.recommended ? '★ ' : ''}
                      {candidate.policyCode}
                    </span>
                    <span className="block text-[11px] font-sans text-slate-400">
                      {candidate.title}
                    </span>
                  </td>
                  <td className="py-3 px-3 text-slate-200 max-w-md">
                    <div className="font-medium text-slate-100">{candidate.actionPlanText}</div>
                    <div className="text-[11px] text-slate-400 mt-0.5">{candidate.rationale}</div>
                  </td>
                  <td className="py-3 px-3 text-right font-mono font-bold text-emerald-400 tabular-nums">
                    {candidate.netUtilityScore.toFixed(1)}
                  </td>
                  <td className="py-3 px-3 text-right font-mono text-slate-300 tabular-nums">
                    {candidate.cropDamageRiskPct.toFixed(1)}%
                  </td>
                  <td className="py-3 px-3 text-right font-mono text-slate-300 tabular-nums">
                    {candidate.secondaryImmobilizationProbPct.toFixed(1)}%
                  </td>
                  <td className="py-3 px-3 text-right whitespace-nowrap">
                    <button
                      type="button"
                      onClick={() => {
                        logFunctionCall('ScenarioTriagePanel.onApplyRescueCandidate', {
                          stuckRobotId: triageTargetRobot.id,
                          policyCode: candidate.policyCode,
                        });
                        onApplyRescueCandidate(triageTargetRobot, candidate);
                      }}
                      className={`px-3 py-1.5 text-xs font-semibold rounded transition-colors cursor-pointer ${
                        candidate.recommended
                          ? 'bg-cyan-600 hover:bg-cyan-500 text-white'
                          : 'bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700'
                      }`}
                    >
                      Apply Plan
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </section>
  );
};
