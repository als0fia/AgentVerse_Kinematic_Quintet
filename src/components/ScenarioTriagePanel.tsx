/**
 * @file src/components/ScenarioTriagePanel.tsx
 * @description Paddy Immobilization Scenario Simulator (`S1`–`S6`) & Multi-Objective Rescue Evaluator.
 *
 * This component brings the `stuck_scenario.txt` engineering specifications from the
 * `AgentVerse_Kinematic_Quintet` repository directly into an interactive diagnostic console:
 * - Allows the operator to inject any of the physical paddy field failure scenarios (`S1` Deep Plow-Pan Breach,
 *   `S2` Asymmetric Yaw Pocket, `S3` Earthen Bund High-Centering, `S4` Irrigation Trench Drop-In,
 *   `S5` Rice Straw Entanglement, `S6` Thixotropic Clay Suction Lock) onto a selected robot.
 * - Computes and displays the AI Coordinator Multi-Objective Rescue Evaluation:
 *   $\mathcal{J}(\text{action}) = w_{\text{task}} \Delta \text{Coverage} - w_{\text{crop}} \text{CropDamage} - w_{\text{energy}} \Delta E - w_{\text{risk}} P(\text{SecondaryImmobilization})$
 * - Provides a one-click button on each candidate policy to execute that recovery plan across the fleet.
 *
 * Use Cases:
 * 1. Testing how the swarm responds when a robot gets bogged down or drops into a perimeter drainage trench.
 * 2. Comparing why deterministic risk-aware abstention (`SAFE_ABSTAIN_DEFER_HUMAN`) or crop-safe detour tow
 *    beats naive direct towing.
 */

import React from 'react';
import { FAULT_SCENARIO_CATALOG } from '../data/initialSwarmState';
import { evaluateRescueCandidates } from '../logic/swarmEngine';
import {
  FaultScenarioId,
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
 * Renders the S1–S6 immobilization scenario injector and multi-objective rescue policy comparison table.
 * @param props - ScenarioTriagePanelProps state and scenario/rescue callbacks.
 * @returns React JSX section element.
 */
export const ScenarioTriagePanel: React.FC<ScenarioTriagePanelProps> = ({
  robots,
  selectedRobotId,
  onSelectRobot,
  onInjectFaultScenario,
  onApplyRescueCandidate,
}) => {
  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];
  const immobilizedRobots = robots.filter((r) => r.faultScenario !== 'NONE');
  const triageTargetRobot =
    selectedRobot.faultScenario !== 'NONE'
      ? selectedRobot
      : immobilizedRobots[0] ?? selectedRobot;

  const rescueEvaluations = evaluateRescueCandidates(triageTargetRobot, robots);

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
              Paddy Field Immobilization Diagnostics & Multi-Objective Rescue Triage
            </h2>
            <p className="text-xs text-slate-400">
              200Hz STM32H7 slip ratio classification · Sub-1GHz Wi-Fi HaLow incident broadcast · Cost function J(action)
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
                className="px-3 py-1.5 text-xs font-semibold bg-emerald-600 hover:bg-emerald-500 text-white rounded transition-colors"
              >
                Clear Fault on {selectedRobot.id}
              </button>
            )}
          </div>
        </div>

        {/* S1 - S6 Scenario Cards */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
          {FAULT_SCENARIO_CATALOG.map((scenario) => {
            const isActiveOnSelected = selectedRobot.faultScenario === scenario.id;
            return (
              <div
                key={scenario.id}
                className={`p-3.5 rounded border flex flex-col justify-between transition-colors ${
                  isActiveOnSelected
                    ? 'bg-rose-950/40 border-rose-500/60'
                    : 'bg-slate-950 border-slate-800'
                }`}
              >
                <div>
                  <div className="flex items-center justify-between gap-2 text-xs font-mono mb-1">
                    <span className="font-bold text-rose-400">
                      {scenario.code} · {scenario.chineseTerm}
                    </span>
                    <span className="text-slate-400">
                      Policy: {scenario.recommendedPolicy.split('_')[0]}
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
                  className={`mt-3 w-full py-1.5 px-3 text-xs font-semibold rounded border transition-colors ${
                    isActiveOnSelected
                      ? 'bg-rose-600 border-rose-400 text-white'
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

      {/* Multi-Objective Coordinator Rescue Evaluation Table */}
      <div className="p-4 bg-slate-900 border border-slate-800 rounded-lg space-y-3">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <div>
            <h3 className="text-sm font-semibold text-slate-100">
              AI Coordinator Multi-Objective Rescue Evaluation for {triageTargetRobot.callsign}
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
          <div className="text-xs font-mono text-slate-400">
            J(a) = 0.35·ΔCov - 0.30·CropDmg - 0.15·ΔE - 0.20·P(SecTrap)
          </div>
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
                      className={`px-3 py-1.5 text-xs font-semibold rounded transition-colors ${
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
