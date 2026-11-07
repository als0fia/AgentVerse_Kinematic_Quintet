/**
 * @file BekkerTerramechanicsLab.tsx
 * @description Interactive Bekker-Wong Wheel-Mud Sinkage & Vehicle Sizing Lab
 * and Mycelial Peer-to-Peer Gossip Log (`antenna.dashboard.v1` specification).
 *
 * Implements the interactive terramechanics and communications inspection requirements
 * from `src/ricebot/viewer/README.md` and `src/ricebot/viewer/dashboard.py`:
 * 1. Interactive Bekker-Wong Wheel-Mud Sinkage Calculator:
 *    - Vehicle Mass m (1.5 - 18.0 kg, default 4.2 kg)
 *    - Wheel Radius r (35 - 110 mm, default 58 mm)
 *    - Wheel Width b (20 - 90 mm, default 40 mm)
 *    - Local Mud Moisture (0.20 - 0.96, default 0.88)
 *    - Live Sinkage z (mm), Belly Clearance (mm), Mud Compaction Drag Rc (N),
 *      and Traversability Verdict (`TRAVERSABLE`, `MARGINAL_SLIP`, `BELLY_OUT_STUCK`).
 * 2. Peer-to-Peer Mycelial Gossip Log (`MycelialGossipPacket`) with Comms Scenario selector
 *    and `antenna.dashboard.v1` JSON snapshot export.
 */

import React, { useState } from 'react';
import { Sliders, Radio, Download, CheckCircle2, AlertTriangle, ShieldAlert } from 'lucide-react';
import {
  BekkerTerramechanicsInput,
  MycelialGossipPacketLog,
} from '../types/antenna';
import { APP_CONFIG } from '../config/appConfig';
import { evaluateBekkerTerramechanics } from '../logic/swarmEngine';
import { logFunctionCall } from '../utils/logger';

interface BekkerTerramechanicsLabProps {
  /** Peer-to-peer mycelial gossip packet history */
  gossipLogs: MycelialGossipPacketLog[];
  /** Callback to export the current `antenna.dashboard.v1` state snapshot as JSON */
  onExportDashboardJson: () => void;
}

const INITIAL_BEKKER_INPUT: BekkerTerramechanicsInput = {
  massKg: APP_CONFIG.roverHomogeneousSpec.defaultMassKg,
  wheelRadiusMm: APP_CONFIG.roverHomogeneousSpec.wheelRadiusMm,
  wheelWidthMm: APP_CONFIG.roverHomogeneousSpec.wheelWidthMm,
  mudMoisture: 0.88,
};

/**
 * Renders the Bekker-Wong Wheel-Mud Sinkage Lab and P2P Mycelial Gossip Inspector.
 */
export const BekkerTerramechanicsLab: React.FC<BekkerTerramechanicsLabProps> = ({
  gossipLogs,
  onExportDashboardJson,
}) => {
  const [bekkerInput, setBekkerInput] = useState<BekkerTerramechanicsInput>(INITIAL_BEKKER_INPUT);
  const [commsScenario, setCommsScenario] = useState<'NORMAL_P2P' | 'BLACKOUT_WINDOW' | 'S1_MUD_RESCUE'>('S1_MUD_RESCUE');

  const bekkerResult = evaluateBekkerTerramechanics(bekkerInput);

  /**
   * Updates a single Bekker-Wong terramechanics parameter slider.
   */
  const handleSliderChange = (key: keyof BekkerTerramechanicsInput, value: number) => {
    logFunctionCall('BekkerTerramechanicsLab.handleSliderChange', { key, value });
    setBekkerInput((prev) => ({
      ...prev,
      [key]: value,
    }));
  };

  /**
   * Resets the Bekker-Wong parameters back to the canonical 4.2kg Medic-Ant rover spec.
   */
  const handleResetSpec = () => {
    logFunctionCall('BekkerTerramechanicsLab.handleResetSpec', {});
    setBekkerInput(INITIAL_BEKKER_INPUT);
  };

  const filteredGossipLogs = gossipLogs.filter((pkt) => {
    if (commsScenario === 'BLACKOUT_WINDOW') {
      return pkt.status === 'BLACKOUT_DROPPED' || pkt.sharedHazardCells > 0;
    }
    if (commsScenario === 'NORMAL_P2P') {
      return pkt.status === 'DELIVERED';
    }
    return true;
  });

  return (
    <div className="bg-slate-900/90 border border-slate-800 rounded-md flex flex-col overflow-hidden">
      {/* Header */}
      <div className="px-3.5 py-2.5 border-b border-slate-800 flex items-center justify-between bg-slate-950/60">
        <div className="flex items-center gap-2">
          <Sliders className="w-3.5 h-3.5 text-cyan-400" />
          <h2 className="text-xs font-semibold uppercase tracking-wider text-slate-200">
            Bekker–Wong Terramechanics Lab &amp; Mycelial Gossip (`antenna.dashboard.v1`)
          </h2>
        </div>
        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={handleResetSpec}
            className="px-2 py-1 rounded text-[11px] font-mono text-slate-300 hover:text-white bg-slate-800/90 hover:bg-slate-700 border border-slate-700 whitespace-nowrap shrink-0 cursor-pointer"
          >
            Reset Medic-Ant Spec (4.2 kg)
          </button>
          <button
            type="button"
            onClick={() => {
              logFunctionCall('BekkerTerramechanicsLab.onExportDashboardJson', {});
              onExportDashboardJson();
            }}
            data-testid="export-dashboard-json-btn"
            className="px-2.5 py-1 rounded text-[11px] font-mono text-cyan-300 hover:text-cyan-200 bg-cyan-950/50 hover:bg-cyan-900/60 border border-cyan-500/40 flex items-center gap-1.5 whitespace-nowrap shrink-0 cursor-pointer"
          >
            <Download className="w-3 h-3" />
            Export dashboard.v1 JSON
          </button>
        </div>
      </div>

      {/* Main 2-column content */}
      <div className="p-3.5 grid grid-cols-1 lg:grid-cols-12 gap-4">
        {/* Left 7 Cols: Interactive Bekker-Wong Wheel-Mud Sinkage & Vehicle Sizing Lab */}
        <div className="lg:col-span-7 flex flex-col justify-between bg-slate-950/70 border border-slate-800/90 rounded p-3">
          <div>
            <div className="flex items-center justify-between mb-2">
              <span className="text-[11px] font-mono uppercase tracking-wider text-slate-400">
                Wheel–Mud Sinkage Equation: z = [ (m·g / 4) / (b · (k_c/b + k_φ) · √(2r)) ]^(2/2n+1)
              </span>
              <span className="text-[11px] font-mono text-slate-400 tabular-nums">
                Hull: {APP_CONFIG.roverHomogeneousSpec.hullDimensionsMm}
              </span>
            </div>

            {/* 4 Sliders */}
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 mt-2.5">
              {/* Mass Slider */}
              <div className="bg-slate-900/90 border border-slate-800/80 rounded p-2">
                <div className="flex justify-between text-[11px] font-mono mb-1">
                  <span className="text-slate-400">Vehicle Mass m</span>
                  <span className="text-slate-100 font-semibold tabular-nums">
                    {bekkerInput.massKg.toFixed(1)} kg
                  </span>
                </div>
                <input
                  type="range"
                  min={1.5}
                  max={18.0}
                  step={0.1}
                  value={bekkerInput.massKg}
                  onChange={(e) => handleSliderChange('massKg', parseFloat(e.target.value))}
                  data-testid="bekker-mass-slider"
                  className="w-full accent-cyan-400 cursor-pointer h-1.5 bg-slate-800 rounded-lg"
                />
                <div className="flex justify-between text-[10px] font-mono text-slate-400 mt-0.5">
                  <span>1.5 kg</span>
                  <span>Default: 4.2 kg</span>
                  <span>18.0 kg</span>
                </div>
              </div>

              {/* Wheel Radius Slider */}
              <div className="bg-slate-900/90 border border-slate-800/80 rounded p-2">
                <div className="flex justify-between text-[11px] font-mono mb-1">
                  <span className="text-slate-400">Wheel Radius r</span>
                  <span className="text-slate-100 font-semibold tabular-nums">
                    {bekkerInput.wheelRadiusMm.toFixed(0)} mm (Ø{(bekkerInput.wheelRadiusMm * 2).toFixed(0)})
                  </span>
                </div>
                <input
                  type="range"
                  min={35}
                  max={110}
                  step={1}
                  value={bekkerInput.wheelRadiusMm}
                  onChange={(e) => handleSliderChange('wheelRadiusMm', parseFloat(e.target.value))}
                  data-testid="bekker-radius-slider"
                  className="w-full accent-cyan-400 cursor-pointer h-1.5 bg-slate-800 rounded-lg"
                />
                <div className="flex justify-between text-[10px] font-mono text-slate-400 mt-0.5">
                  <span>35 mm</span>
                  <span>Default: 58 mm</span>
                  <span>110 mm</span>
                </div>
              </div>

              {/* Wheel Width Slider */}
              <div className="bg-slate-900/90 border border-slate-800/80 rounded p-2">
                <div className="flex justify-between text-[11px] font-mono mb-1">
                  <span className="text-slate-400">Wheel Width b</span>
                  <span className="text-slate-100 font-semibold tabular-nums">
                    {bekkerInput.wheelWidthMm.toFixed(0)} mm
                  </span>
                </div>
                <input
                  type="range"
                  min={20}
                  max={90}
                  step={1}
                  value={bekkerInput.wheelWidthMm}
                  onChange={(e) => handleSliderChange('wheelWidthMm', parseFloat(e.target.value))}
                  data-testid="bekker-width-slider"
                  className="w-full accent-cyan-400 cursor-pointer h-1.5 bg-slate-800 rounded-lg"
                />
                <div className="flex justify-between text-[10px] font-mono text-slate-400 mt-0.5">
                  <span>20 mm</span>
                  <span>Default: 40 mm</span>
                  <span>90 mm</span>
                </div>
              </div>

              {/* Local Mud Moisture Slider */}
              <div className="bg-slate-900/90 border border-slate-800/80 rounded p-2">
                <div className="flex justify-between text-[11px] font-mono mb-1">
                  <span className="text-slate-400">Local Mud Moisture M(x,y)</span>
                  <span className="text-cyan-300 font-semibold tabular-nums">
                    {(bekkerInput.mudMoisture * 100).toFixed(0)}% ({bekkerInput.mudMoisture.toFixed(2)})
                  </span>
                </div>
                <input
                  type="range"
                  min={0.20}
                  max={0.96}
                  step={0.01}
                  value={bekkerInput.mudMoisture}
                  onChange={(e) => handleSliderChange('mudMoisture', parseFloat(e.target.value))}
                  data-testid="bekker-moisture-slider"
                  className="w-full accent-cyan-400 cursor-pointer h-1.5 bg-slate-800 rounded-lg"
                />
                <div className="flex justify-between text-[10px] font-mono text-slate-400 mt-0.5">
                  <span>0.20 (Firm)</span>
                  <span>0.88 (Puddled)</span>
                  <span>0.96 (Pit)</span>
                </div>
              </div>
            </div>
          </div>

          {/* Computed Terramechanics Readout & Verdict */}
          <div className="mt-3 pt-2.5 border-t border-slate-800/80 grid grid-cols-2 sm:grid-cols-4 gap-2.5 items-center">
            <div className="bg-slate-900/80 border border-slate-800 rounded px-2.5 py-1.5">
              <div className="text-[10px] font-mono uppercase text-slate-400">Wheel Sinkage z</div>
              <div
                data-testid="bekker-sinkage-readout"
                className="text-sm font-mono font-bold tabular-nums text-slate-100"
              >
                {bekkerResult.sinkageMm.toFixed(1)} mm
              </div>
            </div>

            <div className="bg-slate-900/80 border border-slate-800 rounded px-2.5 py-1.5">
              <div className="text-[10px] font-mono uppercase text-slate-400">Belly Clearance</div>
              <div
                data-testid="bekker-clearance-readout"
                className={`text-sm font-mono font-bold tabular-nums ${
                  bekkerResult.bellyClearanceMm <= 0
                    ? 'text-red-400'
                    : bekkerResult.bellyClearanceMm < 12
                      ? 'text-amber-400'
                      : 'text-emerald-400'
                }`}
              >
                {bekkerResult.bellyClearanceMm.toFixed(1)} mm
              </div>
            </div>

            <div className="bg-slate-900/80 border border-slate-800 rounded px-2.5 py-1.5">
              <div className="text-[10px] font-mono uppercase text-slate-400">Mud Drag Rc</div>
              <div className="text-sm font-mono font-bold tabular-nums text-slate-100">
                {bekkerResult.mudDragForceN.toFixed(1)} N
              </div>
            </div>

            <div
              data-testid="bekker-verdict-badge"
              className={`rounded px-2.5 py-1.5 border flex items-center gap-1.5 ${
                bekkerResult.verdictCode === 'TRAVERSABLE'
                  ? 'bg-emerald-950/40 border-emerald-500/40 text-emerald-300'
                  : bekkerResult.verdictCode === 'MARGINAL_SLIP'
                    ? 'bg-amber-950/40 border-amber-500/40 text-amber-300'
                    : 'bg-red-950/50 border-red-500/50 text-red-300'
              }`}
            >
              {bekkerResult.verdictCode === 'TRAVERSABLE' && <CheckCircle2 className="w-3.5 h-3.5 shrink-0" />}
              {bekkerResult.verdictCode === 'MARGINAL_SLIP' && (
                <AlertTriangle className="w-3.5 h-3.5 shrink-0" />
              )}
              {bekkerResult.verdictCode === 'BELLY_OUT_STUCK' && (
                <ShieldAlert className="w-3.5 h-3.5 shrink-0" />
              )}
              <div className="min-w-0">
                <div className="text-[10px] font-mono uppercase opacity-80">Verdict</div>
                <div className="text-[11px] font-mono font-bold truncate">
                  {bekkerResult.verdictLabel}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Right 5 Cols: Peer-to-Peer Mycelial Gossip Log & Comms Scenario */}
        <div className="lg:col-span-5 flex flex-col justify-between bg-slate-950/70 border border-slate-800/90 rounded p-3">
          <div>
            <div className="flex items-center justify-between gap-2 mb-2">
              <div className="flex items-center gap-1.5">
                <Radio className="w-3.5 h-3.5 text-emerald-400" />
                <span className="text-[11px] font-mono uppercase tracking-wider text-slate-300 font-semibold">
                  P2P Mycelial Gossip (`MycelialGossipPacket`)
                </span>
              </div>

              {/* Comms Scenario Selector */}
              <div className="flex items-center gap-1">
                {(
                  [
                    { id: 'NORMAL_P2P', label: 'Normal Link' },
                    { id: 'BLACKOUT_WINDOW', label: 'Blackout' },
                    { id: 'S1_MUD_RESCUE', label: 'S1 Rescue' },
                  ] as const
                ).map((mode) => (
                  <button
                    key={mode.id}
                    type="button"
                    onClick={() => setCommsScenario(mode.id)}
                    className={`px-2 py-0.5 rounded text-[10px] font-mono whitespace-nowrap shrink-0 cursor-pointer border ${
                      commsScenario === mode.id
                        ? 'bg-emerald-950/60 text-emerald-300 border-emerald-500/40 font-semibold'
                        : 'bg-slate-900 text-slate-400 border-slate-800 hover:text-slate-200'
                    }`}
                  >
                    {mode.label}
                  </button>
                ))}
              </div>
            </div>

            {/* Packet Feed */}
            <div className="space-y-1.5 max-h-44 overflow-y-auto pr-1">
              {filteredGossipLogs.map((pkt) => (
                <div
                  key={pkt.id}
                  className="bg-slate-900/90 border border-slate-800/90 rounded px-2.5 py-1.5 text-[11px] font-mono"
                >
                  <div className="flex items-center justify-between gap-2">
                    <span className="text-slate-200 font-semibold">
                      t={pkt.timestampSec}s · {pkt.senderId} → {pkt.receiverId}
                    </span>
                    <span
                      className={
                        pkt.status === 'DELIVERED'
                          ? 'text-emerald-400'
                          : 'text-amber-400'
                      }
                    >
                      {pkt.status === 'DELIVERED' ? '● DELIVERED' : '▲ BLACKOUT DROPPED'} · {pkt.distanceM.toFixed(1)}m ({pkt.rssiDbm} dBm)
                    </span>
                  </div>
                  <div className="text-slate-400 mt-0.5">
                    {pkt.summary}
                  </div>
                </div>
              ))}
            </div>
          </div>

          <div className="mt-2.5 pt-2 border-t border-slate-800/80 flex items-center justify-between text-[11px] font-mono text-slate-400">
            <span>Gossip Radius: ≤14.0m (ESP-NOW / Wi-Fi HaLow)</span>
            <span>Provenance Dedup: Active</span>
          </div>
        </div>
      </div>
    </div>
  );
};
