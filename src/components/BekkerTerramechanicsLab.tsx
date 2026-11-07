/**
 * @file BekkerTerramechanicsLab.tsx
 * @description Clean Terrain Sinkage Calculator & Peer-to-Peer Comms Feed (`Terrain & Comms` tab).
 *
 * Separated into its own tab so the main Live Dashboard stays uncluttered.
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
  gossipLogs: MycelialGossipPacketLog[];
  onExportDashboardJson: () => void;
}

const INITIAL_BEKKER_INPUT: BekkerTerramechanicsInput = {
  massKg: APP_CONFIG.roverHomogeneousSpec.defaultMassKg,
  wheelRadiusMm: APP_CONFIG.roverHomogeneousSpec.wheelRadiusMm,
  wheelWidthMm: APP_CONFIG.roverHomogeneousSpec.wheelWidthMm,
  mudMoisture: 0.88,
};

/**
 * Renders the Wheel-Mud Sinkage Simulator and P2P Comms Inspector.
 */
export const BekkerTerramechanicsLab: React.FC<BekkerTerramechanicsLabProps> = ({
  gossipLogs,
  onExportDashboardJson,
}) => {
  const [bekkerInput, setBekkerInput] = useState<BekkerTerramechanicsInput>(INITIAL_BEKKER_INPUT);
  const [commsScenario, setCommsScenario] = useState<'NORMAL_P2P' | 'BLACKOUT_WINDOW' | 'S1_MUD_RESCUE'>('S1_MUD_RESCUE');

  const bekkerResult = evaluateBekkerTerramechanics(bekkerInput);

  /**
   * Updates a single parameter slider.
   */
  const handleSliderChange = (key: keyof BekkerTerramechanicsInput, value: number) => {
    logFunctionCall('BekkerTerramechanicsLab.handleSliderChange', { key, value });
    setBekkerInput((prev) => ({
      ...prev,
      [key]: value,
    }));
  };

  /**
   * Resets parameters back to the default 4.2 kg rover spec.
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
    <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 items-start">
      {/* Left 7 Cols: Wheel-Mud Sinkage Simulator */}
      <div className="lg:col-span-7 bg-slate-900 border border-slate-800 rounded-lg p-4 space-y-4">
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center gap-2">
            <Sliders className="w-4 h-4 text-cyan-400" />
            <div>
              <h2 className="text-sm font-semibold text-slate-100">
                Wheel–Mud Sinkage Simulator
              </h2>
              <p className="text-xs text-slate-400">
                Test how robot weight, wheel size, and mud moisture affect ground clearance.
              </p>
            </div>
          </div>

          <button
            type="button"
            onClick={handleResetSpec}
            className="px-2.5 py-1 rounded text-xs text-slate-300 hover:text-white bg-slate-800 hover:bg-slate-700 border border-slate-700 whitespace-nowrap shrink-0 cursor-pointer"
          >
            Reset Default (4.2 kg)
          </button>
        </div>

        {/* 4 Sliders */}
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
          <div className="bg-slate-950 border border-slate-800 rounded p-3">
            <div className="flex justify-between text-xs mb-1.5">
              <span className="text-slate-400">Robot Mass</span>
              <span className="font-mono text-slate-100 font-semibold tabular-nums">
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
              className="w-full accent-cyan-400 cursor-pointer"
            />
          </div>

          <div className="bg-slate-950 border border-slate-800 rounded p-3">
            <div className="flex justify-between text-xs mb-1.5">
              <span className="text-slate-400">Wheel Radius</span>
              <span className="font-mono text-slate-100 font-semibold tabular-nums">
                {bekkerInput.wheelRadiusMm.toFixed(0)} mm
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
              className="w-full accent-cyan-400 cursor-pointer"
            />
          </div>

          <div className="bg-slate-950 border border-slate-800 rounded p-3">
            <div className="flex justify-between text-xs mb-1.5">
              <span className="text-slate-400">Wheel Width</span>
              <span className="font-mono text-slate-100 font-semibold tabular-nums">
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
              className="w-full accent-cyan-400 cursor-pointer"
            />
          </div>

          <div className="bg-slate-950 border border-slate-800 rounded p-3">
            <div className="flex justify-between text-xs mb-1.5">
              <span className="text-slate-400">Mud Moisture</span>
              <span className="font-mono text-cyan-300 font-semibold tabular-nums">
                {(bekkerInput.mudMoisture * 100).toFixed(0)}%
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
              className="w-full accent-cyan-400 cursor-pointer"
            />
          </div>
        </div>

        {/* Results Row */}
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2.5">
          <div className="bg-slate-950 border border-slate-800 rounded p-2.5">
            <div className="text-xs text-slate-400">Wheel Sinkage</div>
            <div
              data-testid="bekker-sinkage-readout"
              className="text-base font-mono font-bold tabular-nums text-slate-100 mt-0.5"
            >
              {bekkerResult.sinkageMm.toFixed(1)} mm
            </div>
          </div>

          <div className="bg-slate-950 border border-slate-800 rounded p-2.5">
            <div className="text-xs text-slate-400">Belly Clearance</div>
            <div
              data-testid="bekker-clearance-readout"
              className={`text-base font-mono font-bold tabular-nums mt-0.5 ${
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

          <div className="bg-slate-950 border border-slate-800 rounded p-2.5">
            <div className="text-xs text-slate-400">Mud Drag</div>
            <div className="text-base font-mono font-bold tabular-nums text-slate-100 mt-0.5">
              {bekkerResult.mudDragForceN.toFixed(1)} N
            </div>
          </div>

          <div
            data-testid="bekker-verdict-badge"
            className={`rounded p-2.5 border flex items-center gap-2 ${
              bekkerResult.verdictCode === 'TRAVERSABLE'
                ? 'bg-emerald-950/40 border-emerald-500/40 text-emerald-300'
                : bekkerResult.verdictCode === 'MARGINAL_SLIP'
                  ? 'bg-amber-950/40 border-amber-500/40 text-amber-300'
                  : 'bg-red-950/50 border-red-500/50 text-red-300'
            }`}
          >
            {bekkerResult.verdictCode === 'TRAVERSABLE' && <CheckCircle2 className="w-4 h-4 shrink-0" />}
            {bekkerResult.verdictCode === 'MARGINAL_SLIP' && <AlertTriangle className="w-4 h-4 shrink-0" />}
            {bekkerResult.verdictCode === 'BELLY_OUT_STUCK' && <ShieldAlert className="w-4 h-4 shrink-0" />}
            <div className="min-w-0">
              <div className="text-[10px] uppercase opacity-80">Status</div>
              <div className="text-xs font-bold truncate">
                {bekkerResult.verdictLabel}
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Right 5 Cols: Peer-to-Peer Radio Comms & Snapshot Export */}
      <div className="lg:col-span-5 bg-slate-900 border border-slate-800 rounded-lg p-4 space-y-4">
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center gap-2">
            <Radio className="w-4 h-4 text-emerald-400" />
            <h2 className="text-sm font-semibold text-slate-100">
              Peer-to-Peer Radio Log
            </h2>
          </div>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('BekkerTerramechanicsLab.onExportDashboardJson', {});
              onExportDashboardJson();
            }}
            data-testid="export-dashboard-json-btn"
            className="px-2.5 py-1 rounded text-xs text-cyan-300 hover:text-cyan-200 bg-cyan-950/50 hover:bg-cyan-900/60 border border-cyan-500/40 flex items-center gap-1.5 whitespace-nowrap shrink-0 cursor-pointer"
          >
            <Download className="w-3.5 h-3.5" />
            Export JSON
          </button>
        </div>

        <div className="flex items-center gap-1.5">
          {(
            [
              { id: 'S1_MUD_RESCUE', label: 'All Messages' },
              { id: 'NORMAL_P2P', label: 'Delivered' },
              { id: 'BLACKOUT_WINDOW', label: 'Blackout / Alerts' },
            ] as const
          ).map((mode) => (
            <button
              key={mode.id}
              type="button"
              onClick={() => setCommsScenario(mode.id)}
              className={`px-2.5 py-1 rounded text-xs whitespace-nowrap shrink-0 cursor-pointer border ${
                commsScenario === mode.id
                  ? 'bg-slate-800 text-slate-100 border-slate-700 font-medium'
                  : 'bg-slate-950 text-slate-400 border-slate-800 hover:text-slate-200'
              }`}
            >
              {mode.label}
            </button>
          ))}
        </div>

        <div className="space-y-2 max-h-60 overflow-y-auto pr-1">
          {filteredGossipLogs.map((pkt) => (
            <div
              key={pkt.id}
              className="bg-slate-950 border border-slate-800 rounded p-2.5 text-xs"
            >
              <div className="flex items-center justify-between gap-2 font-mono">
                <span className="text-slate-200 font-semibold">
                  {pkt.senderId} → {pkt.receiverId}
                </span>
                <span
                  className={
                    pkt.status === 'DELIVERED' ? 'text-emerald-400' : 'text-amber-400'
                  }
                >
                  {pkt.status === 'DELIVERED' ? '● Delivered' : '▲ Dropped'} ({pkt.distanceM.toFixed(1)}m)
                </span>
              </div>
              <p className="text-slate-400 mt-1">
                {pkt.summary}
              </p>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
