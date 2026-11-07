/**
 * @file src/components/FieldMapCanvas.tsx
 * @description Clean, User-Friendly Interactive Field Map for the Antenna Multi-Robot Dashboard.
 *
 * Displays:
 * - Color-coded robot dots (`R1`–`R5`) showing where each robot is and its status at a glance.
 * - Dashed plan lines showing where each robot is heading (e.g., Robot 1 retrieving Robot 3).
 * - Crop gap sites (`C-01`–`C-08`) and mud hazard zones.
 * - A single streamlined toolbar with Play/Pause, Step, `+ Obstacle`, Reset, and a compact
 *   dropdown for optional heatmap overlays (so the user isn't overwhelmed by dozens of buttons).
 */

import React, { useMemo, useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import { evaluateContinuousBeliefField } from '../logic/swarmEngine';
import {
  BeliefPerspective,
  ContinuousHeatmapLayer,
  CropGapSite,
  DynamicObstacle,
  FieldHazardZone,
  MetricCoordinate,
  RobotState,
  TaskMode,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface FieldMapCanvasProps {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
  hazardZones: FieldHazardZone[];
  obstacles: DynamicObstacle[];
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  onDispatchRobotToPoint: (
    robotId: string,
    taskMode: TaskMode,
    coord: MetricCoordinate,
    options?: { targetPeerId?: string | null; targetSiteId?: string | null }
  ) => void;
  onInjectDynamicObstacle: () => void;
  simulationRunning: boolean;
  onToggleSimulation: () => void;
  onStepSimulationOnce: () => void;
  onResetSwarm: () => void;
}

type HeatmapSelectorOption = ContinuousHeatmapLayer | 'none';

/**
 * Maps a normalized [0, 1] heatmap intensity and layer type to an RGBA fill color.
 */
function getHeatmapCellColor(layer: HeatmapSelectorOption, value: number): string {
  if (layer === 'none') return 'transparent';
  const norm =
    layer === 'firmness'
      ? Math.max(0, Math.min(1, value / 45))
      : Math.max(0, Math.min(1, value));

  if (layer === 'traversability_risk' || layer === 'stand_gap') {
    const r = Math.round(60 + 195 * norm);
    const g = Math.round(140 * (1 - norm));
    const b = Math.round(90 * (1 - norm));
    return `rgba(${r}, ${g}, ${b}, ${0.1 + norm * 0.28})`;
  }
  if (layer === 'plant_density' || layer === 'firmness') {
    const r = Math.round(16 + 40 * (1 - norm));
    const g = Math.round(90 + 145 * norm);
    const b = Math.round(80 + 60 * norm);
    return `rgba(${r}, ${g}, ${b}, ${0.08 + norm * 0.25})`;
  }
  const r = Math.round(14 + 30 * (1 - norm));
  const g = Math.round(116 + 110 * norm);
  const b = Math.round(160 + 85 * norm);
  return `rgba(${r}, ${g}, ${b}, ${0.1 + norm * 0.28})`;
}

/**
 * Renders the clean, interactive multi-robot field map.
 */
export const FieldMapCanvas: React.FC<FieldMapCanvasProps> = ({
  robots,
  cropGapSites,
  hazardZones,
  obstacles,
  selectedRobotId,
  onSelectRobot,
  onDispatchRobotToPoint,
  onInjectDynamicObstacle,
  simulationRunning,
  onToggleSimulation,
  onStepSimulationOnce,
  onResetSwarm,
}) => {
  const [heatmapLayer, setHeatmapLayer] = useState<HeatmapSelectorOption>('none');
  const [perspective, setPerspective] = useState<BeliefPerspective>('planter');
  const [clickDispatchEnabled, setClickDispatchEnabled] = useState<boolean>(false);

  const widthM = APP_CONFIG.fieldGeometry.widthMeters;
  const heightM = APP_CONFIG.fieldGeometry.heightMeters;
  const svgWidth = 960;
  const svgHeight = 620;

  /**
   * Converts metric field coordinates (meters) to SVG viewport pixels.
   */
  function metricToPixel(coord: MetricCoordinate): { px: number; py: number } {
    return {
      px: Math.round((coord.x / widthM) * svgWidth * 10) / 10,
      py: Math.round((coord.y / heightM) * svgHeight * 10) / 10,
    };
  }

  /**
   * Precomputes the continuous RBF belief heatmap cells only when an overlay is chosen.
   */
  const heatmapCells = useMemo(() => {
    if (heatmapLayer === 'none') return [];
    const cols = 16;
    const rows = 10;
    const cellW = svgWidth / cols;
    const cellH = svgHeight / rows;
    const cells: { key: string; x: number; y: number; w: number; h: number; fill: string }[] = [];

    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const xm = ((c + 0.5) / cols) * widthM;
        const ym = ((r + 0.5) / rows) * heightM;
        const val = evaluateContinuousBeliefField(xm, ym, heatmapLayer, perspective);
        cells.push({
          key: `hm-${r}-${c}`,
          x: c * cellW,
          y: r * cellH,
          w: cellW,
          h: cellH,
          fill: getHeatmapCellColor(heatmapLayer, val),
        });
      }
    }
    return cells;
  }, [heatmapLayer, perspective, widthM, heightM]);

  /**
   * Handles clicks on the map surface to move the selected robot when Click-to-Move is enabled.
   */
  function handleMapSurfaceClick(event: React.MouseEvent<SVGSVGElement>): void {
    if (!clickDispatchEnabled) return;
    const rect = event.currentTarget.getBoundingClientRect();
    const relX = (event.clientX - rect.left) / rect.width;
    const relY = (event.clientY - rect.top) / rect.height;
    const coord: MetricCoordinate = {
      x: Math.round(relX * widthM * 10) / 10,
      y: Math.round(relY * heightM * 10) / 10,
    };

    logFunctionCall('FieldMapCanvas.handleMapSurfaceClick', {
      selectedRobotId,
      coord,
    });

    onDispatchRobotToPoint(selectedRobotId, 'NAVIGATE', coord);
    setClickDispatchEnabled(false);
  }

  /**
   * Handles clicking a robot dot on the map.
   */
  function handleRobotDotClick(event: React.MouseEvent, robot: RobotState): void {
    event.stopPropagation();
    logFunctionCall('FieldMapCanvas.handleRobotDotClick', {
      clickedRobotId: robot.id,
    });
    onSelectRobot(robot.id);
  }

  /**
   * Handles clicking a crop gap site marker on the map to dispatch the selected robot.
   */
  function handleCropSiteClick(event: React.MouseEvent, site: CropGapSite): void {
    event.stopPropagation();
    logFunctionCall('FieldMapCanvas.handleCropSiteClick', {
      siteId: site.id,
      selectedRobotId,
    });
    const selectedRobot = robots.find((r) => r.id === selectedRobotId);
    if (!selectedRobot) return;

    const targetMode: TaskMode =
      selectedRobot.capabilities.canPlantSeed && selectedRobot.seedHopperCount > 0
        ? 'PLANT'
        : 'SAMPLE';

    onDispatchRobotToPoint(selectedRobotId, targetMode, site.position, {
      targetSiteId: site.id,
    });
  }

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];

  return (
    <section
      aria-label="Flooded Paddy Field Multi-Robot Map"
      className="flex flex-col bg-slate-900 border border-slate-800 rounded-lg overflow-hidden"
    >
      {/* Single Clean Map Toolbar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 bg-slate-950 border-b border-slate-800">
        {/* Left: Map Title + Compact Overlay Selector */}
        <div className="flex flex-wrap items-center gap-3">
          <h2 className="text-sm font-semibold text-slate-100">
            Field Map (24m × 16m)
          </h2>

          <div className="flex items-center gap-1.5 text-xs">
            <label htmlFor="map-overlay-select" className="text-slate-400">
              Overlay:
            </label>
            <select
              id="map-overlay-select"
              value={heatmapLayer}
              onChange={(e) => {
                const nextLayer = e.target.value as HeatmapSelectorOption;
                logFunctionCall('FieldMapCanvas.setHeatmapLayer', { layer: nextLayer });
                setHeatmapLayer(nextLayer);
              }}
              className="px-2 py-1 text-xs bg-slate-900 border border-slate-700 rounded text-slate-200 cursor-pointer"
            >
              <option value="none">Clean Map (Default)</option>
              <option value="traversability_risk">Mud &amp; Slip Risk</option>
              <option value="moisture">Soil Moisture</option>
              <option value="firmness">Soil Firmness</option>
              <option value="stand_gap">Missing Crop Gaps</option>
              <option value="uncertainty">Map Uncertainty</option>
            </select>

            {heatmapLayer !== 'none' && (
              <select
                aria-label="Belief Perspective"
                value={perspective}
                onChange={(e) => setPerspective(e.target.value as BeliefPerspective)}
                className="px-2 py-1 text-xs bg-slate-900 border border-slate-700 rounded text-cyan-300 cursor-pointer"
              >
                <option value="planter">Planter View</option>
                <option value="scout">Scout View</option>
                <option value="divergence">Planter vs Scout Gap</option>
                <option value="truth">Ground Truth</option>
              </select>
            )}
          </div>
        </div>

        {/* Right: Clean Playback & Action Buttons */}
        <div className="flex flex-wrap items-center gap-1.5">
          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onToggleSimulation', { next: !simulationRunning });
              onToggleSimulation();
            }}
            className={`px-3 py-1 text-xs font-semibold rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              simulationRunning
                ? 'bg-amber-500/20 border-amber-500/50 text-amber-300 hover:bg-amber-500/30'
                : 'bg-cyan-600 border-cyan-500 text-white hover:bg-cyan-500'
            }`}
          >
            {simulationRunning ? 'Pause Sim' : 'Play Sim'}
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onStepSimulationOnce', {});
              onStepSimulationOnce();
            }}
            className="px-2.5 py-1 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-200 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Step
          </button>

          <button
            type="button"
            data-testid="inject-obstacle-btn"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onInjectDynamicObstacle', {});
              onInjectDynamicObstacle();
            }}
            className="px-2.5 py-1 text-xs font-medium rounded border border-amber-500/40 bg-amber-500/10 text-amber-300 hover:bg-amber-500/20 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            + Add Obstacle
          </button>

          <button
            type="button"
            onClick={() => setClickDispatchEnabled((v) => !v)}
            className={`px-2.5 py-1 text-xs font-medium rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              clickDispatchEnabled
                ? 'bg-cyan-600 border-cyan-400 text-white'
                : 'border-slate-700 bg-slate-900 text-slate-300 hover:bg-slate-800'
            }`}
          >
            {clickDispatchEnabled ? 'Click Map to Move...' : 'Set Waypoint'}
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onResetSwarm', {});
              onResetSwarm();
            }}
            className="px-2.5 py-1 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-400 hover:text-slate-200 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Reset
          </button>
        </div>
      </div>

      {/* Main SVG Metric Field Map */}
      <div className="relative w-full bg-[#071118] select-none">
        <svg
          data-testid="paddy-field-map-svg"
          viewBox={`0 0 ${svgWidth} ${svgHeight}`}
          onClick={handleMapSurfaceClick}
          className={`w-full h-auto block ${
            clickDispatchEnabled ? 'cursor-crosshair' : 'cursor-pointer'
          }`}
        >
          <defs>
            <pattern
              id="paddyMetricGrid"
              width={svgWidth / 12}
              height={svgHeight / 8}
              patternUnits="userSpaceOnUse"
            >
              <path
                d={`M ${svgWidth / 12} 0 L 0 0 0 ${svgHeight / 8}`}
                fill="none"
                stroke="rgba(148, 163, 184, 0.08)"
                strokeWidth="1"
              />
            </pattern>
            <marker
              id="arrow-cyan"
              viewBox="0 0 10 10"
              refX="7"
              refY="5"
              markerWidth="6"
              markerHeight="6"
              orient="auto-start-reverse"
            >
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#06b6d4" />
            </marker>
            <marker
              id="arrow-emerald"
              viewBox="0 0 10 10"
              refX="7"
              refY="5"
              markerWidth="6"
              markerHeight="6"
              orient="auto-start-reverse"
            >
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#10b981" />
            </marker>
            <marker
              id="arrow-amber"
              viewBox="0 0 10 10"
              refX="7"
              refY="5"
              markerWidth="6"
              markerHeight="6"
              orient="auto-start-reverse"
            >
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#f59e0b" />
            </marker>
          </defs>

          {/* Base Paddy Surface */}
          <rect x="0" y="0" width={svgWidth} height={svgHeight} fill="#07131a" />

          {/* Optional Heatmap Overlay */}
          {heatmapCells.map((cell) => (
            <rect
              key={cell.key}
              x={cell.x}
              y={cell.y}
              width={cell.w}
              height={cell.h}
              fill={cell.fill}
            />
          ))}

          <rect x="0" y="0" width={svgWidth} height={svgHeight} fill="url(#paddyMetricGrid)" />

          {/* Section A / Section B Subtle Divider */}
          <line
            x1={svgWidth / 2}
            y1={0}
            x2={svgWidth / 2}
            y2={svgHeight}
            stroke="rgba(56, 189, 248, 0.2)"
            strokeWidth="1"
            strokeDasharray="6 6"
          />
          <text
            x={svgWidth * 0.25}
            y={svgHeight - 10}
            textAnchor="middle"
            fill="rgba(148, 163, 184, 0.45)"
            fontSize="10"
            fontFamily="JetBrains Mono, monospace"
          >
            SECTION A (WEST)
          </text>
          <text
            x={svgWidth * 0.75}
            y={svgHeight - 10}
            textAnchor="middle"
            fill="rgba(148, 163, 184, 0.45)"
            fontSize="10"
            fontFamily="JetBrains Mono, monospace"
          >
            SECTION B (EAST)
          </text>

          {/* Rice Crop Furrow Lines */}
          {Array.from({ length: 7 }).map((_, idx) => {
            const rowNumber = idx + 1;
            const yMeters = rowNumber * 2;
            const py = (yMeters / heightM) * svgHeight;
            return (
              <g key={`crop-row-${rowNumber}`}>
                <line
                  x1={28}
                  y1={py}
                  x2={svgWidth - 28}
                  y2={py}
                  stroke="rgba(16, 185, 129, 0.14)"
                  strokeWidth="2.5"
                  strokeDasharray="10 6"
                />
                <text
                  x={8}
                  y={py - 5}
                  fill="rgba(148, 163, 184, 0.4)"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                >
                  Row {rowNumber}
                </text>
              </g>
            );
          })}

          {/* Seed Depot Station */}
          {(() => {
            const depot = metricToPixel(APP_CONFIG.fieldGeometry.depotPosition);
            return (
              <g>
                <rect
                  x={depot.px - 22}
                  y={depot.py - 14}
                  width={44}
                  height={28}
                  rx={4}
                  fill="rgba(15, 23, 42, 0.9)"
                  stroke="#38bdf8"
                  strokeWidth="1.5"
                />
                <text
                  x={depot.px}
                  y={depot.py + 3}
                  textAnchor="middle"
                  fill="#bae6fd"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="600"
                >
                  DEPOT
                </text>
              </g>
            );
          })()}

          {/* Hazard Zones */}
          {hazardZones.map((hazard) => {
            const { px, py } = metricToPixel(hazard.position);
            const rPx = (hazard.radiusMeters / widthM) * svgWidth;
            return (
              <g key={hazard.id}>
                <circle
                  cx={px}
                  cy={py}
                  r={rPx}
                  fill="rgba(239, 68, 68, 0.12)"
                  stroke="rgba(248, 113, 113, 0.55)"
                  strokeWidth="1.5"
                  strokeDasharray="5 4"
                />
                <text
                  x={px}
                  y={py + rPx + 13}
                  textAnchor="middle"
                  fill="rgba(252, 165, 165, 0.85)"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                >
                  {hazard.label}
                </text>
              </g>
            );
          })}

          {/* Dynamically Injected Obstacles */}
          {obstacles.map((obs) => {
            const { px, py } = metricToPixel(obs.position);
            const rPx = (obs.radiusMeters / widthM) * svgWidth;
            return (
              <g key={obs.id} data-testid={`dynamic-obstacle-${obs.id}`}>
                <circle
                  cx={px}
                  cy={py}
                  r={rPx}
                  fill="rgba(245, 158, 11, 0.25)"
                  stroke="#fbbf24"
                  strokeWidth="2"
                  strokeDasharray="4 3"
                />
                <text
                  x={px}
                  y={py - rPx - 6}
                  textAnchor="middle"
                  fill="#fde68a"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="700"
                >
                  ▲ {obs.label}
                </text>
              </g>
            );
          })}

          {/* Crop Gap Sites */}
          {cropGapSites.map((site) => {
            const { px, py } = metricToPixel(site.position);
            const isPlanted = site.status === 'PLANTED_VERIFIED';
            const isHazardSkipped = site.status === 'HAZARD_SKIPPED';
            const strokeColor = isPlanted
              ? '#10b981'
              : isHazardSkipped
                ? '#f43f5e'
                : '#fbbf24';
            const fillColor = isPlanted
              ? 'rgba(16, 185, 129, 0.22)'
              : isHazardSkipped
                ? 'rgba(244, 63, 94, 0.22)'
                : 'rgba(251, 191, 36, 0.18)';

            return (
              <g
                key={site.id}
                onClick={(e) => handleCropSiteClick(e, site)}
                className="cursor-pointer"
              >
                <rect
                  x={px - 10}
                  y={py - 10}
                  width={20}
                  height={20}
                  rx={4}
                  fill={fillColor}
                  stroke={strokeColor}
                  strokeWidth="1.5"
                />
                <text
                  x={px}
                  y={py + 3}
                  textAnchor="middle"
                  fill="#f8fafc"
                  fontSize="8.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="700"
                >
                  {site.id}
                </text>
              </g>
            );
          })}

          {/* Robot Plan Trajectories & Rescue Links */}
          {robots.map((robot) => {
            const start = metricToPixel(robot.position);
            const end = metricToPixel(robot.targetWaypoint);
            const dist = Math.hypot(end.px - start.px, end.py - start.py);
            const theme = APP_CONFIG.statusTheme[robot.status];

            const routePoints =
              robot.plannedRoute && robot.plannedRoute.length > 0
                ? [robot.position, ...robot.plannedRoute]
                : [robot.position, robot.targetWaypoint];

            const polylineStr = routePoints
              .map((pt) => {
                const p = metricToPixel(pt);
                return `${p.px},${p.py}`;
              })
              .join(' ');

            const peerRobot =
              robot.taskMode === 'RETRIEVE_PEER' && robot.targetPeerId
                ? robots.find((r) => r.id === robot.targetPeerId)
                : null;

            return (
              <g key={`plan-vec-${robot.id}`}>
                {dist > 8 && (
                  <g>
                    <polyline
                      points={polylineStr}
                      fill="none"
                      stroke={theme.hex}
                      strokeWidth="2.2"
                      strokeDasharray="6 4"
                      markerEnd={
                        robot.status === 'RETRIEVING'
                          ? 'url(#arrow-cyan)'
                          : robot.status === 'WARNING_SLIP'
                            ? 'url(#arrow-amber)'
                            : 'url(#arrow-emerald)'
                      }
                    />
                    <circle
                      cx={end.px}
                      cy={end.py}
                      r={4.5}
                      fill="none"
                      stroke={theme.hex}
                      strokeWidth="1.5"
                    />
                  </g>
                )}

                {peerRobot && (
                  <g>
                    {(() => {
                      const peerPx = metricToPixel(peerRobot.position);
                      const midX = (start.px + peerPx.px) / 2;
                      const midY = (start.py + peerPx.py) / 2;
                      return (
                        <>
                          <line
                            x1={start.px}
                            y1={start.py}
                            x2={peerPx.px}
                            y2={peerPx.py}
                            stroke="#22d3ee"
                            strokeWidth="2.5"
                          />
                          <rect
                            x={midX - 48}
                            y={midY - 20}
                            width={96}
                            height={16}
                            rx={3}
                            fill="#083344"
                            stroke="#06b6d4"
                            strokeWidth="1"
                          />
                          <text
                            x={midX}
                            y={midY - 9}
                            textAnchor="middle"
                            fill="#a5f3fc"
                            fontSize="9"
                            fontFamily="JetBrains Mono, monospace"
                            fontWeight="600"
                          >
                            RESCUE LINK
                          </text>
                        </>
                      );
                    })()}
                  </g>
                )}
              </g>
            );
          })}

          {/* Color-Coded Robot Dots */}
          {robots.map((robot) => {
            const { px, py } = metricToPixel(robot.position);
            const theme = APP_CONFIG.statusTheme[robot.status];
            const isSelected = robot.id === selectedRobotId;
            const headingRad = (robot.headingDeg * Math.PI) / 180;
            const dirX = px + Math.cos(headingRad) * 20;
            const dirY = py + Math.sin(headingRad) * 20;
            const shortNum = robot.id.replace('ANT-0', 'R');

            return (
              <g
                key={robot.id}
                onClick={(e) => handleRobotDotClick(e, robot)}
                className="cursor-pointer"
              >
                {isSelected && (
                  <circle
                    cx={px}
                    cy={py}
                    r={25}
                    fill="none"
                    stroke="#f8fafc"
                    strokeWidth="2"
                    strokeDasharray="4 3"
                  />
                )}

                <circle
                  cx={px}
                  cy={py}
                  r={19}
                  fill={theme.ringHex}
                  stroke={theme.hex}
                  strokeWidth={isSelected ? '2.5' : '1.5'}
                />

                <line
                  x1={px}
                  y1={py}
                  x2={dirX}
                  y2={dirY}
                  stroke={theme.hex}
                  strokeWidth="2.5"
                />

                <circle
                  cx={px}
                  cy={py}
                  r={11}
                  fill={theme.hex}
                  stroke="#020617"
                  strokeWidth="2"
                />

                <text
                  x={px}
                  y={py + 3.5}
                  textAnchor="middle"
                  fill="#020617"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="800"
                >
                  {shortNum}
                </text>

                {/* Clean Floating Label */}
                <g transform={`translate(${px}, ${py - 26})`}>
                  <rect
                    x={-52}
                    y={-16}
                    width={104}
                    height={18}
                    rx={3}
                    fill="rgba(2, 6, 23, 0.9)"
                    stroke={theme.hex}
                    strokeWidth="1"
                  />
                  <text
                    x={0}
                    y={-4}
                    textAnchor="middle"
                    fill="#f8fafc"
                    fontSize="9.5"
                    fontFamily="JetBrains Mono, monospace"
                    fontWeight="600"
                  >
                    {theme.glyph} Robot {robot.id.replace('ANT-0', '')}
                  </text>
                </g>
              </g>
            );
          })}
        </svg>

        {/* Clean Bottom Legend & Selected Robot Plan Bar */}
        <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 bg-slate-950 border-t border-slate-800 text-xs">
          {/* Status Color Key */}
          <div className="flex flex-wrap items-center gap-3" aria-label="Robot Status Color Legend">
            {Object.values(APP_CONFIG.statusTheme).map((theme) => (
              <span key={theme.code} className={`inline-flex items-center gap-1 font-medium ${theme.textClass}`}>
                <span aria-hidden="true">{theme.glyph}</span>
                <span>{theme.shortLabel}</span>
              </span>
            ))}
          </div>

          {/* Selected Robot Current Plan */}
          <div className="flex items-center gap-2 text-slate-300 truncate max-w-md">
            <span className="font-mono font-semibold text-cyan-400 shrink-0">
              {selectedRobot.callsign.split('·')[0].trim()}:
            </span>
            <span className="truncate">{selectedRobot.planSummary}</span>
          </div>
        </div>
      </div>
    </section>
  );
};
