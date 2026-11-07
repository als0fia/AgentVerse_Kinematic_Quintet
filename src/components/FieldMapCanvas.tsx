/**
 * @file src/components/FieldMapCanvas.tsx
 * @description Interactive 2D Flooded Paddy Field Map (`antenna.dashboard.v1` + `D-0012`) for the Antenna Multi-Robot System.
 *
 * This component renders the metric field coordinate canvas ($24.0\text{ m} \times 16.0\text{ m}$) displaying:
 * - Each homogeneous Medic-Ant robot (`ANT-01` through `ANT-05`) as a high-contrast color-coded dot
 *   with heading indicator, 3-ray LiDAR cone, callsign badge, and multi-waypoint A* planned trajectory (`plannedRoute`).
 * - Continuous 2D Gaussian-RBF Belief Heatmap overlay (`Moisture M(x,y)`, `Firmness F(x,y)`, `Plant Density ρ(x,y)`,
 *   `Stand Gap G(x,y)`, `Epistemic Uncertainty σ(x,y)`, `Mud/SLAM Risk R(x,y)`, `Elevation z(x,y)`) across
 *   4 selectable epistemic perspectives (`Planter Belief`, `Scout Belief`, `|Planter − Scout| Gap`, `Hidden Ground Truth`).
 * - Two-Section Staggered Paddy Plot divider (`Section A: Batch 1 Day -25` vs `Section B: Batch 2 +20d Offset`).
 * - Active Collinear Rigid Docking, Cam-Latch & Pitch-Lift links connecting rescuing rovers (`ANT-01`)
 *   to immobilized peers (`ANT-03`) with live pitch-lift angle (`rad`) and cam-latch state.
 * - `D-0012` Interactive Mid-Run Dynamic Obstacle Injection (`Inject Obstacle` button) that inserts an
 *   unexpected obstacle on an active route and triggers instant multi-waypoint A* detour replanning.
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

const PERSPECTIVE_OPTIONS: { id: BeliefPerspective; label: string }[] = [
  { id: 'planter', label: 'Planter Belief' },
  { id: 'scout', label: 'Scout Belief' },
  { id: 'divergence', label: '|Planter − Scout| Gap' },
  { id: 'truth', label: 'Ground Truth (Ref)' },
];

type HeatmapSelectorOption = ContinuousHeatmapLayer | 'none';

const HEATMAP_LAYERS: { id: HeatmapSelectorOption; label: string; unit: string }[] = [
  { id: 'moisture', label: 'Moisture M(x,y)', unit: '' },
  { id: 'firmness', label: 'Firmness F(x,y)', unit: 'kPa' },
  { id: 'plant_density', label: 'Density ρ(x,y)', unit: '' },
  { id: 'stand_gap', label: 'Stand Gap G(x,y)', unit: '' },
  { id: 'uncertainty', label: 'Uncertainty σ(x,y)', unit: '' },
  { id: 'traversability_risk', label: 'Mud Risk R(x,y)', unit: '' },
  { id: 'elevation_m', label: 'Elevation z(x,y)', unit: 'm' },
  { id: 'none', label: 'Heatmap Off', unit: '' },
];

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
    return `rgba(${r}, ${g}, ${b}, ${0.12 + norm * 0.34})`;
  }
  if (layer === 'plant_density' || layer === 'firmness') {
    const r = Math.round(16 + 40 * (1 - norm));
    const g = Math.round(90 + 145 * norm);
    const b = Math.round(80 + 60 * norm);
    return `rgba(${r}, ${g}, ${b}, ${0.1 + norm * 0.3})`;
  }
  if (layer === 'uncertainty') {
    const r = Math.round(140 + 105 * norm);
    const g = Math.round(80 + 80 * norm);
    const b = Math.round(210);
    return `rgba(${r}, ${g}, ${b}, ${0.1 + norm * 0.32})`;
  }
  // Default cyan/blue gradient for Moisture & Elevation
  const r = Math.round(14 + 30 * (1 - norm));
  const g = Math.round(116 + 110 * norm);
  const b = Math.round(160 + 85 * norm);
  return `rgba(${r}, ${g}, ${b}, ${0.12 + norm * 0.32})`;
}

/**
 * Renders the interactive paddy field map with continuous 2D RBF heatmaps, color-coded robot dots,
 * multi-waypoint A* routes, rigid docking indicators, and D-0012 obstacle injection.
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
  const [showPlanVectors, setShowPlanVectors] = useState<boolean>(true);
  const [showHazardZones, setShowHazardZones] = useState<boolean>(true);
  const [showBreadcrumbs, setShowBreadcrumbs] = useState<boolean>(true);
  const [perspective, setPerspective] = useState<BeliefPerspective>('planter');
  const [heatmapLayer, setHeatmapLayer] = useState<HeatmapSelectorOption>('traversability_risk');
  const [mapClickMode, setMapClickMode] = useState<
    'INSPECT' | 'DISPATCH_NAVIGATE' | 'DISPATCH_PLANT' | 'DISPATCH_SAMPLE'
  >('INSPECT');
  const [hoverCoord, setHoverCoord] = useState<MetricCoordinate | null>(null);

  const widthM = APP_CONFIG.fieldGeometry.widthMeters;
  const heightM = APP_CONFIG.fieldGeometry.heightMeters;
  const svgWidth = 960;
  const svgHeight = 640;

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
   * Precomputes the 16x10 continuous RBF belief heatmap cells when a layer is active.
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
   * Handles clicks on the SVG background field to dispatch the currently selected robot when in a dispatch mode.
   */
  function handleMapSurfaceClick(event: React.MouseEvent<SVGSVGElement>): void {
    const rect = event.currentTarget.getBoundingClientRect();
    const relX = (event.clientX - rect.left) / rect.width;
    const relY = (event.clientY - rect.top) / rect.height;
    const coord: MetricCoordinate = {
      x: Math.round(relX * widthM * 10) / 10,
      y: Math.round(relY * heightM * 10) / 10,
    };

    logFunctionCall('FieldMapCanvas.handleMapSurfaceClick', {
      mapClickMode,
      selectedRobotId,
      coord,
    });

    if (mapClickMode === 'DISPATCH_NAVIGATE') {
      onDispatchRobotToPoint(selectedRobotId, 'NAVIGATE', coord);
      setMapClickMode('INSPECT');
    } else if (mapClickMode === 'DISPATCH_PLANT') {
      onDispatchRobotToPoint(selectedRobotId, 'PLANT', coord);
      setMapClickMode('INSPECT');
    } else if (mapClickMode === 'DISPATCH_SAMPLE') {
      onDispatchRobotToPoint(selectedRobotId, 'SAMPLE', coord);
      setMapClickMode('INSPECT');
    }
  }

  /**
   * Updates the hovered coordinate readout in the HUD corner when moving the cursor over the map.
   */
  function handleMapMouseMove(event: React.MouseEvent<SVGSVGElement>): void {
    const rect = event.currentTarget.getBoundingClientRect();
    const relX = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
    const relY = Math.max(0, Math.min(1, (event.clientY - rect.top) / rect.height));
    setHoverCoord({
      x: Math.round(relX * widthM * 10) / 10,
      y: Math.round(relY * heightM * 10) / 10,
    });
  }

  /**
   * Handles clicking a specific robot dot on the map.
   */
  function handleRobotDotClick(event: React.MouseEvent, robot: RobotState): void {
    event.stopPropagation();
    logFunctionCall('FieldMapCanvas.handleRobotDotClick', {
      clickedRobotId: robot.id,
      currentSelectedId: selectedRobotId,
    });
    onSelectRobot(robot.id);
  }

  /**
   * Handles clicking a missing crop gap site marker on the map.
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
  const hoveredFieldValue =
    hoverCoord && heatmapLayer !== 'none'
      ? evaluateContinuousBeliefField(hoverCoord.x, hoverCoord.y, heatmapLayer, perspective)
      : null;

  return (
    <section
      aria-label="Flooded Paddy Field Multi-Robot Map"
      className="flex flex-col bg-slate-900 border border-slate-800 rounded-lg overflow-hidden"
    >
      {/* Map Control Toolbar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div>
          <h2 className="text-sm font-semibold text-slate-100">
            Terrace Sector B-04 · Homogeneous Medic-Ant Swarm Map (24.0m × 16.0m)
          </h2>
          <p className="text-xs text-slate-400">
            Click any robot dot to inspect · Click crop gap site (C-01–C-08) to dispatch{' '}
            {selectedRobot.callsign.split('·')[0].trim()}
          </p>
        </div>

        {/* Simulation Playback + D-0012 Obstacle Injection Controls */}
        <div className="flex flex-wrap items-center gap-2">
          <button
            type="button"
            data-testid="inject-obstacle-btn"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onInjectDynamicObstacle', {});
              onInjectDynamicObstacle();
            }}
            className="px-3 py-1.5 text-xs font-semibold rounded border border-amber-500/50 bg-amber-500/15 text-amber-300 hover:bg-amber-500/25 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            + Inject Obstacle (D-0012)
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onToggleSimulation', { next: !simulationRunning });
              onToggleSimulation();
            }}
            className={`px-3 py-1.5 text-xs font-medium rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              simulationRunning
                ? 'bg-amber-500/15 border-amber-500/40 text-amber-300 hover:bg-amber-500/25'
                : 'bg-cyan-500/15 border-cyan-500/40 text-cyan-300 hover:bg-cyan-500/25'
            }`}
          >
            {simulationRunning ? 'Pause Sim Loop' : 'Run Sim Loop'}
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onStepSimulationOnce', {});
              onStepSimulationOnce();
            }}
            className="px-3 py-1.5 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-200 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Step +1 Tick
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onResetSwarm', {});
              onResetSwarm();
            }}
            className="px-3 py-1.5 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-300 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            Reset Baseline
          </button>
        </div>
      </div>

      {/* Perspective Switcher & Continuous 2D RBF Heatmap Layer Selector (`antenna.dashboard.v1`) */}
      <div className="px-4 py-2 bg-slate-950/80 border-b border-slate-800 flex flex-wrap items-center justify-between gap-2 text-xs">
        {/* Perspective Switcher */}
        <div className="flex flex-wrap items-center gap-1" role="group" aria-label="Belief Perspective Switcher">
          <span className="text-[11px] font-mono text-slate-400 mr-1">Perspective:</span>
          {PERSPECTIVE_OPTIONS.map((opt) => (
            <button
              key={opt.id}
              type="button"
              onClick={() => {
                logFunctionCall('FieldMapCanvas.setPerspective', { perspective: opt.id });
                setPerspective(opt.id);
              }}
              className={`px-2 py-1 text-[11px] font-mono rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                perspective === opt.id
                  ? 'bg-cyan-950/70 border-cyan-500/50 text-cyan-200 font-semibold'
                  : 'bg-slate-900 border-slate-800 text-slate-400 hover:text-slate-200'
              }`}
            >
              {opt.label}
            </button>
          ))}
        </div>

        {/* Continuous 2D RBF Heatmap Layer Selector */}
        <div className="flex flex-wrap items-center gap-1" role="group" aria-label="Continuous 2D RBF Heatmap Layer">
          <span className="text-[11px] font-mono text-slate-400 mr-1">RBF Field:</span>
          {HEATMAP_LAYERS.map((layer) => (
            <button
              key={layer.id}
              type="button"
              onClick={() => {
                logFunctionCall('FieldMapCanvas.setHeatmapLayer', { layer: layer.id });
                setHeatmapLayer(layer.id);
              }}
              className={`px-2 py-1 text-[11px] font-mono rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                heatmapLayer === layer.id
                  ? 'bg-emerald-950/70 border-emerald-500/50 text-emerald-200 font-semibold'
                  : 'bg-slate-900 border-slate-800 text-slate-400 hover:text-slate-200'
              }`}
            >
              {layer.label}
            </button>
          ))}
        </div>
      </div>

      {/* Color-Coded Status Legend & Map Click Dispatch Mode Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2 bg-slate-900/90 border-b border-slate-800 text-xs">
        {/* Status Color Legend (Non-hue-only: Glyph + Text + Color) */}
        <div className="flex flex-wrap items-center gap-3.5" aria-label="Robot Status Color Legend">
          <span className="text-slate-400 font-medium">Status Legend:</span>
          {Object.values(APP_CONFIG.statusTheme).map((theme) => (
            <span key={theme.code} className={`inline-flex items-center gap-1.5 font-mono ${theme.textClass}`}>
              <span aria-hidden="true">{theme.glyph}</span>
              <span>{theme.shortLabel}</span>
            </span>
          ))}
        </div>

        {/* Layer Toggles & Map Click Mode */}
        <div className="flex flex-wrap items-center gap-1.5">
          <div className="flex items-center gap-1 p-0.5 bg-slate-950 border border-slate-800 rounded">
            <button
              type="button"
              onClick={() => setMapClickMode('INSPECT')}
              className={`px-2 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                mapClickMode === 'INSPECT'
                  ? 'bg-slate-800 text-slate-100 font-medium'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Select Mode
            </button>
            <button
              type="button"
              onClick={() => setMapClickMode('DISPATCH_NAVIGATE')}
              className={`px-2 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                mapClickMode === 'DISPATCH_NAVIGATE'
                  ? 'bg-cyan-600 text-white font-medium'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Click to Navigate
            </button>
            <button
              type="button"
              onClick={() => setMapClickMode('DISPATCH_PLANT')}
              className={`px-2 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                mapClickMode === 'DISPATCH_PLANT'
                  ? 'bg-emerald-600 text-white font-medium'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Click to Plant
            </button>
            <button
              type="button"
              onClick={() => setMapClickMode('DISPATCH_SAMPLE')}
              className={`px-2 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
                mapClickMode === 'DISPATCH_SAMPLE'
                  ? 'bg-amber-600 text-white font-medium'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Click to Sample
            </button>
          </div>

          <button
            type="button"
            onClick={() => setShowPlanVectors((v) => !v)}
            className={`px-2 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              showPlanVectors
                ? 'border-cyan-500/40 bg-cyan-500/10 text-cyan-300'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Routes
          </button>

          <button
            type="button"
            onClick={() => setShowHazardZones((v) => !v)}
            className={`px-2 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              showHazardZones
                ? 'border-rose-500/40 bg-rose-500/10 text-rose-300'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Hazards
          </button>

          <button
            type="button"
            onClick={() => setShowBreadcrumbs((v) => !v)}
            className={`px-2 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 cursor-pointer ${
              showBreadcrumbs
                ? 'border-slate-600 bg-slate-800 text-slate-200'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Trails
          </button>
        </div>
      </div>

      {/* Main SVG Metric Field Map */}
      <div className="relative w-full bg-[#071118] select-none">
        <svg
          data-testid="paddy-field-map-svg"
          viewBox={`0 0 ${svgWidth} ${svgHeight}`}
          onClick={handleMapSurfaceClick}
          onMouseMove={handleMapMouseMove}
          onMouseLeave={() => setHoverCoord(null)}
          className={`w-full h-auto block ${
            mapClickMode === 'INSPECT' ? 'cursor-pointer' : 'cursor-crosshair'
          }`}
        >
          <defs>
            {/* Subtle metric grid pattern (2m x 2m cells) */}
            <pattern
              id="paddyMetricGrid"
              width={svgWidth / 12}
              height={svgHeight / 8}
              patternUnits="userSpaceOnUse"
            >
              <path
                d={`M ${svgWidth / 12} 0 L 0 0 0 ${svgHeight / 8}`}
                fill="none"
                stroke="rgba(148, 163, 184, 0.10)"
                strokeWidth="1"
              />
            </pattern>
            {/* Arrowhead markers for robot plan vectors */}
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

          {/* Flooded paddy water & soil base rect */}
          <rect x="0" y="0" width={svgWidth} height={svgHeight} fill="#07131a" />

          {/* Continuous 2D Gaussian-RBF Belief Heatmap Layer */}
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

          {/* Two-Section Staggered Paddy Divider at x = 12.0m (Section A vs Section B) */}
          <line
            x1={svgWidth / 2}
            y1={0}
            x2={svgWidth / 2}
            y2={svgHeight}
            stroke="rgba(56, 189, 248, 0.35)"
            strokeWidth="1.5"
            strokeDasharray="8 6"
          />
          <text
            x={svgWidth * 0.25}
            y={svgHeight - 10}
            textAnchor="middle"
            fill="rgba(148, 163, 184, 0.65)"
            fontSize="10"
            fontFamily="JetBrains Mono, monospace"
          >
            SECTION A · BATCH 1 (PLANTED DAY -25 · ESTABLISHED STAND)
          </text>
          <text
            x={svgWidth * 0.75}
            y={svgHeight - 10}
            textAnchor="middle"
            fill="rgba(56, 189, 248, 0.75)"
            fontSize="10"
            fontFamily="JetBrains Mono, monospace"
          >
            SECTION B · BATCH 2 (+20d OFFSET · ACTIVE GAP INFILL)
          </text>

          {/* Metric Ruler Ticks along Top & Left */}
          {Array.from({ length: 12 }).map((_, idx) => {
            const meters = (idx + 1) * 2;
            const px = ((idx + 1) / 12) * svgWidth;
            return (
              <text
                key={`x-tick-${meters}`}
                x={px - 18}
                y={16}
                fill="rgba(148, 163, 184, 0.45)"
                fontSize="10"
                fontFamily="JetBrains Mono, monospace"
              >
                {meters}m
              </text>
            );
          })}
          {Array.from({ length: 7 }).map((_, idx) => {
            const rowNumber = idx + 1;
            const yMeters = rowNumber * 2;
            const py = (yMeters / heightM) * svgHeight;
            return (
              <g key={`crop-row-${rowNumber}`}>
                <line
                  x1={32}
                  y1={py}
                  x2={svgWidth - 32}
                  y2={py}
                  stroke="rgba(16, 185, 129, 0.18)"
                  strokeWidth="3"
                  strokeDasharray="10 6"
                />
                <text
                  x={8}
                  y={py - 6}
                  fill="rgba(148, 163, 184, 0.5)"
                  fontSize="10"
                  fontFamily="JetBrains Mono, monospace"
                >
                  R{rowNumber} ({yMeters}m)
                </text>
              </g>
            );
          })}

          {/* Seed Hopper Refill Depot & UWB Master Relay Station */}
          {(() => {
            const depot = metricToPixel(APP_CONFIG.fieldGeometry.depotPosition);
            const relay = metricToPixel(APP_CONFIG.fieldGeometry.relayTowerPosition);
            return (
              <g>
                <rect
                  x={depot.px - 22}
                  y={depot.py - 16}
                  width={44}
                  height={32}
                  rx={4}
                  fill="rgba(15, 23, 42, 0.9)"
                  stroke="#38bdf8"
                  strokeWidth="1.5"
                />
                <text
                  x={depot.px}
                  y={depot.py + 4}
                  textAnchor="middle"
                  fill="#bae6fd"
                  fontSize="10"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="600"
                >
                  DEPOT
                </text>

                <rect
                  x={relay.px - 26}
                  y={relay.py - 16}
                  width={52}
                  height={32}
                  rx={4}
                  fill="rgba(15, 23, 42, 0.9)"
                  stroke="#a855f7"
                  strokeWidth="1.5"
                />
                <text
                  x={relay.px}
                  y={relay.py + 4}
                  textAnchor="middle"
                  fill="#e9d5ff"
                  fontSize="10"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="600"
                >
                  RTK-BASE
                </text>
              </g>
            );
          })()}

          {/* Hazard Zones (S1 Mud Sinkhole, S3 Bund, S4 Trench) */}
          {showHazardZones &&
            hazardZones.map((hazard) => {
              const { px, py } = metricToPixel(hazard.position);
              const rPx = (hazard.radiusMeters / widthM) * svgWidth;
              return (
                <g key={hazard.id}>
                  <circle
                    cx={px}
                    cy={py}
                    r={rPx}
                    fill="rgba(239, 68, 68, 0.14)"
                    stroke="rgba(248, 113, 113, 0.65)"
                    strokeWidth="1.5"
                    strokeDasharray="5 4"
                  />
                  <rect
                    x={px - 76}
                    y={py + rPx + 4}
                    width={152}
                    height={15}
                    rx={3}
                    fill="rgba(2, 6, 23, 0.85)"
                  />
                  <text
                    x={px}
                    y={py + rPx + 14}
                    textAnchor="middle"
                    fill="#fca5a5"
                    fontSize="9.5"
                    fontFamily="JetBrains Mono, monospace"
                    fontWeight="600"
                  >
                    {hazard.label}
                  </text>
                </g>
              );
            })}

          {/* Dynamically Injected Mid-Run Obstacles (D-0012) */}
          {obstacles.map((obs) => {
            const { px, py } = metricToPixel(obs.position);
            const rPx = (obs.radiusMeters / widthM) * svgWidth;
            return (
              <g key={obs.id} data-testid={`dynamic-obstacle-${obs.id}`}>
                <circle
                  cx={px}
                  cy={py}
                  r={rPx}
                  fill="rgba(245, 158, 11, 0.28)"
                  stroke="#fbbf24"
                  strokeWidth="2.2"
                  strokeDasharray="4 3"
                />
                <rect
                  x={px - 68}
                  y={py - rPx - 20}
                  width={136}
                  height={16}
                  rx={3}
                  fill="rgba(120, 53, 15, 0.95)"
                  stroke="#fbbf24"
                  strokeWidth="1"
                />
                <text
                  x={px}
                  y={py - rPx - 9}
                  textAnchor="middle"
                  fill="#fef3c7"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="700"
                >
                  ▲ {obs.label}
                </text>
              </g>
            );
          })}

          {/* Crop Gap Sites (Missing Seedlings vs Verified Planted) */}
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
              ? 'rgba(16, 185, 129, 0.25)'
              : isHazardSkipped
                ? 'rgba(244, 63, 94, 0.25)'
                : 'rgba(251, 191, 36, 0.2)';

            return (
              <g
                key={site.id}
                onClick={(e) => handleCropSiteClick(e, site)}
                className="cursor-pointer"
              >
                <rect
                  x={px - 11}
                  y={py - 11}
                  width={22}
                  height={22}
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
                  fontSize="9"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="700"
                >
                  {site.id}
                </text>
                <text
                  x={px}
                  y={py + 23}
                  textAnchor="middle"
                  fill={isPlanted ? '#6ee7b7' : '#fcd34d'}
                  fontSize="9"
                  fontFamily="JetBrains Mono, monospace"
                >
                  {isPlanted ? 'PLANTED' : `${site.soilFirmnessKpa}kPa`}
                </text>
              </g>
            );
          })}

          {/* Robot Breadcrumb Trails */}
          {showBreadcrumbs &&
            robots.map((robot) => {
              if (robot.breadcrumbs.length < 2) return null;
              const pointsStr = robot.breadcrumbs
                .map((pt) => {
                  const { px, py } = metricToPixel(pt);
                  return `${px},${py}`;
                })
                .join(' ');
              const theme = APP_CONFIG.statusTheme[robot.status];
              return (
                <polyline
                  key={`crumb-${robot.id}`}
                  points={pointsStr}
                  fill="none"
                  stroke={theme.hex}
                  strokeWidth="2"
                  strokeOpacity="0.35"
                  strokeDasharray="2 4"
                />
              );
            })}

          {/* Robot Multi-Waypoint A* Planned Routes & Active Rigid Docking Links */}
          {showPlanVectors &&
            robots.map((robot) => {
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
                      {/* Intermediate A* Detour Waypoint Nodes */}
                      {robot.plannedRoute &&
                        robot.plannedRoute.length > 1 &&
                        robot.plannedRoute.slice(0, -1).map((wp, idx) => {
                          const wpPx = metricToPixel(wp);
                          return (
                            <circle
                              key={`wp-${robot.id}-${idx}`}
                              cx={wpPx.px}
                              cy={wpPx.py}
                              r={3.5}
                              fill="#fbbf24"
                              stroke="#020617"
                              strokeWidth="1"
                            />
                          );
                        })}
                      <circle
                        cx={end.px}
                        cy={end.py}
                        r={5}
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
                        const { rigidDock } = robot.telemetry;
                        return (
                          <>
                            <line
                              x1={start.px}
                              y1={start.py}
                              x2={peerPx.px}
                              y2={peerPx.py}
                              stroke="#22d3ee"
                              strokeWidth="3"
                            />
                            <rect
                              x={midX - 66}
                              y={midY - 22}
                              width={132}
                              height={18}
                              rx={3}
                              fill="#083344"
                              stroke="#06b6d4"
                              strokeWidth="1"
                            />
                            <text
                              x={midX}
                              y={midY - 10}
                              textAnchor="middle"
                              fill="#a5f3fc"
                              fontSize="9.5"
                              fontFamily="JetBrains Mono, monospace"
                              fontWeight="600"
                            >
                              {rigidDock.isRigidLatched ? 'DOCK LATCHED' : 'RIGID DOCK'} · +
                              {rigidDock.dockPitchRad.toFixed(2)}rad
                            </text>
                          </>
                        );
                      })()}
                    </g>
                  )}
                </g>
              );
            })}

          {/* Robot Position Dots (Color-Coded Status + Selection Ring + Plan Label) */}
          {robots.map((robot) => {
            const { px, py } = metricToPixel(robot.position);
            const theme = APP_CONFIG.statusTheme[robot.status];
            const isSelected = robot.id === selectedRobotId;
            const headingRad = (robot.headingDeg * Math.PI) / 180;
            const dirX = px + Math.cos(headingRad) * 20;
            const dirY = py + Math.sin(headingRad) * 20;

            return (
              <g
                key={robot.id}
                onClick={(e) => handleRobotDotClick(e, robot)}
                className="cursor-pointer"
              >
                {/* Selection Outer Halo */}
                {isSelected && (
                  <circle
                    cx={px}
                    cy={py}
                    r={26}
                    fill="none"
                    stroke="#f8fafc"
                    strokeWidth="2"
                    strokeDasharray="4 3"
                  />
                )}

                {/* Status Halo Ring */}
                <circle
                  cx={px}
                  cy={py}
                  r={20}
                  fill={theme.ringHex}
                  stroke={theme.hex}
                  strokeWidth={isSelected ? '2.5' : '1.5'}
                />

                {/* Heading Vector Indicator */}
                <line
                  x1={px}
                  y1={py}
                  x2={dirX}
                  y2={dirY}
                  stroke={theme.hex}
                  strokeWidth="2.5"
                />

                {/* Core Color-Coded Robot Dot */}
                <circle
                  cx={px}
                  cy={py}
                  r={11}
                  fill={theme.hex}
                  stroke="#020617"
                  strokeWidth="2"
                />

                {/* Robot Numeric Callsign Inside Dot */}
                <text
                  x={px}
                  y={py + 3.5}
                  textAnchor="middle"
                  fill="#020617"
                  fontSize="9.5"
                  fontFamily="JetBrains Mono, monospace"
                  fontWeight="800"
                >
                  {robot.id.replace('ANT-0', 'R')}
                </text>

                {/* Floating Callsign + Status + Short Plan Tag */}
                <g transform={`translate(${px}, ${py - 28})`}>
                  <rect
                    x={-68}
                    y={-18}
                    width={136}
                    height={20}
                    rx={3}
                    fill="rgba(2, 6, 23, 0.92)"
                    stroke={theme.hex}
                    strokeWidth="1"
                  />
                  <text
                    x={0}
                    y={-5}
                    textAnchor="middle"
                    fill="#f8fafc"
                    fontSize="10"
                    fontFamily="JetBrains Mono, monospace"
                    fontWeight="600"
                  >
                    {theme.glyph} {robot.id} · {robot.taskMode}
                  </text>
                </g>
              </g>
            );
          })}
        </svg>

        {/* Bottom Map Coordinate & Active Selected Robot Plan Overlay Bar */}
        <div className="flex flex-wrap items-center justify-between gap-4 px-4 py-2.5 bg-slate-950/95 border-t border-slate-800 text-xs">
          <div className="flex items-center gap-2 min-w-0">
            <span className="font-mono font-semibold text-cyan-400 whitespace-nowrap shrink-0">
              [{selectedRobot.id}]
            </span>
            <span className="font-medium text-slate-400 whitespace-nowrap shrink-0">
              Active Plan:
            </span>
            <span className="text-slate-100 font-medium truncate">
              {selectedRobot.planSummary}
            </span>
          </div>

          <div className="flex items-center gap-4 font-mono text-slate-400 tabular-nums">
            <span>
              Pose: ({selectedRobot.position.x.toFixed(1)}m, {selectedRobot.position.y.toFixed(1)}m)
            </span>
            {hoverCoord && (
              <span className="text-cyan-300">
                Cursor: ({hoverCoord.x.toFixed(1)}m, {hoverCoord.y.toFixed(1)}m)
                {hoveredFieldValue !== null && ` · ${heatmapLayer}=${hoveredFieldValue.toFixed(2)}`}
              </span>
            )}
          </div>
        </div>
      </div>
    </section>
  );
};
