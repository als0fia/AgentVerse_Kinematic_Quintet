/**
 * @file src/components/FieldMapCanvas.tsx
 * @description Interactive 2D Flooded Paddy Field Map for the Antenna Multi-Robot System.
 *
 * This component renders the metric field coordinate canvas ($24\text{ m} \times 16\text{ m}$) displaying:
 * - Each robot in the Kinematic Quintet (`ANT-01` through `ANT-05`) as a high-contrast color-coded dot
 *   with heading indicator, UWB ranging ring, callsign badge, and live plan target vector.
 * - Color-coded status legend and interactive layer toggles (Trajectories, UWB Tether Links, Crop Gaps, Soil Hazards).
 * - Active INA240 winch tether lines connecting rescuing rovers (`ANT-01`) to immobilized peers (`ANT-03`)
 *   with live tension readouts in Newtons.
 * - Missing rice crop gap sites (`C-01`–`C-08`) and submerged paddy hazards (`S1` sinkhole, `S3` bund, `S4` trench).
 * - Click-to-Command interactivity: Clicking any robot selects it; clicking any crop gap site or field coordinate
 *   can immediately dispatch the selected robot to navigate, sample, or plant at that location.
 *
 * Use Cases:
 * 1. Real-time spatial situational awareness of where every robot is and what plan it is executing.
 * 2. Rapid visual identification of immobilized or high-slip robots via color + symbol coding.
 * 3. Direct point-and-click waypoint or crop-site dispatch on the map surface.
 */

import React, { useState } from 'react';
import { APP_CONFIG } from '../config/appConfig';
import {
  CropGapSite,
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
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  onDispatchRobotToPoint: (
    robotId: string,
    taskMode: TaskMode,
    coord: MetricCoordinate,
    options?: { targetPeerId?: string | null; targetSiteId?: string | null }
  ) => void;
  simulationRunning: boolean;
  onToggleSimulation: () => void;
  onStepSimulationOnce: () => void;
  onResetSwarm: () => void;
}

/**
 * Renders the interactive paddy field map with color-coded robot dots, plan vectors, and crop sites.
 * @param props - FieldMapCanvasProps state and action callbacks.
 * @returns React JSX map workspace element.
 */
export const FieldMapCanvas: React.FC<FieldMapCanvasProps> = ({
  robots,
  cropGapSites,
  hazardZones,
  selectedRobotId,
  onSelectRobot,
  onDispatchRobotToPoint,
  simulationRunning,
  onToggleSimulation,
  onStepSimulationOnce,
  onResetSwarm,
}) => {
  const [showPlanVectors, setShowPlanVectors] = useState<boolean>(true);
  const [showHazardZones, setShowHazardZones] = useState<boolean>(true);
  const [showBreadcrumbs, setShowBreadcrumbs] = useState<boolean>(true);
  const [mapClickMode, setMapClickMode] = useState<'INSPECT' | 'DISPATCH_NAVIGATE' | 'DISPATCH_PLANT' | 'DISPATCH_SAMPLE'>('INSPECT');
  const [hoverCoord, setHoverCoord] = useState<MetricCoordinate | null>(null);

  const widthM = APP_CONFIG.fieldGeometry.widthMeters;
  const heightM = APP_CONFIG.fieldGeometry.heightMeters;
  const svgWidth = 960;
  const svgHeight = 640;

  /**
   * Converts metric field coordinates (meters) to SVG viewport pixels.
   * @param coord - MetricCoordinate (x in [0, 24], y in [0, 16]).
   * @returns Pixel coordinate { px, py }.
   */
  function metricToPixel(coord: MetricCoordinate): { px: number; py: number } {
    return {
      px: Math.round((coord.x / widthM) * svgWidth * 10) / 10,
      py: Math.round((coord.y / heightM) * svgHeight * 10) / 10,
    };
  }

  /**
   * Handles clicks on the SVG background field to dispatch the currently selected robot when in a dispatch mode.
   * @param event - Mouse click event on the SVG element.
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
   * @param event - Mouse move event on the SVG element.
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
   * @param event - Mouse click event.
   * @param robot - Clicked RobotState.
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
   * @param event - Mouse click event.
   * @param site - Clicked CropGapSite.
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
      {/* Map Control Toolbar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-3 bg-slate-950 border-b border-slate-800">
        <div>
          <h2 className="text-sm font-semibold text-slate-100">
            Terrace Sector B-04 · Live Multi-Robot Field Map (24.0m × 16.0m)
          </h2>
          <p className="text-xs text-slate-400">
            Click any robot dot to inspect · Click any crop gap site (C-01–C-08) to assign {selectedRobot.callsign.split('·')[0].trim()}
          </p>
        </div>

        {/* Simulation Playback Controls */}
        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onToggleSimulation', { next: !simulationRunning });
              onToggleSimulation();
            }}
            className={`px-3 py-1.5 text-xs font-medium rounded border transition-colors whitespace-nowrap shrink-0 ${
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
            className="px-3 py-1.5 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-200 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0"
          >
            Step +1 Tick
          </button>

          <button
            type="button"
            onClick={() => {
              logFunctionCall('FieldMapCanvas.onResetSwarm', {});
              onResetSwarm();
            }}
            className="px-3 py-1.5 text-xs font-medium rounded border border-slate-700 bg-slate-900 text-slate-300 hover:bg-slate-800 transition-colors whitespace-nowrap shrink-0"
          >
            Reset Baseline
          </button>
        </div>
      </div>

      {/* Color-Coded Status Legend & Map Click Dispatch Mode Bar */}
      <div className="flex flex-wrap items-center justify-between gap-3 px-4 py-2.5 bg-slate-900/90 border-b border-slate-800 text-xs">
        {/* Status Color Legend (Non-hue-only: Glyph + Text + Color) */}
        <div className="flex flex-wrap items-center gap-4" aria-label="Robot Status Color Legend">
          <span className="text-slate-400 font-medium">Status Legend:</span>
          {Object.values(APP_CONFIG.statusTheme).map((theme) => (
            <span key={theme.code} className={`inline-flex items-center gap-1.5 font-mono ${theme.textClass}`}>
              <span aria-hidden="true">{theme.glyph}</span>
              <span>{theme.shortLabel}</span>
            </span>
          ))}
        </div>

        {/* Layer Toggles & Map Click Mode */}
        <div className="flex flex-wrap items-center gap-2">
          <div className="flex items-center gap-1 p-0.5 bg-slate-950 border border-slate-800 rounded">
            <button
              type="button"
              onClick={() => setMapClickMode('INSPECT')}
              className={`px-2.5 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 ${
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
              className={`px-2.5 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 ${
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
              className={`px-2.5 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 ${
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
              className={`px-2.5 py-1 text-xs rounded transition-colors whitespace-nowrap shrink-0 ${
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
            className={`px-2.5 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 ${
              showPlanVectors
                ? 'border-cyan-500/40 bg-cyan-500/10 text-cyan-300'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Plan Vectors
          </button>

          <button
            type="button"
            onClick={() => setShowHazardZones((v) => !v)}
            className={`px-2.5 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 ${
              showHazardZones
                ? 'border-rose-500/40 bg-rose-500/10 text-rose-300'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Mud Hazards
          </button>

          <button
            type="button"
            onClick={() => setShowBreadcrumbs((v) => !v)}
            className={`px-2.5 py-1 text-xs rounded border transition-colors whitespace-nowrap shrink-0 ${
              showBreadcrumbs
                ? 'border-slate-600 bg-slate-800 text-slate-200'
                : 'border-slate-800 bg-slate-950 text-slate-400'
            }`}
          >
            Breadcrumbs
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
            <pattern id="paddyMetricGrid" width={svgWidth / 12} height={svgHeight / 8} patternUnits="userSpaceOnUse">
              <path
                d={`M ${svgWidth / 12} 0 L 0 0 0 ${svgHeight / 8}`}
                fill="none"
                stroke="rgba(148, 163, 184, 0.10)"
                strokeWidth="1"
              />
            </pattern>
            {/* Arrowhead markers for robot plan vectors */}
            <marker id="arrow-cyan" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#06b6d4" />
            </marker>
            <marker id="arrow-emerald" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#10b981" />
            </marker>
            <marker id="arrow-amber" viewBox="0 0 10 10" refX="7" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
              <path d="M 0 1 L 10 5 L 0 9 z" fill="#f59e0b" />
            </marker>
          </defs>

          {/* Flooded paddy water & soil base rect */}
          <rect x="0" y="0" width={svgWidth} height={svgHeight} fill="#07131a" />
          <rect x="0" y="0" width={svgWidth} height={svgHeight} fill="url(#paddyMetricGrid)" />

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
                {/* Established Rice Crop Furrow Line */}
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
                  UWB-BASE
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

          {/* Robot Plan Trajectory Vectors & Active Winch Tethers */}
          {showPlanVectors &&
            robots.map((robot) => {
              const start = metricToPixel(robot.position);
              const end = metricToPixel(robot.targetWaypoint);
              const dist = Math.hypot(end.px - start.px, end.py - start.py);
              const theme = APP_CONFIG.statusTheme[robot.status];

              // Also check if this robot is actively retrieving a peer via winch tether
              const peerRobot =
                robot.taskMode === 'RETRIEVE_PEER' && robot.targetPeerId
                  ? robots.find((r) => r.id === robot.targetPeerId)
                  : null;

              return (
                <g key={`plan-vec-${robot.id}`}>
                  {dist > 8 && (
                    <g>
                      <line
                        x1={start.px}
                        y1={start.py}
                        x2={end.px}
                        y2={end.py}
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
                              x={midX - 42}
                              y={midY - 22}
                              width={84}
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
                              TETHER {robot.telemetry.tetherTensionN}N
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
              Selected Pose: ({selectedRobot.position.x.toFixed(1)}m, {selectedRobot.position.y.toFixed(1)}m)
            </span>
            {hoverCoord && (
              <span className="text-cyan-300">
                Cursor: ({hoverCoord.x.toFixed(1)}m, {hoverCoord.y.toFixed(1)}m)
              </span>
            )}
          </div>
        </div>
      </div>
    </section>
  );
};
