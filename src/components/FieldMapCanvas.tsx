/**
 * @file src/components/FieldMapCanvas.tsx
 * @description Interactive 2D Top-Down Belief & Rice-Fitness Visualiser for the Antenna Multi-Robot Dashboard.
 *
 * Mirrors the Right Pane ("BIG Interactive 2D Top-Down Belief & Rice-Fitness Visualiser") from
 * `src/ricebot/viewer/live_app.py` (`LiveSwarmDashboardApp`):
 * - Supports switching the inspected world model across:
 *   - `planter` ("Planter Local Belief")
 *   - `scout` ("Scout Local Belief")
 *   - `divergence` ("|Planter - Scout| Belief Gap")
 *   - `truth` ("Hidden Ground Truth f(x,y)")
 * - Supports all 8 continuous 2D spatial layers from `live_app.py` (`LAYER_META`):
 *   - `planting_fitness` ("Rice Fitness Φ(x,y)" — default)
 *   - `moisture` ("Soil Moisture M(x,y)")
 *   - `firmness` ("Cone Firmness F(x,y)")
 *   - `stand_gap` ("Canopy Stand Gap G(x,y)")
 *   - `plant_density` ("Plant Density ρ(x,y)")
 *   - `uncertainty` ("Epistemic Uncertainty σ(x,y)")
 *   - `traversability_risk` ("Mud / SLAM Risk R(x,y)")
 *   - `elevation_m` ("Micro-Topography z(x,y)")
 * - Renders color-coded robot dots (`ANT-01`–`ANT-05`), heading vectors, dashed plan paths,
 *   explicit plain-English robot plans, and UWB+HaLow P2P gossip beams.
 *
 * Use Cases:
 * 1. Observing where each robot is, its color-coded status, and its active plan on the 2D belief map.
 * 2. Comparing Planter vs. Scout local belief maps and inspecting the Rice Planting Fitness Index Φ(x,y).
 * 3. Clicking any robot dot to inspect/control it, or clicking any crop-gap site to dispatch the selected robot.
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

const BELIEF_VIEW_OPTIONS: { id: BeliefPerspective; label: string }[] = [
  { id: 'planter', label: 'Planter Local Belief' },
  { id: 'scout', label: 'Scout Local Belief' },
  { id: 'divergence', label: '|Planter − Scout| Gap' },
  { id: 'truth', label: 'Hidden Ground Truth f(x,y)' },
];

const LAYER_OPTIONS: { id: HeatmapSelectorOption; shortLabel: string; fullTitle: string }[] = [
  { id: 'planting_fitness', shortLabel: 'Rice Fitness Φ', fullTitle: 'Rice Planting Fitness Φ(x,y)' },
  { id: 'moisture', shortLabel: 'Moisture M', fullTitle: 'Soil Moisture M(x,y)' },
  { id: 'firmness', shortLabel: 'Firmness F', fullTitle: 'Cone Firmness F(x,y)' },
  { id: 'stand_gap', shortLabel: 'Stand Gap G', fullTitle: 'Canopy Stand Gap G(x,y)' },
  { id: 'plant_density', shortLabel: 'Plant Density ρ', fullTitle: 'Plant Density ρ(x,y)' },
  { id: 'uncertainty', shortLabel: 'Uncertainty σ', fullTitle: 'Epistemic Uncertainty σ(x,y)' },
  { id: 'traversability_risk', shortLabel: 'Mud/SLAM Risk R', fullTitle: 'Mud / SLAM Risk R(x,y)' },
  { id: 'elevation_m', shortLabel: 'Micro-Topo z', fullTitle: 'Micro-Topography z(x,y)' },
];

/**
 * Maps a normalized [0, 1] value to the exact RGB palette formulas from `_grid_to_rgb_ppm` in `src/ricebot/viewer/live_app.py`.
 * @param layer - Active heatmap layer.
 * @param value - Normalized scalar value in [0, 1].
 * @returns CSS rgba color string.
 */
function getLiveAppHeatmapColor(layer: HeatmapSelectorOption, value: number): string {
  if (layer === 'none') return 'transparent';
  const t = Math.max(0, Math.min(1, value));
  let r = 30;
  let g = 120;
  let b = 90;

  if (layer === 'planting_fitness') {
    r = 195.0 * (1.0 - t) + 25.0 * t;
    g = 55.0 + 175.0 * Math.pow(t, 0.85);
    b = 45.0 + 65.0 * t;
  } else if (layer === 'moisture') {
    r = 215.0 - 185.0 * t;
    g = 180.0 - 75.0 * Math.pow(t, 1.1) + 30.0 * (1.0 - Math.abs(2.0 * t - 1.0));
    b = 95.0 + 150.0 * Math.pow(t, 0.85);
  } else if (layer === 'firmness') {
    r = 120.0 + 95.0 * (1.0 - t);
    g = 65.0 + 150.0 * t;
    b = 45.0 + 75.0 * t;
  } else if (layer === 'plant_density') {
    r = 145.0 - 110.0 * t;
    g = 110.0 + 115.0 * t;
    b = 65.0 - 20.0 * t;
  } else if (layer === 'stand_gap') {
    r = 35.0 + 210.0 * t;
    g = 150.0 + 55.0 * t - 60.0 * (t * t);
    b = 85.0 - 55.0 * t;
  } else if (layer === 'uncertainty') {
    r = 20.0 + 225.0 * t;
    g = 130.0 - 90.0 * t + 70.0 * (t * t);
    b = 190.0 - 40.0 * t;
  } else if (layer === 'elevation_m') {
    r = 50.0 + 175.0 * t;
    g = 95.0 + 125.0 * t;
    b = 85.0 + 60.0 * (1.0 - t);
  } else {
    // traversability_risk / divergence
    r = 30.0 + 220.0 * Math.pow(t, 0.85);
    g = 175.0 * (1.0 - t);
    b = 95.0 * (1.0 - t) + 110.0 * (t * t);
  }

  return `rgba(${Math.round(r)}, ${Math.round(g)}, ${Math.round(b)}, 0.42)`;
}

/**
 * Renders the Right-Pane 2D Top-Down Belief & Rice-Fitness Visualiser matching `src/ricebot/viewer/live_app.py`.
 */
export const FieldMapCanvas: React.FC<FieldMapCanvasProps> = ({
  robots,
  cropGapSites,
  hazardZones,
  obstacles,
  selectedRobotId,
  onSelectRobot,
  onDispatchRobotToPoint,
}) => {
  const [heatmapLayer, setHeatmapLayer] = useState<HeatmapSelectorOption>('planting_fitness');
  const [perspective, setPerspective] = useState<BeliefPerspective>('planter');
  const [clickDispatchEnabled, setClickDispatchEnabled] = useState<boolean>(false);

  const widthM = APP_CONFIG.fieldGeometry.widthMeters;
  const heightM = APP_CONFIG.fieldGeometry.heightMeters;
  const svgWidth = 840;
  const svgHeight = 540;

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
   * Precomputes the vectorized 2D continuous belief grid (`32 x 20` cells).
   */
  const heatmapCells = useMemo(() => {
    if (heatmapLayer === 'none') return [];
    const cols = 32;
    const rows = 20;
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
          w: cellW + 0.6,
          h: cellH + 0.6,
          fill: getLiveAppHeatmapColor(
            perspective === 'divergence' ? 'traversability_risk' : heatmapLayer,
            val
          ),
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

  const activeLayerMeta =
    LAYER_OPTIONS.find((l) => l.id === heatmapLayer) ?? LAYER_OPTIONS[0];

  return (
    <section
      aria-label="2D Top-Down Belief and Rice-Fitness Visualiser"
      className="flex flex-col bg-[#111d2c] border border-[#1f3247] rounded-md overflow-hidden h-full"
    >
      {/* Row 1: Inspect World Model Selector + Click-to-Move toggle */}
      <div className="flex flex-wrap items-center justify-between gap-2 px-3 py-1.5 bg-[#0d1826] border-b border-[#1f3247]">
        <div className="flex flex-wrap items-center gap-1">
          <span className="text-xs font-bold text-amber-400 mr-1 whitespace-nowrap">
            2D Belief Map:
          </span>
          {BELIEF_VIEW_OPTIONS.map((opt) => {
            const active = perspective === opt.id;
            return (
              <button
                key={opt.id}
                type="button"
                onClick={() => {
                  logFunctionCall('FieldMapCanvas.setPerspective', { perspective: opt.id });
                  setPerspective(opt.id);
                }}
                className={`px-2 py-0.5 text-[11px] font-medium rounded transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
                  active
                    ? 'bg-sky-600 text-white font-semibold'
                    : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700'
                }`}
              >
                {opt.label}
              </button>
            );
          })}
        </div>

        <button
          type="button"
          onClick={() => setClickDispatchEnabled((v) => !v)}
          className={`px-2 py-0.5 text-[11px] font-medium rounded border transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
            clickDispatchEnabled
              ? 'bg-amber-500/20 border-amber-400 text-amber-200'
              : 'bg-slate-800 border-slate-700 text-slate-300 hover:bg-slate-700'
          }`}
        >
          {clickDispatchEnabled ? 'Click Map to Send Robot...' : 'Send Selected Robot'}
        </button>
      </div>

      {/* Row 2: 2D Field Layer Radio Bar matching live_app.py */}
      <div className="flex flex-wrap items-center gap-1 px-3 py-1 bg-[#111d2c] border-b border-[#1f3247]">
        <span className="text-[11px] font-semibold text-slate-400 mr-1">2D Field Layer:</span>
        {LAYER_OPTIONS.map((l) => {
          const active = heatmapLayer === l.id;
          return (
            <button
              key={l.id}
              type="button"
              onClick={() => {
                logFunctionCall('FieldMapCanvas.setHeatmapLayer', { layer: l.id });
                setHeatmapLayer(l.id);
              }}
              className={`px-2 py-0.5 text-[11px] rounded transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
                active
                  ? 'bg-sky-600 text-white font-semibold'
                  : 'bg-slate-800/70 text-slate-300 hover:bg-slate-700'
              }`}
            >
              {l.shortLabel}
            </button>
          );
        })}
      </div>

      {/* Maximized 2D Top-Down SVG Map */}
      <div className="relative flex-1 bg-[#070d14] min-h-[470px] flex items-center justify-center">
        <svg
          viewBox={`0 0 ${svgWidth} ${svgHeight}`}
          onClick={handleMapSurfaceClick}
          className={`w-full h-full block select-none ${
            clickDispatchEnabled ? 'cursor-crosshair' : 'cursor-default'
          }`}
        >
          {/* Vectorized 2D Heatmap Grid */}
          <g>
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
          </g>

          {/* Subtle Meter Grid Lines */}
          <g stroke="rgba(148, 163, 184, 0.12)" strokeWidth="1">
            {[4, 8, 12, 16, 20].map((xm) => {
              const { px } = metricToPixel({ x: xm, y: 0 });
              return <line key={`vx-${xm}`} x1={px} y1={0} x2={px} y2={svgHeight} />;
            })}
            {[4, 8, 12].map((ym) => {
              const { py } = metricToPixel({ x: 0, y: ym });
              return <line key={`hy-${ym}`} x1={0} y1={py} x2={svgWidth} y2={py} />;
            })}
          </g>

          {/* SLAM / Mud Hazard Zones */}
          {hazardZones.map((hz) => {
            const { px, py } = metricToPixel(hz.position);
            const rPx = (hz.radiusMeters / widthM) * svgWidth;
            return (
              <g key={hz.id}>
                <circle
                  cx={px}
                  cy={py}
                  r={rPx}
                  fill="rgba(239, 68, 68, 0.14)"
                  stroke="#ef4444"
                  strokeWidth="1.5"
                  strokeDasharray="4 3"
                />
                <text
                  x={px}
                  y={py - rPx - 5}
                  textAnchor="middle"
                  fill="#fca5a5"
                  fontSize="10"
                  fontFamily="monospace"
                  fontWeight="bold"
                >
                  {hz.label}
                </text>
              </g>
            );
          })}

          {/* Dynamic Injected Obstacles */}
          {obstacles.map((obs) => {
            const { px, py } = metricToPixel(obs.position);
            return (
              <g key={obs.id}>
                <rect
                  x={px - 11}
                  y={py - 11}
                  width={22}
                  height={22}
                  fill="#dc2626"
                  stroke="#fef08a"
                  strokeWidth="2"
                />
                <text
                  x={px}
                  y={py - 15}
                  textAnchor="middle"
                  fill="#fef08a"
                  fontSize="10"
                  fontFamily="monospace"
                  fontWeight="bold"
                >
                  OBSTACLE
                </text>
              </g>
            );
          })}

          {/* Crop Gap / Patch Sites (`patch_a`..`patch_d` / `C-01`..`C-08`) */}
          {cropGapSites.map((site) => {
            const { px, py } = metricToPixel(site.position);
            const isPlanted = site.status === 'PLANTED_VERIFIED';
            const isRejected = site.status === 'HAZARD_SKIPPED';
            const ringColor = isPlanted
              ? '#22c55e'
              : isRejected
              ? '#f87171'
              : site.status === 'ASSIGNED'
              ? '#38bdf8'
              : '#fbbf24';
            const statusTag = isPlanted
              ? 'PLANTED'
              : isRejected
              ? 'MUD REJECT'
              : site.status === 'ASSIGNED'
              ? 'VIABLE'
              : '?';

            return (
              <g
                key={site.id}
                onClick={(e) => handleCropSiteClick(e, site)}
                className="cursor-pointer"
              >
                <circle
                  cx={px}
                  cy={py}
                  r={15}
                  fill="rgba(15, 23, 42, 0.55)"
                  stroke={ringColor}
                  strokeWidth="2"
                />
                <text
                  x={px}
                  y={py + 3}
                  textAnchor="middle"
                  fill="#f8fafc"
                  fontSize="9"
                  fontFamily="monospace"
                  fontWeight="bold"
                >
                  {site.id}
                </text>
                <text
                  x={px}
                  y={py - 19}
                  textAnchor="middle"
                  fill={ringColor}
                  fontSize="9"
                  fontFamily="monospace"
                  fontWeight="bold"
                >
                  {site.id} [{statusTag}]
                </text>
              </g>
            );
          })}

          {/* UWB + Wi-Fi HaLow Peer-to-Peer Gossip Link between active rovers */}
          {robots.length >= 2 && (() => {
            const p0 = metricToPixel(robots[0].position);
            const p1 = metricToPixel(robots[2]?.position ?? robots[1].position);
            return (
              <g>
                <line
                  x1={p0.px}
                  y1={p0.py}
                  x2={p1.px}
                  y2={p1.py}
                  stroke="#22d3ee"
                  strokeWidth="2"
                  strokeDasharray="6 4"
                />
                <text
                  x={(p0.px + p1.px) * 0.5}
                  y={(p0.py + p1.py) * 0.5 - 7}
                  textAnchor="middle"
                  fill="#fef08a"
                  fontSize="9"
                  fontFamily="monospace"
                  fontWeight="bold"
                >
                  UWB+HaLow GOSSIP
                </text>
              </g>
            );
          })()}

          {/* Robot Trajectories, Dashed Plan Paths & Color-Coded Robot Dots */}
          {robots.map((robot, idx) => {
            const { px, py } = metricToPixel(robot.position);
            const targetPx = metricToPixel(robot.targetWaypoint);
            const theme = APP_CONFIG.statusTheme[robot.status];
            const isSelected = robot.id === selectedRobotId;
            const yawRad = (robot.headingDeg * Math.PI) / 180;
            const hx = px + 18 * Math.cos(yawRad);
            const hy = py + 18 * Math.sin(yawRad);

            return (
              <g key={robot.id}>
                {/* Dashed Planned Trajectory Line */}
                <line
                  x1={px}
                  y1={py}
                  x2={targetPx.px}
                  y2={targetPx.py}
                  stroke={theme.hex}
                  strokeWidth={isSelected ? '2.5' : '1.8'}
                  strokeDasharray="5 4"
                  opacity={0.85}
                />

                {/* Clickable Robot Dot */}
                <g
                  onClick={(e) => handleRobotDotClick(e, robot)}
                  className="cursor-pointer"
                >
                  {isSelected && (
                    <circle
                      cx={px}
                      cy={py}
                      r={20}
                      fill="none"
                      stroke="#fef08a"
                      strokeWidth="2.5"
                    />
                  )}
                  <circle
                    cx={px}
                    cy={py}
                    r={13}
                    fill={theme.hex}
                    stroke="#ffffff"
                    strokeWidth="2"
                  />
                  {/* Heading Indicator */}
                  <line
                    x1={px}
                    y1={py}
                    x2={hx}
                    y2={hy}
                    stroke="#fef08a"
                    strokeWidth="2"
                  />
                  <text
                    x={px}
                    y={py + 3.5}
                    textAnchor="middle"
                    fill="#0f172a"
                    fontSize="9"
                    fontFamily="monospace"
                    fontWeight="bold"
                  >
                    R{idx + 1}
                  </text>

                  {/* Robot ID + Color-Coded Status Tag */}
                  <rect
                    x={px - 62}
                    y={py + 16}
                    width={124}
                    height={15}
                    rx={3}
                    fill="rgba(9, 19, 31, 0.88)"
                    stroke={isSelected ? '#fef08a' : '#1f3247'}
                    strokeWidth="1"
                  />
                  <text
                    x={px}
                    y={py + 26.5}
                    textAnchor="middle"
                    fill={isSelected ? '#fef08a' : '#f8fafc'}
                    fontSize="9.5"
                    fontFamily="monospace"
                    fontWeight="bold"
                  >
                    {robot.id} [{theme.shortLabel.toUpperCase()}]
                  </text>

                  {/* Concise Plan Label on Map */}
                  <text
                    x={px}
                    y={py + 40}
                    textAnchor="middle"
                    fill="#cbd5e1"
                    fontSize="9"
                    fontFamily="monospace"
                  >
                    {robot.planSummary.length > 36
                      ? `${robot.planSummary.slice(0, 36)}...`
                      : robot.planSummary}
                  </text>
                </g>
              </g>
            );
          })}

          {/* Top HUD Banner matching live_app.py lines 1439-1447 */}
          <rect
            x={8}
            y={8}
            width={svgWidth - 16}
            height={24}
            fill="#09131f"
            stroke="#1f3247"
            strokeWidth="1"
          />
          <text
            x={14}
            y={24}
            fill="#38bdf8"
            fontSize="11"
            fontFamily="monospace"
            fontWeight="bold"
          >
            {`View: ${perspective.toUpperCase()}  |  Layer: ${activeLayerMeta.fullTitle}  (Click any robot dot to inspect or control)`}
          </text>
        </svg>
      </div>
    </section>
  );
};
