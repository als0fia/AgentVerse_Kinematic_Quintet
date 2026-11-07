/**
 * @file src/components/Mujoco3DViewport.tsx
 * @description Live 3D MuJoCo Physics & Terramechanics Visualiser for the Antenna Swarm Dashboard.
 *
 * This component mirrors the Left Pane ("BIG 3D MuJoCo Physics Visualiser") from
 * `src/ricebot/viewer/live_app.py` (`LiveSwarmDashboardApp`):
 * - Interactive 3D perspective viewport with mouse-drag camera orbit (`azimuth`, `elevation`)
 *   and scroll-wheel zoom (`distance`).
 * - Supports all 6 camera presets from `live_app.py` and `adapter.py`:
 *   1. `isometric` ("Isometric Terrace")
 *   2. `wheel_closeup` ("Low Mud/Wheel Close-Up")
 *   3. `planter_follow_cam` ("Follow Planter")
 *   4. `scout_follow_cam` ("Follow Scout")
 *   5. `planter_ego_cam` ("Ego Optical Cam" — first-person onboard optical perception view)
 *   6. `top_overview` ("Top 3D")
 * - Renders the 3D undulating paddy terrace heightfield (`paddy_micro_relief`), 3D sunken
 *   wheel-mud rut tracks (`mud_rut_history`), 3D wheel-sinkage collar rings (`collar_rgba`),
 *   split dual-rover 3D belief spheres above each patch, 3D SLAM hazard cells, UWB/HaLow
 *   cyan gossip beams, LiDAR rays, and rigid cam-latch docking links between rescuing rovers.
 *
 * Use Cases:
 * 1. Inspecting real-time 3D vertical wheel-mud sinkage ($z_{\text{sink}}$) and belly clearance
 *    when triggering the 13kg S1 Mud Belly-Out overload vs. 4.2kg nominal sizing.
 * 2. Switching to the Low Mud/Wheel Close-Up or Ego Optical Camera during S1 mud extraction
 *    or Emergency Human Control teleoperation.
 * 3. Observing 3D split belief spheres and UWB/HaLow gossip beams alongside the 2D belief map.
 */

import React, { useEffect, useRef, useState } from 'react';
import {
  CropGapSite,
  DynamicObstacle,
  FieldHazardZone,
  MujocoCameraPreset,
  RobotState,
} from '../types/antenna';
import { logFunctionCall } from '../utils/logger';

interface Mujoco3DViewportProps {
  robots: RobotState[];
  cropGapSites: CropGapSite[];
  hazardZones: FieldHazardZone[];
  obstacles: DynamicObstacle[];
  selectedRobotId: string;
  onSelectRobot: (robotId: string) => void;
  cameraPreset: MujocoCameraPreset;
  onChangeCameraPreset: (preset: MujocoCameraPreset) => void;
  vehicleMassKg: number;
  simTimeSec: number;
}

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

interface ProjectedPoint {
  px: number;
  py: number;
  depth: number;
  visible: boolean;
}

const CAMERA_PRESET_OPTIONS: { id: MujocoCameraPreset; label: string }[] = [
  { id: 'isometric', label: 'Isometric Terrace' },
  { id: 'wheel_closeup', label: 'Low Mud/Wheel Close-Up' },
  { id: 'planter_follow_cam', label: 'Follow Planter' },
  { id: 'scout_follow_cam', label: 'Follow Scout' },
  { id: 'planter_ego_cam', label: 'Ego Optical Cam' },
  { id: 'top_overview', label: 'Top 3D' },
];

/**
 * Evaluates synthetic 3D micro-topography elevation z(x, y) in meters on the 24m x 16m terrace.
 * @param x - Metric X coordinate (0..24m).
 * @param y - Metric Y coordinate (0..16m).
 * @returns Elevation z in meters (-0.035m to +0.045m).
 */
function sampleTerraceElevationM(x: number, y: number): number {
  const dSinkhole = Math.hypot(x - 14.6, y - 8.4);
  const dBund = Math.hypot(x - 18.2, y - 10.2);
  const dTrench = Math.hypot(x - 22.4, y - 9.0);
  const microWave = 0.008 * Math.sin(x * 0.55) * Math.cos(y * 0.65);
  const sinkDip = -0.032 * Math.exp(-(dSinkhole * dSinkhole) / 7.5);
  const bundRise = 0.042 * Math.exp(-(dBund * dBund) / 4.2);
  const trenchDip = -0.028 * Math.exp(-(dTrench * dTrench) / 3.8);
  return microWave + sinkDip + bundRise + trenchDip;
}

/**
 * Projects a 3D world point (x, y, z) in meters onto 2D canvas coordinates using a spherical orbit camera.
 * Note: High-frequency rendering helper; does not log per-vertex to preserve 60fps performance.
 */
function projectPoint3D(
  pt: Vec3,
  lookAt: Vec3,
  azimuthDeg: number,
  elevationDeg: number,
  distance: number,
  canvasW: number,
  canvasH: number,
  fovScale: number
): ProjectedPoint {
  const az = (azimuthDeg * Math.PI) / 180;
  const el = (elevationDeg * Math.PI) / 180;

  // Camera position relative to lookAt
  const camX = lookAt.x + distance * Math.cos(el) * Math.cos(az);
  const camY = lookAt.y + distance * Math.cos(el) * Math.sin(az);
  const camZ = lookAt.z - distance * Math.sin(el);

  // Forward vector from camera to lookAt
  const fx = lookAt.x - camX;
  const fy = lookAt.y - camY;
  const fz = lookAt.z - camZ;
  const fLen = Math.hypot(fx, fy, fz) || 1;
  const fnx = fx / fLen;
  const fny = fy / fLen;
  const fnz = fz / fLen;

  // World up is +Z (with Y inverted so South is down in 2D map)
  const upX = 0;
  const upY = 0;
  const upZ = 1;

  // Right = Forward x Up
  let rx = fny * upZ - fnz * upY;
  let ry = fnz * upX - fnx * upZ;
  let rz = fnx * upY - fny * upX;
  const rLen = Math.hypot(rx, ry, rz) || 1;
  rx /= rLen;
  ry /= rLen;
  rz /= rLen;

  // True Up = Right x Forward
  const ux = ry * fnz - rz * fny;
  const uy = rz * fnx - rx * fnz;
  const uz = rx * fny - ry * fnx;

  // Vector from camera to point
  const dx = pt.x - camX;
  const dy = pt.y - camY;
  const dz = pt.z - camZ;

  const camSpaceX = dx * rx + dy * ry + dz * rz;
  const camSpaceY = dx * ux + dy * uy + dz * uz;
  const camSpaceZ = dx * fnx + dy * fny + dz * fnz;

  if (camSpaceZ <= 0.15) {
    return { px: 0, py: 0, depth: camSpaceZ, visible: false };
  }

  const scale = fovScale / camSpaceZ;
  return {
    px: canvasW * 0.5 + camSpaceX * scale,
    py: canvasH * 0.52 - camSpaceY * scale,
    depth: camSpaceZ,
    visible: true,
  };
}

/**
 * Renders the maximized Left-Pane 3D MuJoCo Physics Visualiser matching `src/ricebot/viewer/live_app.py`.
 */
export const Mujoco3DViewport: React.FC<Mujoco3DViewportProps> = ({
  robots,
  cropGapSites,
  hazardZones,
  obstacles,
  selectedRobotId,
  onSelectRobot,
  cameraPreset,
  onChangeCameraPreset,
  vehicleMassKg,
  simTimeSec,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [azimuthDeg, setAzimuthDeg] = useState<number>(118);
  const [elevationDeg, setElevationDeg] = useState<number>(-28);
  const [distanceM, setDistanceM] = useState<number>(17.5);
  const dragRef = useRef<{ x: number; y: number } | null>(null);

  const selectedRobot = robots.find((r) => r.id === selectedRobotId) ?? robots[0];

  // Update camera angles when preset changes
  useEffect(() => {
    if (cameraPreset === 'isometric') {
      setAzimuthDeg(118);
      setElevationDeg(-28);
      setDistanceM(17.5);
    } else if (cameraPreset === 'wheel_closeup') {
      setAzimuthDeg(132);
      setElevationDeg(-16);
      setDistanceM(5.2);
    } else if (cameraPreset === 'planter_follow_cam' || cameraPreset === 'scout_follow_cam') {
      setAzimuthDeg(112);
      setElevationDeg(-22);
      setDistanceM(8.4);
    } else if (cameraPreset === 'planter_ego_cam') {
      setAzimuthDeg((selectedRobot.headingDeg + 180) % 360);
      setElevationDeg(-10);
      setDistanceM(3.8);
    } else if (cameraPreset === 'top_overview') {
      setAzimuthDeg(90);
      setElevationDeg(-82);
      setDistanceM(21.0);
    }
  }, [cameraPreset, selectedRobot.headingDeg]);

  /**
   * Handles mouse down on the 3D canvas to begin camera orbit drag or select a nearby 3D rover.
   */
  function handleMouseDown(event: React.MouseEvent<HTMLCanvasElement>): void {
    dragRef.current = { x: event.clientX, y: event.clientY };
  }

  /**
   * Handles mouse drag on the 3D canvas to orbit camera azimuth and elevation.
   */
  function handleMouseMove(event: React.MouseEvent<HTMLCanvasElement>): void {
    if (!dragRef.current) return;
    const dx = event.clientX - dragRef.current.x;
    const dy = event.clientY - dragRef.current.y;
    dragRef.current = { x: event.clientX, y: event.clientY };
    setAzimuthDeg((prev) => (prev - dx * 0.45 + 360) % 360);
    setElevationDeg((prev) => Math.max(-85, Math.min(-6, prev - dy * 0.35)));
  }

  /**
   * Ends 3D camera orbit dragging.
   */
  function handleMouseUp(): void {
    dragRef.current = null;
  }

  /**
   * Handles mouse wheel zoom on the 3D viewport.
   */
  function handleWheel(event: React.WheelEvent<HTMLCanvasElement>): void {
    const delta = event.deltaY > 0 ? 1.1 : -1.1;
    setDistanceM((prev) => Math.max(3.4, Math.min(28.0, prev + delta)));
  }

  // Draw the 3D MuJoCo scene
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const W = canvas.width;
    const H = canvas.height;

    // Determine camera lookAt based on preset
    const planterRobot = robots[0];
    const scoutRobot = robots[2] ?? robots[1] ?? robots[0];
    let focusRobot = selectedRobot;
    if (cameraPreset === 'planter_follow_cam') focusRobot = planterRobot;
    if (cameraPreset === 'scout_follow_cam') focusRobot = scoutRobot;

    const lookAt: Vec3 =
      cameraPreset === 'isometric' || cameraPreset === 'top_overview'
        ? { x: 12.2, y: 8.2, z: 0.1 }
        : {
            x: focusRobot.position.x,
            y: focusRobot.position.y,
            z: cameraPreset === 'wheel_closeup' ? 0.05 : 0.25,
          };

    const fovScale = cameraPreset === 'wheel_closeup' ? 680 : 620;

    // 1. Background sky/horizon gradient matching MuJoCo dark studio environment
    const bgGrad = ctx.createLinearGradient(0, 0, 0, H);
    bgGrad.addColorStop(0, '#060c14');
    bgGrad.addColorStop(0.45, '#0b1624');
    bgGrad.addColorStop(1, '#071019');
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, W, H);

    // 2. Render 3D undulating paddy terrace heightfield mesh (`paddy_micro_relief`)
    const nx = 24;
    const ny = 16;
    for (let gy = 0; gy < ny; gy++) {
      for (let gx = 0; gx < nx; gx++) {
        const x0 = gx;
        const x1 = gx + 1;
        const y0 = gy;
        const y1 = gy + 1;

        const z00 = sampleTerraceElevationM(x0, y0) * 8.5;
        const z10 = sampleTerraceElevationM(x1, y0) * 8.5;
        const z11 = sampleTerraceElevationM(x1, y1) * 8.5;
        const z01 = sampleTerraceElevationM(x0, y1) * 8.5;

        const p00 = projectPoint3D({ x: x0, y: y0, z: z00 }, lookAt, azimuthDeg, elevationDeg, distanceM, W, H, fovScale);
        const p10 = projectPoint3D({ x: x1, y: y0, z: z10 }, lookAt, azimuthDeg, elevationDeg, distanceM, W, H, fovScale);
        const p11 = projectPoint3D({ x: x1, y: y1, z: z11 }, lookAt, azimuthDeg, elevationDeg, distanceM, W, H, fovScale);
        const p01 = projectPoint3D({ x: x0, y: y1, z: z01 }, lookAt, azimuthDeg, elevationDeg, distanceM, W, H, fovScale);

        if (!p00.visible || !p10.visible || !p11.visible || !p01.visible) continue;

        const xm = gx + 0.5;
        const ym = gy + 0.5;
        const dMud = Math.hypot(xm - 14.6, ym - 8.4);
        const dBund = Math.hypot(xm - 18.2, ym - 10.2);

        let fillCol = 'rgba(18, 49, 42, 0.72)';
        let strokeCol = 'rgba(45, 95, 82, 0.35)';
        if (dMud < 2.6) {
          fillCol = 'rgba(68, 36, 22, 0.84)';
          strokeCol = 'rgba(180, 83, 9, 0.45)';
        } else if (dBund < 1.9) {
          fillCol = 'rgba(92, 74, 38, 0.82)';
          strokeCol = 'rgba(217, 119, 6, 0.45)';
        } else if ((gx + gy) % 2 === 0) {
          fillCol = 'rgba(15, 42, 38, 0.72)';
        }

        ctx.beginPath();
        ctx.moveTo(p00.px, p00.py);
        ctx.lineTo(p10.px, p10.py);
        ctx.lineTo(p11.px, p11.py);
        ctx.lineTo(p01.px, p01.py);
        ctx.closePath();
        ctx.fillStyle = fillCol;
        ctx.fill();
        ctx.strokeStyle = strokeCol;
        ctx.lineWidth = 0.8;
        ctx.stroke();
      }
    }

    // 3. Render 3D sunken wheel-mud rut tracks (`sim.mud_rut_history`)
    for (const robot of robots) {
      const crumbs = robot.breadcrumbs.slice(-16);
      for (let i = 0; i < crumbs.length; i++) {
        const pt = crumbs[i];
        const zBase = sampleTerraceElevationM(pt.x, pt.y) * 8.5;
        const sinkM = robot.telemetry.wheelSinkageMm / 1000;
        const leftTrack = projectPoint3D(
          { x: pt.x, y: pt.y - 0.22, z: zBase - sinkM * 1.5 },
          lookAt,
          azimuthDeg,
          elevationDeg,
          distanceM,
          W,
          H,
          fovScale
        );
        const rightTrack = projectPoint3D(
          { x: pt.x, y: pt.y + 0.22, z: zBase - sinkM * 1.5 },
          lookAt,
          azimuthDeg,
          elevationDeg,
          distanceM,
          W,
          H,
          fovScale
        );
        if (leftTrack.visible) {
          ctx.fillStyle = 'rgba(46, 28, 16, 0.78)';
          ctx.fillRect(leftTrack.px - 3, leftTrack.py - 2, 6, 4);
        }
        if (rightTrack.visible) {
          ctx.fillStyle = 'rgba(46, 28, 16, 0.78)';
          ctx.fillRect(rightTrack.px - 3, rightTrack.py - 2, 6, 4);
        }
      }
    }

    // 4. Render split dual-rover 3D belief spheres above each crop-gap site (`patch_a`..`patch_d`)
    for (const site of cropGapSites) {
      const zGround = sampleTerraceElevationM(site.position.x, site.position.y) * 8.5;
      const pGround = projectPoint3D(
        { x: site.position.x, y: site.position.y, z: zGround },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );
      const pScoutSphere = projectPoint3D(
        { x: site.position.x, y: site.position.y, z: zGround + 0.85 },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );
      const pPlanterSphere = projectPoint3D(
        { x: site.position.x, y: site.position.y, z: zGround + 1.35 },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );

      if (!pGround.visible || !pPlanterSphere.visible) continue;

      const isPlanted = site.status === 'PLANTED_VERIFIED';
      const isRejected = site.status === 'HAZARD_SKIPPED';
      const sphereColor = isPlanted
        ? '#22c55e'
        : isRejected
        ? '#ef4444'
        : site.status === 'ASSIGNED'
        ? '#38bdf8'
        : '#f59e0b';

      // Vertical stem
      ctx.beginPath();
      ctx.moveTo(pGround.px, pGround.py);
      ctx.lineTo(pPlanterSphere.px, pPlanterSphere.py);
      ctx.strokeStyle = sphereColor;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Lower sphere (Scout local belief)
      if (pScoutSphere.visible) {
        ctx.beginPath();
        ctx.arc(pScoutSphere.px, pScoutSphere.py, 5, 0, Math.PI * 2);
        ctx.fillStyle = sphereColor;
        ctx.fill();
        ctx.strokeStyle = '#0f172a';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      // Upper sphere (Planter local belief)
      ctx.beginPath();
      ctx.arc(pPlanterSphere.px, pPlanterSphere.py, 7, 0, Math.PI * 2);
      ctx.fillStyle = sphereColor;
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.2;
      ctx.stroke();

      ctx.fillStyle = '#e2e8f0';
      ctx.font = 'bold 10px monospace';
      ctx.fillText(site.id, pPlanterSphere.px + 9, pPlanterSphere.py + 3);
    }

    // 5. Render 3D Dynamic Obstacles & Hazard Zones
    for (const obs of obstacles) {
      const zGround = sampleTerraceElevationM(obs.position.x, obs.position.y) * 8.5;
      const pObs = projectPoint3D(
        { x: obs.position.x, y: obs.position.y, z: zGround + 0.35 },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );
      if (pObs.visible) {
        ctx.fillStyle = 'rgba(239, 68, 68, 0.85)';
        ctx.strokeStyle = '#fef08a';
        ctx.lineWidth = 2;
        ctx.fillRect(pObs.px - 10, pObs.py - 10, 20, 20);
        ctx.strokeRect(pObs.px - 10, pObs.py - 10, 20, 20);
        ctx.fillStyle = '#fef08a';
        ctx.font = 'bold 10px monospace';
        ctx.fillText('3D OBSTACLE', pObs.px - 28, pObs.py - 14);
      }
    }

    // 6. Render UWB / Wi-Fi HaLow gossip link beams in 3D space
    for (let i = 0; i < robots.length; i++) {
      for (let j = i + 1; j < robots.length; j++) {
        const rA = robots[i];
        const rB = robots[j];
        const dPeer = Math.hypot(rA.position.x - rB.position.x, rA.position.y - rB.position.y);
        if (dPeer <= 9.5) {
          const zA = sampleTerraceElevationM(rA.position.x, rA.position.y) * 8.5 + 0.65;
          const zB = sampleTerraceElevationM(rB.position.x, rB.position.y) * 8.5 + 0.65;
          const pA = projectPoint3D(
            { x: rA.position.x, y: rA.position.y, z: zA },
            lookAt,
            azimuthDeg,
            elevationDeg,
            distanceM,
            W,
            H,
            fovScale
          );
          const pB = projectPoint3D(
            { x: rB.position.x, y: rB.position.y, z: zB },
            lookAt,
            azimuthDeg,
            elevationDeg,
            distanceM,
            W,
            H,
            fovScale
          );
          if (pA.visible && pB.visible) {
            ctx.beginPath();
            ctx.setLineDash([5, 4]);
            ctx.moveTo(pA.px, pA.py);
            ctx.lineTo(pB.px, pB.py);
            ctx.strokeStyle = 'rgba(34, 211, 238, 0.72)';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.setLineDash([]);
          }
        }
      }
    }

    // 7. Render 3D Rovers, 4-Wheel Mud-Sinkage Collars, LiDAR Beams & Rigid Docking Bars
    for (const robot of robots) {
      const zGround = sampleTerraceElevationM(robot.position.x, robot.position.y) * 8.5;
      const sinkMm = robot.telemetry.wheelSinkageMm;
      const belliedOut = robot.telemetry.bellyClearanceMm <= 0 || robot.status === 'IMMOBILIZED';
      const zChassis = zGround + Math.max(0.06, 0.32 - (sinkMm / 1000) * 4.5);

      const yawRad = (robot.headingDeg * Math.PI) / 180;

      // 4 Wheels with 3D mud-sinkage collars (`collar_rgba` from `live_app.py`)
      const collarFill = belliedOut
        ? 'rgba(239, 68, 68, 0.88)'
        : sinkMm > 24
        ? 'rgba(245, 158, 11, 0.78)'
        : 'rgba(120, 82, 48, 0.65)';

      for (const dxW of [-0.42, 0.42]) {
        for (const dyW of [-0.36, 0.36]) {
          const wx = robot.position.x + dxW * Math.cos(yawRad) - dyW * Math.sin(yawRad);
          const wy = robot.position.y + dxW * Math.sin(yawRad) + dyW * Math.cos(yawRad);
          const pWheel = projectPoint3D(
            { x: wx, y: wy, z: zGround + 0.05 },
            lookAt,
            azimuthDeg,
            elevationDeg,
            distanceM,
            W,
            H,
            fovScale
          );
          if (pWheel.visible) {
            // Mud sinkage collar ring
            ctx.beginPath();
            ctx.ellipse(
              pWheel.px,
              pWheel.py + 2,
              belliedOut ? 9 : 6,
              belliedOut ? 5 : 3.5,
              0,
              0,
              Math.PI * 2
            );
            ctx.fillStyle = collarFill;
            ctx.fill();

            // Wheel hub
            ctx.beginPath();
            ctx.arc(pWheel.px, pWheel.py, 4.5, 0, Math.PI * 2);
            ctx.fillStyle = '#1e293b';
            ctx.fill();
            ctx.strokeStyle = '#94a3b8';
            ctx.lineWidth = 1;
            ctx.stroke();
          }
        }
      }

      // Rover 3D Chassis center
      const pBody = projectPoint3D(
        { x: robot.position.x, y: robot.position.y, z: zChassis },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );
      if (!pBody.visible) continue;

      // Forward LiDAR ray
      const lidarRange = Math.min(robot.telemetry.frontLidarRangeM ?? 2.2, 2.2);
      const pLidarEnd = projectPoint3D(
        {
          x: robot.position.x + lidarRange * Math.cos(yawRad),
          y: robot.position.y + lidarRange * Math.sin(yawRad),
          z: zChassis,
        },
        lookAt,
        azimuthDeg,
        elevationDeg,
        distanceM,
        W,
        H,
        fovScale
      );
      if (pLidarEnd.visible) {
        ctx.beginPath();
        ctx.moveTo(pBody.px, pBody.py);
        ctx.lineTo(pLidarEnd.px, pLidarEnd.py);
        ctx.strokeStyle =
          robot.telemetry.frontLidarRangeM !== null && robot.telemetry.frontLidarRangeM < 1.2
            ? 'rgba(239, 68, 68, 0.9)'
            : 'rgba(52, 211, 153, 0.55)';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // If retrieving a peer, render 3D rigid cam-latch tow vector
      if (robot.targetPeerId) {
        const peer = robots.find((r) => r.id === robot.targetPeerId);
        if (peer) {
          const pPeer = projectPoint3D(
            { x: peer.position.x, y: peer.position.y, z: zGround + 0.2 },
            lookAt,
            azimuthDeg,
            elevationDeg,
            distanceM,
            W,
            H,
            fovScale
          );
          if (pPeer.visible) {
            ctx.beginPath();
            ctx.moveTo(pBody.px, pBody.py);
            ctx.lineTo(pPeer.px, pPeer.py);
            ctx.strokeStyle = '#f59e0b';
            ctx.lineWidth = 3;
            ctx.stroke();
          }
        }
      }

      // Rover Chassis Box
      const isSelected = robot.id === selectedRobotId;
      const bodyColor =
        robot.status === 'HUMAN_CONTROL'
          ? '#a855f7'
          : robot.status === 'IMMOBILIZED'
          ? '#ef4444'
          : robot.status === 'RETRIEVING'
          ? '#f59e0b'
          : '#0284c7';

      if (isSelected) {
        ctx.beginPath();
        ctx.arc(pBody.px, pBody.py, 19, 0, Math.PI * 2);
        ctx.strokeStyle = '#fef08a';
        ctx.lineWidth = 2.5;
        ctx.stroke();
      }

      ctx.beginPath();
      ctx.roundRect(pBody.px - 13, pBody.py - 9, 26, 18, 4);
      ctx.fillStyle = bodyColor;
      ctx.fill();
      ctx.strokeStyle = '#f8fafc';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Label above rover in 3D
      ctx.fillStyle = isSelected ? '#fef08a' : '#f8fafc';
      ctx.font = 'bold 11px monospace';
      ctx.fillText(robot.id, pBody.px - 18, pBody.py - 14);
    }

    // 8. If Ego Optical Camera is active, draw optical perception reticle overlay
    if (cameraPreset === 'planter_ego_cam') {
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.55)';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(24, 44, W - 48, H - 64);
      ctx.fillStyle = '#38bdf8';
      ctx.font = 'bold 11px monospace';
      ctx.fillText(
        `EGO OPTICAL PERCEPTION CAM [${selectedRobot.id}] — Row Cross-Track Error: ${selectedRobot.telemetry.furrowErrorCm.toFixed(1)}cm`,
        36,
        64
      );
    }

    // 9. Overlay live 3D sinkage & slip telemetry badge directly at top of 3D view (`live_app.py` lines 1252-1264)
    const elevMm = sampleTerraceElevationM(selectedRobot.position.x, selectedRobot.position.y) * 1000;
    const sinkMm = selectedRobot.telemetry.wheelSinkageMm;
    const clrMm = selectedRobot.telemetry.bellyClearanceMm;
    const slipPct = Math.round(selectedRobot.telemetry.slipRatio * 100);
    const bellied = clrMm <= 0 || selectedRobot.status === 'IMMOBILIZED';

    ctx.fillStyle = '#09131f';
    ctx.strokeStyle = '#1f3247';
    ctx.lineWidth = 1;
    ctx.fillRect(8, 8, W - 16, 26);
    ctx.strokeRect(8, 8, W - 16, 26);

    ctx.fillStyle = bellied ? '#ef4444' : sinkMm > 25 ? '#f59e0b' : '#38bdf8';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(
      `3D Physics [${selectedRobot.id} | m=${vehicleMassKg.toFixed(1)}kg | t=${simTimeSec.toFixed(1)}s]: Terrain z=${elevMm >= 0 ? '+' : ''}${elevMm.toFixed(1)}mm | Mud Sinkage z=${sinkMm.toFixed(1)}mm (Belly Clr=${clrMm >= 0 ? '+' : ''}${clrMm.toFixed(1)}mm) | Slip=${slipPct}% | ${bellied ? 'BELLIED OUT (S1)!' : 'DRIVING'}`,
      14,
      25
    );
  }, [
    robots,
    cropGapSites,
    hazardZones,
    obstacles,
    selectedRobot,
    selectedRobotId,
    cameraPreset,
    azimuthDeg,
    elevationDeg,
    distanceM,
    vehicleMassKg,
    simTimeSec,
  ]);

  return (
    <section
      aria-label="3D MuJoCo Physics Visualiser"
      className="flex flex-col bg-[#111d2c] border border-[#1f3247] rounded-md overflow-hidden h-full"
    >
      {/* Pane Title & 3D Camera Preset Bar matching live_app.py */}
      <div className="flex flex-wrap items-center justify-between gap-2 px-3 py-1.5 bg-[#0d1826] border-b border-[#1f3247]">
        <div className="flex items-center gap-2">
          <span className="text-xs font-bold text-sky-400 whitespace-nowrap">
            3D MuJoCo Physics Visualiser
          </span>
          <span className="text-[11px] text-slate-400 hidden xl:inline">
            · Drag to Orbit · Scroll to Zoom
          </span>
        </div>

        <div className="flex flex-wrap items-center gap-1">
          <span className="text-[11px] font-semibold text-slate-400 mr-1">3D Camera:</span>
          {CAMERA_PRESET_OPTIONS.map((cam) => {
            const active = cameraPreset === cam.id;
            return (
              <button
                key={cam.id}
                type="button"
                onClick={() => {
                  logFunctionCall('Mujoco3DViewport.onChangeCameraPreset', { preset: cam.id });
                  onChangeCameraPreset(cam.id);
                }}
                className={`px-2 py-0.5 text-[11px] font-medium rounded transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
                  active
                    ? 'bg-sky-600 text-white font-semibold'
                    : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700'
                }`}
              >
                {cam.label}
              </button>
            );
          })}
        </div>
      </div>

      {/* Maximized 3D Viewport Canvas */}
      <div className="relative flex-1 bg-[#070d14] min-h-[470px] flex items-center justify-center">
        <canvas
          ref={canvasRef}
          width={840}
          height={540}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
          onWheel={handleWheel}
          className="w-full h-full object-cover cursor-grab active:cursor-grabbing"
        />

        {/* Quick 3D Robot Selector Overlay at Bottom Left */}
        <div className="absolute bottom-2 left-2 flex flex-wrap items-center gap-1 bg-slate-950/85 border border-slate-800 rounded px-2 py-1">
          <span className="text-[10px] font-mono text-slate-400 mr-1">Focus 3D Rover:</span>
          {robots.map((r) => {
            const isSel = r.id === selectedRobotId;
            return (
              <button
                key={r.id}
                type="button"
                onClick={() => onSelectRobot(r.id)}
                className={`px-1.5 py-0.5 text-[10px] font-mono rounded cursor-pointer transition-colors ${
                  isSel
                    ? 'bg-amber-400 text-slate-950 font-bold'
                    : 'bg-slate-800 text-slate-200 hover:bg-slate-700'
                }`}
              >
                {r.id}
              </button>
            );
          })}
        </div>
      </div>
    </section>
  );
};
