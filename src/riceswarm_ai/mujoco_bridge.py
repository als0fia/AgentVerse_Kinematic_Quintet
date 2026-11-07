"""Optional MuJoCo visualisation bridge for 6-Robot RiceSwarm simulation."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from .core import RiceSwarmSimulation, Field


def build_mjcf(field: Field) -> str:
    parts = [
        '<mujoco model="riceswarm_china_paddy_6robots">',
        '<compiler angle="radian"/>',
        '<option timestep="0.02" gravity="0 0 -9.81"/>',
        '<worldbody>',
        f'<geom name="paddy_ground" type="plane" pos="{field.width/2} {field.height/2} 0" '
        f'size="{field.width/2+1} {field.height/2+1} 0.1" rgba="0.32 0.45 0.30 1"/>',
    ]
    for (x, y), terrain in sorted(field.truth.items()):
        if terrain.value == "blocked":
            parts.append(
                f'<geom name="bund_dike_{x}_{y}" type="box" '
                f'pos="{x+0.5} {y+0.5} 0.20" size="0.45 0.45 0.20" '
                'rgba="0.45 0.38 0.28 1"/>'
            )
        if (x, y) in field.crop_targets:
            parts.append(
                f'<geom name="crop_{x}_{y}" type="cylinder" '
                f'pos="{x+0.5} {y+0.5} 0.04" size="0.18 0.04" '
                'rgba="0.25 0.75 0.35 1" contype="0" conaffinity="0"/>'
            )
    colors = [
        "0.2 0.5 0.9 1",
        "0.9 0.4 0.2 1",
        "0.3 0.8 0.4 1",
        "0.8 0.3 0.8 1",
        "0.9 0.8 0.2 1",
        "0.2 0.8 0.8 1",
    ]
    for idx, (name, x, y) in enumerate([
        ("RS1", 0, 1), ("RS2", 0, 4), ("RS3", 0, 8),
        ("RS4", 11, 1), ("RS5", 11, 5), ("RS6", 11, 8),
    ]):
        parts.append(
            f'<body name="robot_{name}" mocap="true" pos="{x+0.5} {y+0.5} 0.21">'
            f'<geom name="shell_{name}" type="box" size="0.22 0.18 0.12" '
            f'rgba="{colors[idx]}" contype="0" conaffinity="0"/>'
            '</body>'
        )
    parts.append("</worldbody></mujoco>")
    xml = "\n".join(parts)
    ET.fromstring(xml)
    return xml


class MujocoBridge:
    def __init__(self, sim: RiceSwarmSimulation):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo unavailable; install with `pip install mujoco`") from exc
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(sim.field))
        self.data = mujoco.MjData(self.model)
        self.ids: dict[str, int] = {}
        for name in sim.robots:
            body_id = self.model.body(f"robot_{name}").id
            mocap_id = int(self.model.body_mocapid[body_id])
            if mocap_id < 0:
                raise RuntimeError(f"No mocap body for robot {name}")
            self.ids[name] = mocap_id
        self.update(sim)

    def update(self, sim: RiceSwarmSimulation) -> None:
        for name, robot in sim.robots.items():
            x, y = robot.pos
            self.data.mocap_pos[self.ids[name]] = (x + 0.5, y + 0.5, 0.21)
        for target in sim.field.weeded:
            gid = self.model.geom(f"crop_{target[0]}_{target[1]}").id
            self.model.geom_rgba[gid] = (0.1, 0.5, 0.2, 1.0)
        self.mujoco.mj_forward(self.model, self.data)
