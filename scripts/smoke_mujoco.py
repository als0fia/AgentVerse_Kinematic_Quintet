#!/usr/bin/env python3
"""OPTIONAL MuJoCo toy-physics preflight; never an Isaac or PaddyPatch pass.

Run: .venv/Scripts/python.exe scripts/smoke_mujoco.py  (Windows)
     .venv/bin/python scripts/smoke_mujoco.py          (Linux/macOS)
Only depends on the explicitly installed official mujoco Python package.
"""
from __future__ import annotations

from math import isclose


MODEL_XML = """
<mujoco model="optional-local-physics-smoke">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="test_cart" pos="0 0 0">
      <joint name="x_slide" type="slide" axis="1 0 0" damping="0.1"/>
      <geom type="sphere" size="0.1" mass="1" rgba=".3 .6 .9 1"/>
    </body>
  </worldbody>
  <actuator><motor name="test_motor" joint="x_slide" gear="1"/></actuator>
  <sensor><jointpos name="x_sensor" joint="x_slide"/></sensor>
</mujoco>
"""


def main() -> None:
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(
            "MuJoCo not installed; optional preflight only. "
            "Use the verified .venv Python to run: python -m pip install mujoco"
        ) from exc

    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "x_sensor")
    if sensor_id < 0:
        raise AssertionError("toy joint-position sensor missing")
    sensor_address = model.sensor_adr[sensor_id]

    def episode() -> tuple[float, float, float]:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        initial = float(data.sensordata[sensor_address])
        data.ctrl[0] = 1.0
        for _ in range(100):
            mujoco.mj_step(model, data)
        return initial, float(data.sensordata[sensor_address]), float(data.time)

    first = episode()
    second = episode()
    if not (first[1] > first[0] and first[2] > 0):
        raise AssertionError(f"physics did not move sensed toy joint: {first}")
    if not all(isclose(a, b, abs_tol=1e-10) for a, b in zip(first, second)):
        raise AssertionError(f"toy reset/replay differs: first={first}, second={second}")
    print(f"PASS optional MuJoCo toy physics {mujoco.__version__}: "
          f"joint={first[0]:.5f}->{first[1]:.5f} rad, "
          f"sim_time_s={first[2]:.3f}; deterministic reset PASS")
    print("NOT TESTED: Isaac Sim, rover wheels, real soil sensing, physical seed drop or team demo")


if __name__ == "__main__":
    main()
