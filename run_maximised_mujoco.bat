@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" scripts\run_mujoco_paddypatch.py --dashboard %*
