@echo off
REM Double-click me (Windows). Needs Python 3.9+ from python.org or the Microsoft Store.
cd /d "%~dp0"
where python >nul 2>nul
if errorlevel 1 (
  echo Python was not found. Install it from https://www.python.org/downloads/ ^(tick "Add python.exe to PATH"^) and run this again.
  pause
  exit /b 1
)
python -m pip install --quiet --user -r requirements.txt
python sim_demo.py
if errorlevel 1 pause
