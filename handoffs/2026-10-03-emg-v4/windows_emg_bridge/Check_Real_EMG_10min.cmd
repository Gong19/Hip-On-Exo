@echo off
echo Ten-minute REAL Delsys loopback validation. Close other EMG applications first.
"C:\Users\YOUR_WINDOWS_USER\Documents\Codex\emg-runtime\Scripts\python.exe" -X utf8 "%~dp0validate_bridge.py" --real --duration 600
pause
