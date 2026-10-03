@echo off
echo Close the existing EMG application. Wake sensors before this real-device check.
"C:\Users\YOUR_WINDOWS_USER\Documents\Codex\emg-runtime\Scripts\python.exe" -X utf8 "%~dp0validate_bridge.py" --real --duration 10
pause
