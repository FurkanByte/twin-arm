@echo off
REM TwinArm-5 digital twin: opens in the browser. From a phone: use the http://<IP>:8080 address shown on screen.
cd /d "%~dp0"
python server.py %*
pause
