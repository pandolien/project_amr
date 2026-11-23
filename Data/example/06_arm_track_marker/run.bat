@echo off
rem Project run script: creates the venv (first time), installs requirements.txt, runs main.py.
rem   run.bat                 3D window
rem   run.bat --no-window     no window (5 seconds)
setlocal
cd /d "%~dp0"
set "SIMUL_LIB=%~dp0Simul"
if not exist "venv\Scripts\python.exe" (
    echo [venv] creating venv ...
    where py >nul 2>nul
    if not errorlevel 1 (py -3 -m venv venv) else (python -m venv venv)
    if errorlevel 1 (
        echo [venv] failed. Install Python 3 and try again.
        exit /b 1
    )
)
if exist requirements.txt "venv\Scripts\python.exe" -m pip install -q --disable-pip-version-check -r requirements.txt
"venv\Scripts\python.exe" main.py %*
exit /b %errorlevel%
