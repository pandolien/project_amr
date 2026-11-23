@echo off
rem 에디터 실행: 처음 한 번 .venv 를 만들고 PyQt5 5.12 를 설치한 뒤 simul_editor.py 를 실행한다.
rem 빌드한 simul_core.dll / simul_gui.dll 이 있는 폴더를 SIMUL_LIB 로 알려 준다 (기본: ..\build\bin).
cd /d %~dp0..
if not exist .venv\Scripts\python.exe py -3 -m venv .venv || exit /b 1
.venv\Scripts\python -c "import PyQt5" 2>nul || .venv\Scripts\pip install "PyQt5>=5.12,<5.16" || exit /b 1
if "%SIMUL_LIB%"=="" set SIMUL_LIB=%CD%\build\bin
.venv\Scripts\python editor\simul_editor.py %*
