@echo off
rem ============================================================================
rem  Install missing tools, build, and run (Windows 10/11, Visual Studio + vcpkg).
rem
rem  Usage: run.bat [options] [-- arguments for simul_main]
rem    --test        run the tests (ctest) instead of the program
rem    --no-qt       do not build the 3D window (console mode only, no Qt needed)
rem    --no-install  never install anything; only report what is missing
rem    --clean       wipe build\ and configure from scratch
rem    --web         start only the web server (http://localhost:8080), no 3D window
rem    --no-web      do not start the web server (default: it starts in its own window)
rem    --debug       build Debug (default Release; needs a much longer first vcpkg build)
rem    -h, --help    show this help
rem
rem  Messages are in English on purpose: cmd.exe mangles UTF-8 text unless the
rem  console code page is changed.
rem ============================================================================
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"

set "BUILD_TYPE=Release"
set "USE_QT=ON"
set "DO_TEST=0"
set "DO_CLEAN=0"
set "DO_INSTALL=1"
set "APP_ARGS="
set "USE_WEB=1"
set "WEB_ONLY=0"
set "WEB_OK=0"

:parse
if "%~1"=="" goto parsed
if /i "%~1"=="--test"       (set "DO_TEST=1"    & shift & goto parse)
if /i "%~1"=="--qt"         (set "USE_QT=ON"    & shift & goto parse)
if /i "%~1"=="--no-qt"      (set "USE_QT=OFF"   & shift & goto parse)
if /i "%~1"=="--no-install" (set "DO_INSTALL=0" & shift & goto parse)
if /i "%~1"=="--clean"      (set "DO_CLEAN=1"   & shift & goto parse)
if /i "%~1"=="--debug"      (set "BUILD_TYPE=Debug" & shift & goto parse)
if /i "%~1"=="--web"        (set "WEB_ONLY=1"   & shift & goto parse)
if /i "%~1"=="--no-web"     (set "USE_WEB=0"    & shift & goto parse)
if /i "%~1"=="-h"           goto usage
if /i "%~1"=="--help"       goto usage
if "%~1"=="--" (shift & goto collect)
echo [run.bat] Unknown option: %~1  (run.bat --help)
exit /b 2

:collect
if "%~1"=="" goto parsed
set "APP_ARGS=!APP_ARGS! %1"
shift
goto collect

:usage
echo Usage: run.bat [options] [-- arguments for simul_main]
echo   --test        run the tests (ctest) instead of the program
echo   --no-qt       do not build the 3D window ^(console mode only, no Qt needed^)
echo   --no-install  never install anything; only report what is missing
echo   --clean       wipe build\ and configure from scratch
echo   --debug       build Debug (default Release)
echo   --web         start only the web server ^(http://localhost:8080^), no 3D window
echo   --no-web      do not start the web server ^(default: it starts in its own window^)
exit /b 0

:parsed

rem ---- 1. CMake and Git -------------------------------------------------------
call :need_tool cmake  Kitware.CMake "%ProgramFiles%\CMake\bin"
if errorlevel 1 exit /b 1
call :need_tool git    Git.Git       "%ProgramFiles%\Git\cmd"
if errorlevel 1 exit /b 1

rem ---- 2. C++ compiler (Visual Studio Build Tools) ------------------------------
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set "VS_PATH="
if exist "%VSWHERE%" (
    for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%I"
)
if not defined VS_PATH (
    echo [run.bat] Visual Studio with the "Desktop development with C++" workload was not found.
    if "%DO_INSTALL%"=="0" (
        echo [run.bat] Install Visual Studio Build Tools 2019 or newer, then run this again.
        exit /b 1
    )
    where winget >nul 2>nul
    if errorlevel 1 (
        echo [run.bat] winget is not available. Install Visual Studio Build Tools manually, then run this again.
        exit /b 1
    )
    echo [run.bat] Build Tools is a large download ^(several GB^).
    choice /c YN /m "Install Visual Studio 2022 Build Tools now"
    if errorlevel 2 exit /b 1
    winget install -e --id Microsoft.VisualStudio.2022.BuildTools --accept-package-agreements --accept-source-agreements --override "--passive --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    echo [run.bat] Installation finished. Open a NEW terminal and run run.bat again.
    exit /b 1
)
echo [run.bat] Visual Studio: %VS_PATH%

rem ---- 3. vcpkg + clapack (LAPACK: dgesvd for CMat) -----------------------------------
if defined VCPKG_ROOT (set "VCPKG_DIR=%VCPKG_ROOT%") else (set "VCPKG_DIR=%~dp0.vcpkg")
rem Qt cannot be built in a path that contains a space (the build fails early), so when the project folder has one,
rem keep vcpkg in a space-free folder on the system drive instead. Set VCPKG_ROOT yourself to choose another place.
if not defined VCPKG_ROOT (
    set "VCPKG_CHECK=!VCPKG_DIR!"
    if not "!VCPKG_CHECK: =!"=="!VCPKG_CHECK!" (
        set "VCPKG_DIR=%SystemDrive%\simul-vcpkg"
        rem keep using the folder created before the rename (a Qt build there already exists)
        if exist "%SystemDrive%\amr-vcpkg\vcpkg.exe" set "VCPKG_DIR=%SystemDrive%\amr-vcpkg"
        echo [run.bat] The project path contains a space, so vcpkg will live in !VCPKG_DIR! instead.
    )
)
if not exist "%VCPKG_DIR%\vcpkg.exe" (
    if "%DO_INSTALL%"=="0" (
        echo [run.bat] vcpkg not found at %VCPKG_DIR%. Run without --no-install to set it up.
        exit /b 1
    )
    if not exist "%VCPKG_DIR%\bootstrap-vcpkg.bat" (
        echo [run.bat] Cloning vcpkg into %VCPKG_DIR%
        git clone --depth 1 https://github.com/microsoft/vcpkg.git "%VCPKG_DIR%"
        if errorlevel 1 (echo [run.bat] git clone failed & exit /b 1)
    )
    call "%VCPKG_DIR%\bootstrap-vcpkg.bat" -disableMetrics
    if errorlevel 1 (echo [run.bat] vcpkg bootstrap failed & exit /b 1)
)
rem Release-only triplet: skips the Debug builds, which roughly halves the vcpkg build time.
rem --debug needs the normal triplet (and a much longer first build).
if /i "%BUILD_TYPE%"=="Debug" (set "TRIPLET=x64-windows") else (set "TRIPLET=x64-windows-release")
"%VCPKG_DIR%\vcpkg.exe" list clapack | findstr /i /c:"clapack:%TRIPLET%" >nul
if errorlevel 1 (
    if "%DO_INSTALL%"=="0" (
        echo [run.bat] clapack is not installed. Run: "%VCPKG_DIR%\vcpkg.exe" install clapack:%TRIPLET%
        exit /b 1
    )
    echo [run.bat] Installing clapack ^(LAPACK^) with vcpkg ^(first time takes a while^)
    "%VCPKG_DIR%\vcpkg.exe" install clapack:%TRIPLET%
    if errorlevel 1 (echo [run.bat] vcpkg install failed & exit /b 1)
) else (
    echo [run.bat] clapack is already installed.
)
rem DLLs of installed packages, so the programs can start even outside the build folder
set "PATH=%VCPKG_DIR%\installed\%TRIPLET%\bin;%PATH%"

rem ---- Qt (needed for the 3D window) ----------------------------------------------
if "%USE_QT%"=="ON" (call :ensure_qt & if errorlevel 1 exit /b 1)
rem a Qt installed outside vcpkg: its DLLs must be on PATH to run the program
if defined CMAKE_PREFIX_PATH if exist "%CMAKE_PREFIX_PATH%\bin" set "PATH=%CMAKE_PREFIX_PATH%\bin;%PATH%"

rem ---- 4. Configure + build -----------------------------------------------------
if "%DO_CLEAN%"=="1" (
    echo [run.bat] Cleaning build\
    if exist build (
        for /d %%D in (build\*) do rmdir /s /q "%%D"
        for %%F in (build\*) do if /i not "%%~nxF"==".gitkeep" del /q "%%F"
    )
)
echo [run.bat] Configure: Qt=%USE_QT%, %BUILD_TYPE%
cmake -S . -B build -A x64 -DCMAKE_TOOLCHAIN_FILE="%VCPKG_DIR%\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=%TRIPLET% -DSIMUL_BUILD_QT=%USE_QT% -DSIMUL_BUILD_TESTS=ON
if errorlevel 1 (echo [run.bat] CMake configure failed & exit /b 1)

echo [run.bat] Build
cmake --build build --config %BUILD_TYPE% --parallel
if errorlevel 1 (echo [run.bat] Build failed & exit /b 1)

rem ---- 5. Web server (browser access, up to 30 people) ---------------------------------
if "%DO_TEST%"=="1" goto runtests
if "%WEB_ONLY%"=="1" set "USE_WEB=1"
if "%USE_WEB%"=="1" call :prepare_web
if "%WEB_ONLY%"=="1" goto runweb
if "%WEB_OK%"=="1" call :start_web

rem ---- 6. Run -------------------------------------------------------------------

set "APP=build\bin\%BUILD_TYPE%\simul_main.exe"
if exist "%APP%" (
    echo [run.bat] Running %APP%%APP_ARGS%
    "%APP%"%APP_ARGS%
    exit /b !errorlevel!
)
echo [run.bat] simul_main was not built. Running the tests instead.

:runtests
pushd build
ctest -C %BUILD_TYPE% --output-on-failure
set "RC=!errorlevel!"
popd
exit /b !RC!

rem ---- web server only: run it in this window ------------------------------------------
:runweb
if not "%WEB_OK%"=="1" (
    echo [run.bat] The web server could not be prepared.
    exit /b 1
)
echo [run.bat] Web server: http://localhost:8080   Ctrl+C stops it
pushd web
node server.js --server "%VSRV%"
set "RC=!errorlevel!"
popd
exit /b !RC!

rem ---- helper: village_server.exe, Node.js and the web packages ----------------------------
:prepare_web
set "WEB_OK=0"
set "VSRV=%CD%\build\bin\%BUILD_TYPE%\village_server.exe"
if not exist "%VSRV%" (
    echo [run.bat] village_server.exe was not built, so the web server is skipped.
    exit /b 1
)
call :need_tool node OpenJS.NodeJS.LTS "%ProgramFiles%\nodejs"
if errorlevel 1 (
    echo [run.bat] Node.js is needed for the web server. Skipping it; use --no-web to hide this.
    exit /b 1
)
if not exist "web\node_modules\ws\package.json" (
    echo [run.bat] Installing the web packages with npm install
    pushd web
    call npm install --no-audit --no-fund
    set "NPMRC=!errorlevel!"
    popd
    if not "!NPMRC!"=="0" (
        echo [run.bat] npm install failed, so the web server is skipped.
        exit /b 1
    )
)
set "WEB_OK=1"
exit /b 0

rem ---- helper: start the web server in its own window ---------------------------------------
:start_web
start "Village web server" /d "%CD%\web" cmd /k node server.js --server "%VSRV%"
echo [run.bat] Web server started in its own window: http://localhost:8080
echo [run.bat] Others on the same network use http://this-PC-address:8080 ^(Windows may ask to allow Node.js through the firewall^)
exit /b 0

rem ---- helper: make sure a command exists, installing it with winget if allowed ----
:need_tool
rem %1 command, %2 winget id, %3 folder to add to PATH after installing
where %1 >nul 2>nul
if not errorlevel 1 exit /b 0
if exist "%~3\%1.exe" (set "PATH=%~3;!PATH!" & exit /b 0)
echo [run.bat] %1 was not found.
if "%DO_INSTALL%"=="0" (echo [run.bat] Install %1 and run this again. & exit /b 1)
where winget >nul 2>nul
if errorlevel 1 (echo [run.bat] winget is not available. Install %1 manually and run this again. & exit /b 1)
winget install -e --id %2 --silent --accept-package-agreements --accept-source-agreements
set "PATH=%~3;%PATH%"
where %1 >nul 2>nul
if errorlevel 1 (echo [run.bat] %1 was installed but is not on PATH yet. Open a new terminal and run again. & exit /b 1)
exit /b 0

rem ---- helper: find or install Qt 5 (for the 3D window) ---------------------------------
:ensure_qt
rem 1) a Qt you installed yourself: set CMAKE_PREFIX_PATH (or QT_DIR) to e.g. C:\Qt\5.15.2\msvc2019_64
if defined CMAKE_PREFIX_PATH exit /b 0
if defined QT_DIR (set "CMAKE_PREFIX_PATH=!QT_DIR!" & exit /b 0)
rem 2a) Qt downloaded earlier by this script (aqtinstall)
set "AQT_QT=%SystemDrive%\Qt\5.12.12\msvc2017_64"
if exist "!AQT_QT!\bin\Qt5Core.dll" (set "CMAKE_PREFIX_PATH=!AQT_QT!" & exit /b 0)
rem 2b) Qt installed through vcpkg
"%VCPKG_DIR%\vcpkg.exe" list qt5-base | findstr /i /c:"qt5-base:%TRIPLET%" >nul
if not errorlevel 1 exit /b 0
rem 3) not found: offer to download a prebuilt Qt (fast) or build it with vcpkg (slow)
if "%DO_INSTALL%"=="0" (
    echo [run.bat] Qt 5 was not found, so the 3D window will not be built ^(console mode only^).
    set "USE_QT=OFF"
    exit /b 0
)
echo [run.bat] Qt 5 is needed to show the 3D window.
echo [run.bat]   A = download a prebuilt Qt 5.12.12 with aqtinstall ^(fast, needs Python; installs to !AQT_QT!^)
echo [run.bat]   V = build Qt with vcpkg ^(Release only, no OpenSSL^): 20-40 minutes and a few GB
echo [run.bat]   N = skip; set CMAKE_PREFIX_PATH to your own Qt 5 folder and run again
choice /c AVN /m "Install Qt 5"
if errorlevel 3 (
    echo [run.bat] Skipping Qt: building console mode only.
    set "USE_QT=OFF"
    exit /b 0
)
if errorlevel 2 goto :qt_vcpkg
where python >nul 2>nul
if errorlevel 1 (
    echo [run.bat] Python was not found. Install it ^(winget install Python.Python.3.12^), open a new terminal, and run again.
    exit /b 1
)
python -m pip install --user --upgrade aqtinstall
if errorlevel 1 (echo [run.bat] pip install aqtinstall failed & exit /b 1)
python -m aqt install-qt windows desktop 5.12.12 win64_msvc2017_64 -O "%SystemDrive%\Qt"
if errorlevel 1 (echo [run.bat] aqt install-qt failed. Check the arch name with: python -m aqt list-qt windows desktop --arch 5.12.12 & exit /b 1)
if not exist "!AQT_QT!\bin\Qt5Core.dll" (echo [run.bat] Qt was not found at !AQT_QT! after the download & exit /b 1)
set "CMAKE_PREFIX_PATH=!AQT_QT!"
exit /b 0
:qt_vcpkg
"%VCPKG_DIR%\vcpkg.exe" install "qt5-base[core]:%TRIPLET%"
if errorlevel 1 (echo [run.bat] Qt install failed & exit /b 1)
exit /b 0
