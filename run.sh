#!/usr/bin/env bash
# 필요한 패키지를 확인/설치하고, 빌드한 뒤 실행한다 (Ubuntu 20.04 기준, apt 사용).
#
# 사용법: ./run.sh [옵션] [-- simul_main 에 넘길 인자...]
#   --test        실행 대신 테스트(ctest)만 돌린다
#   --no-qt       Qt(CoreQt)를 빌드하지 않는다 (qtbase5-dev 설치도 건너뜀)
#   --no-install  패키지를 설치하지 않는다 (없으면 목록만 알려주고 종료)
#   --clean       build/ 를 지우고 처음부터 다시 구성한다
#   --debug       Debug 로 빌드한다 (기본 Release)
#   --web         3D 창 없이 웹 접속 서버만 켠다 (http://localhost:8080)
#   --no-web      웹 접속 서버를 같이 켜지 않는다 (기본: Node 가 있으면 같이 켠다)
#   -h, --help    도움말
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$ROOT/build"
BUILD_TYPE=Release
USE_QT=ON
DO_INSTALL=1
DO_TEST=0
DO_CLEAN=0
USE_WEB=1
WEB_ONLY=0
APP_ARGS=()

usage() { sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --test) DO_TEST=1 ;;
        --no-qt) USE_QT=OFF ;;
        --no-install) DO_INSTALL=0 ;;
        --clean) DO_CLEAN=1 ;;
        --debug) BUILD_TYPE=Debug ;;
        --web) WEB_ONLY=1 ;;
        --no-web) USE_WEB=0 ;;
        -h|--help) usage; exit 0 ;;
        --) shift; APP_ARGS=("$@"); break ;;
        *) echo "알 수 없는 옵션: $1 (./run.sh --help)" >&2; exit 2 ;;
    esac
    shift
done

log() { echo "[run.sh] $*"; }

# ---- 1. 필요한 패키지 확인 ------------------------------------------------
# 명령어 형태로 확인하는 것과, dpkg 패키지로 확인하는 것을 나누어 부족한 apt 패키지 목록을 만든다.
MISSING=()
need_cmd() { command -v "$1" >/dev/null 2>&1 || MISSING+=("$2"); }
need_pkg() { dpkg -s "$1" >/dev/null 2>&1 || MISSING+=("$1"); }

need_cmd g++ build-essential
need_cmd make build-essential
need_cmd cmake cmake
need_pkg liblapack-dev
need_pkg libtinyxml2-dev   # 없어도 포함본으로 빌드되지만, 시스템 것을 쓰는 것이 기본
[ "$USE_QT" = ON ] && need_pkg qtbase5-dev

if ! command -v dpkg >/dev/null 2>&1; then
    log "dpkg 가 없는 배포판입니다. 아래를 직접 설치한 뒤 다시 실행하세요: g++ make cmake LAPACK tinyxml2 Qt5(필요 시)"
    MISSING=()   # 자동 설치는 apt 계열에서만 한다
fi

if [ ${#MISSING[@]} -gt 0 ]; then
    # 중복 제거
    UNIQUE=($(printf '%s\n' "${MISSING[@]}" | sort -u))
    log "설치가 필요한 패키지: ${UNIQUE[*]}"
    if [ "$DO_INSTALL" = 0 ]; then
        log "--no-install 이므로 설치하지 않습니다. 설치 후 다시 실행하세요: sudo apt install ${UNIQUE[*]}"
        exit 1
    fi
    SUDO=""
    if [ "$(id -u)" -ne 0 ]; then
        command -v sudo >/dev/null 2>&1 || { log "root 권한이 필요합니다 (sudo 없음). 직접 설치하세요: apt install ${UNIQUE[*]}"; exit 1; }
        SUDO="sudo"
    fi
    $SUDO apt-get update
    $SUDO apt-get install -y "${UNIQUE[@]}"
else
    log "필요한 패키지가 모두 설치되어 있습니다."
fi

# CMake 3.16 이상 확인 (Ubuntu 20.04 기본은 3.16)
CMAKE_VER="$(cmake --version | head -n1 | sed 's/[^0-9.]//g')"
if [ "$(printf '%s\n' 3.16 "$CMAKE_VER" | sort -V | head -n1)" != "3.16" ]; then
    log "CMake $CMAKE_VER 은 너무 낮습니다 (3.16 이상 필요)." >&2
    exit 1
fi

# ---- 2. 구성 + 빌드 ---------------------------------------------------------
if [ "$DO_CLEAN" = 1 ]; then
    log "build/ 를 정리합니다."
    # 폴더 자체는 .gitkeep 때문에 남겨 둔다
    find "$BUILD" -mindepth 1 ! -name .gitkeep -delete 2>/dev/null || true
fi

log "구성: Qt=$USE_QT, $BUILD_TYPE"
cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DSIMUL_BUILD_QT="$USE_QT" -DSIMUL_BUILD_TESTS=ON

JOBS="$(nproc 2>/dev/null || echo 2)"
log "빌드 (-j$JOBS)"
cmake --build "$BUILD" -j"$JOBS"

# ---- 3. 실행 ----------------------------------------------------------------
if [ "$DO_TEST" = 1 ]; then
    log "테스트 실행"
    (cd "$BUILD" && ctest --output-on-failure)
    exit $?
fi

# ---- 웹 접속 서버 (브라우저로 접속, 최대 30명) ----------------------------------------
WEB_PID=""
if [ "$USE_WEB" = 1 ] || [ "$WEB_ONLY" = 1 ]; then
    if [ -x "$BUILD/bin/village_server" ] && command -v node >/dev/null 2>&1 && command -v npm >/dev/null 2>&1; then
        [ -f "$ROOT/web/node_modules/ws/package.json" ] && : || (log "웹 패키지를 설치합니다 (npm install)"; cd "$ROOT/web" && npm install --no-audit --no-fund) || true
        if [ -f "$ROOT/web/node_modules/ws/package.json" ]; then
            if [ "$WEB_ONLY" = 1 ]; then
                log "웹 서버: http://localhost:8080 (Ctrl+C 로 끝냄)"
                cd "$ROOT/web" && exec node server.js --server "$BUILD/bin/village_server"
            fi
            # 3D 앱이 엔진 포트(--serve)를 열고 웹 게이트웨이가 거기에 붙는다: 웹 접속자가 엔진과 3D 화면에도 나타난다
            APP_ARGS+=(--serve 9000)
            (cd "$ROOT/web" && exec node server.js --connect 127.0.0.1:9000) &
            WEB_PID=$!
            trap '[ -n "$WEB_PID" ] && kill "$WEB_PID" 2>/dev/null || true' EXIT
            log "웹 서버를 켰습니다: http://localhost:8080 (같은 네트워크의 다른 기기는 이 컴퓨터 주소:8080)"
        fi
    else
        log "웹 서버를 건너뜁니다 (Node.js/npm 또는 village_server 가 없습니다). 끄려면 --no-web"
        [ "$WEB_ONLY" = 1 ] && exit 1
    fi
fi

APP="$BUILD/bin/simul_main"
if [ -x "$APP" ]; then
    log "실행: $APP ${APP_ARGS[*]:-}"
    "$APP" "${APP_ARGS[@]+"${APP_ARGS[@]}"}"
    exit $?
fi

# main.cpp 가 아직 비어 있으면 simul_main 이 만들어지지 않는다 -> 테스트로 대신 확인
log "실행할 프로그램(simul_main)이 아직 없습니다 (main.cpp 가 비어 있음). 대신 테스트를 실행합니다."
(cd "$BUILD" && ctest --output-on-failure)
