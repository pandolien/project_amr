# Ubuntu 20.04 LTS 기본 CMake 버전(3.16)에 맞춤
cmake_minimum_required(VERSION 3.16)

project(project_simul
    VERSION 0.1.0
    DESCRIPTION "Project Simul: AMR + Robot Arm integrated simulation and control system"
    LANGUAGES CXX)

# ---- C++11 / gcc 기준 ----
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 정적 라이브러리(Core, CoreQt)가 공유 라이브러리(simul_core, simul_gui)에 들어가므로 위치 독립 코드로 만든다
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 소스 파일은 UTF-8(한글 주석 포함). MSVC 는 지정하지 않으면 시스템 코드 페이지로 읽는다.
if(MSVC)
    add_compile_options(/utf-8)
    # fopen, sscanf, snprintf 같은 C 함수에 대한 "안전하지 않음" 경고(C4996)를 끈다 (표준 C 함수를 그대로 쓴다)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# 산출물은 모두 build/ 아래로 (소스 트리에 lib을 두지 않음)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(CompilerWarnings)

# ---- 옵션 ----
option(SIMUL_BUILD_QT    "CoreQt(Qt5 위젯) 빌드 (Qt 5.12, LGPLv3 동적 링크)" ON)
option(SIMUL_BUILD_TESTS "테스트 빌드" OFF)

# 런타임에서 Data/ 위치를 찾기 위한 경로
set(SIMUL_DATA_DIR "${CMAKE_SOURCE_DIR}/Data")

# ---- 라이브러리 (의존 순서: Core <- CoreQt) ----
add_subdirectory(Core)


# ---- 메인 시스템 (main.cpp + 루트 src/ + 루트 include/) ----
# main.cpp 가 비어 있는 동안(구조만 잡은 단계)에는 링크가 실패하므로 simul_main 을 만들지 않는다.
file(SIZE ${CMAKE_SOURCE_DIR}/main.cpp SIMUL_MAIN_SIZE)
if(SIMUL_MAIN_SIZE GREATER 0)
    file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS src/*.cpp)
    add_executable(simul_main main.cpp ${PROJECT_SOURCES})
    target_include_directories(simul_main PRIVATE include)
    target_link_libraries(simul_main PRIVATE Core)
    target_compile_definitions(simul_main PRIVATE SIMUL_DATA_DIR="${SIMUL_DATA_DIR}")
    simul_set_warnings(simul_main)
    if(SIMUL_BUILD_QT)
        target_link_libraries(simul_main PRIVATE CoreQt)   # 3D 뷰어 창
        target_compile_definitions(simul_main PRIVATE SIMUL_HAS_QT)
    endif()
else()
    message(STATUS "main.cpp 가 비어 있어 simul_main 은 만들지 않습니다")
endif()

# ---- CoreQt (Qt 5.12) ----
if(SIMUL_BUILD_QT)
    # Ubuntu 20.04 apt: qtbase5-dev 5.12.8
    # LGPLv3 준수: 반드시 공유 라이브러리(동적 링크)로 사용, GPL 전용 모듈 금지
    find_package(Qt5 5.12 REQUIRED COMPONENTS Core Gui Widgets Network)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)
    add_subdirectory(Core/Qt)   # CoreQt: 재사용 Qt 위젯
endif()

# ---- 공유 라이브러리 simul_core (C API, Qt 없음) 와 simul_gui (3D 창, Qt)  (다른 프로그램이 불러 쓴다) ----
add_subdirectory(api)

# 주민 순찰 예제: 정해 둔 지점들을 simul_move_to 로 차례로 걷는다 (Qt 필요 없음)
add_executable(villager_patrol examples/villager_patrol.cpp)
target_link_libraries(villager_patrol PRIVATE simul_core)
simul_set_warnings(villager_patrol)

# 웹 접속 서버: 웹으로 접속한 사람들의 캐릭터를 움직인다 (web/server.js 가 띄운다, Qt 필요 없음)
add_executable(village_server examples/village_server.cpp)
target_link_libraries(village_server PRIVATE Core)
target_include_directories(village_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Core/include)
simul_set_warnings(village_server)

if(SIMUL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# 다른 Qt 프로그램이 3D 뷰(SimulView)를 붙이는 최소 예
if(SIMUL_BUILD_QT)
    add_executable(embed_host examples/embed_host.cpp)
    target_link_libraries(embed_host PRIVATE CoreQt)
    simul_set_warnings(embed_host)
endif()
