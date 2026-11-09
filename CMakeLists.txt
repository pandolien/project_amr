# Ubuntu 20.04 LTS 기본 CMake 버전(3.16)에 맞춤
cmake_minimum_required(VERSION 3.16)

project(project_amr
    VERSION 0.1.0
    DESCRIPTION "AMR + Robot Arm integrated control system"
    LANGUAGES CXX)

# ---- C++11 / gcc 기준 ----
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 산출물은 모두 build/ 아래로 (소스 트리에 lib을 두지 않음)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(CompilerWarnings)

# ---- 옵션 ----
option(AMR_BUILD_GUI   "Qt5 GUI 빌드 (Qt 5.12, LGPLv3 동적 링크)" ON)
option(AMR_BUILD_TESTS "테스트 빌드" OFF)

# 런타임에서 Data/ 위치를 찾기 위한 경로
set(AMR_DATA_DIR "${CMAKE_SOURCE_DIR}/Data")

# ---- 라이브러리 (의존 순서: Core <- CoreQt <- Gui) ----
add_subdirectory(Core)

# ---- 메인 시스템 (main.cpp + 루트 src/ + 루트 include/) ----
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(amr_main main.cpp ${PROJECT_SOURCES})
target_include_directories(amr_main PRIVATE include)
target_link_libraries(amr_main PRIVATE Core)
target_compile_definitions(amr_main PRIVATE AMR_DATA_DIR="${AMR_DATA_DIR}")
amr_set_warnings(amr_main)

# ---- GUI (Qt 5.12) ----
if(AMR_BUILD_GUI)
    # Ubuntu 20.04 apt: qtbase5-dev 5.12.8
    # LGPLv3 준수: 반드시 공유 라이브러리(동적 링크)로 사용, GPL 전용 모듈 금지
    find_package(Qt5 5.12 REQUIRED COMPONENTS Core Gui Widgets)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)
    add_subdirectory(Core/Qt)   # CoreQt: 재사용 Qt 위젯
    add_subdirectory(Gui)
endif()

if(AMR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
