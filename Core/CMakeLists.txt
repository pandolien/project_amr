# Core: CMat, 3d, CDynamics, CSimul, CSock
# 처음에는 단일 라이브러리로 시작하고, 커지면 모듈별 타겟으로 분리
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(Core STATIC ${CORE_SOURCES})
target_include_directories(Core PUBLIC include)   # #include "CMat/CMat.h"
amr_set_warnings(Core)
