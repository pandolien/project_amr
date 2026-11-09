# CoreQt: 재사용 가능한 Qt5 위젯 (프로젝트 특정 지식 없는 범용 위젯만)
# 의존 방향: Gui -> CoreQt -> Core  (Core는 Qt를 절대 참조하지 않음)
file(GLOB_RECURSE CORE_QT_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(CoreQt STATIC ${CORE_QT_SOURCES})
target_include_directories(CoreQt PUBLIC include)   # #include "CoreQt/CoreQt.h"
target_link_libraries(CoreQt PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Core)
amr_set_warnings(CoreQt)
