# CoreQt: 재사용 가능한 Qt5 위젯 (프로젝트 특정 지식 없는 범용 위젯만)
# 의존 방향: CoreQt -> Core  (Core는 Qt를 절대 참조하지 않음)
# Q_OBJECT 헤더가 include/ 아래에 있으므로 AUTOMOC 이 찾도록 소스 목록에 헤더도 넣는다.
file(GLOB_RECURSE CORE_QT_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE CORE_QT_HEADERS CONFIGURE_DEPENDS include/*.h)

add_library(CoreQt STATIC ${CORE_QT_SOURCES} ${CORE_QT_HEADERS})
target_include_directories(CoreQt PUBLIC include)   # #include "CoreQt/ModelViewer.h"
target_link_libraries(CoreQt PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Core)
simul_set_warnings(CoreQt)
