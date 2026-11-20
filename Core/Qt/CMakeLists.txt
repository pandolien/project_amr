# CoreQt: 재사용 가능한 Qt5 위젯 (프로젝트 특정 지식 없는 범용 위젯만)
# 의존 방향: CoreQt -> Core  (Core는 Qt를 절대 참조하지 않음)
# Q_OBJECT 헤더가 include/ 아래에 있으므로 AUTOMOC 이 찾도록 소스 목록에 헤더도 넣는다.
# 헤더를 새로 더했는데 링크 오류(metaObject, staticMetaObject, 시그널 함수를 찾을 수 없음)가 나면 CMake 를 다시 구성(configure)해야 moc 가 돌아간다.
# 이 파일을 고치면 다시 구성되므로 Q_OBJECT 헤더를 더한 커밋에서는 여기를 같이 고친다. (최근 추가: SimulRunner.h, PythonPanel.h, ObjectInfoPanel.h; ModelViewer.h 에 ModelSourcePath)
file(GLOB_RECURSE CORE_QT_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB_RECURSE CORE_QT_HEADERS CONFIGURE_DEPENDS include/*.h)

add_library(CoreQt STATIC ${CORE_QT_SOURCES} ${CORE_QT_HEADERS})
target_include_directories(CoreQt PUBLIC include)   # #include "CoreQt/ModelViewer.h"
target_link_libraries(CoreQt PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets Core)
simul_set_warnings(CoreQt)
