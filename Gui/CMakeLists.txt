# Gui: 이 프로젝트 전용 Qt5 화면 (CoreQt 위에 얹는 계층)
file(GLOB_RECURSE GUI_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(Gui STATIC ${GUI_SOURCES})
target_include_directories(Gui PUBLIC include)
target_link_libraries(Gui PUBLIC CoreQt)

# TODO: GUI 실행 파일(amr_gui)은 main이 생기면 추가
