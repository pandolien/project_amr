# simul_core: 시뮬레이션 코어를 공유 라이브러리로 (C 인터페이스, Qt 필요 없음). python ctypes 가 이것을 불러 쓴다.
add_library(simul_core SHARED src/simul_api.cpp)
target_include_directories(simul_core PUBLIC include)
target_compile_definitions(simul_core PRIVATE SIMUL_API_BUILD)
target_link_libraries(simul_core PRIVATE Core)
set_target_properties(simul_core PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    WINDOWS_EXPORT_ALL_SYMBOLS OFF)
simul_set_warnings(simul_core)

# simul_gui: 3D 창 (Qt 필요). simul_core 의 C 인터페이스만 써서 시뮬레이션을 읽어 그린다 (Qt 는 simul_core 로 새지 않는다).
if(SIMUL_BUILD_QT AND TARGET CoreQt)
    add_library(simul_gui SHARED src/simul_gui.cpp)
    target_include_directories(simul_gui PUBLIC include)
    target_compile_definitions(simul_gui PRIVATE SIMUL_GUI_BUILD)
    target_link_libraries(simul_gui PRIVATE simul_core CoreQt)
    set_target_properties(simul_gui PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
    simul_set_warnings(simul_gui)
endif()
