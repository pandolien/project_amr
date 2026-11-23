# simul_editor: 프로젝트(main.py + URDF + 오브젝트별 python)를 만들고 고치고 돌려 보는 C++(Qt) 프로그램.
# 3D View 는 CoreQt 의 SimulView, 오브젝트를 움직이는 일만 python 이 DLL(simul_core)을 불러서 한다.
add_executable(simul_editor
    main.cpp
    EditorWindow.h EditorWindow.cpp
    CodeEdit.h CodeEdit.cpp
    PyConsole.h PyConsole.cpp
    ProjectTools.h ProjectTools.cpp)
target_link_libraries(simul_editor PRIVATE CoreQt)
target_compile_definitions(simul_editor PRIVATE SIMUL_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_dependencies(simul_editor simul_core simul_gui)   # 새 프로젝트에 복사할 DLL 이 같이 만들어지게
simul_set_warnings(simul_editor)
