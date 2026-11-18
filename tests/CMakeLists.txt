# 별도 프레임워크 없이 동작하는 단순 테스트 (실패 시 종료 코드 1)
# 실행: cmake -S . -B build -DSIMUL_BUILD_TESTS=ON -DSIMUL_BUILD_QT=OFF
#       cmake --build build --target test_3d test_cmat && (cd build && ctest)
add_executable(test_cbase test_cbase.cpp)
target_link_libraries(test_cbase PRIVATE Core)
simul_set_warnings(test_cbase)
add_test(NAME test_cbase COMMAND test_cbase)

add_executable(test_bvh test_bvh.cpp)
target_link_libraries(test_bvh PRIVATE Core)
simul_set_warnings(test_bvh)
add_test(NAME test_bvh COMMAND test_bvh)

add_executable(test_csensor test_csensor.cpp)
target_link_libraries(test_csensor PRIVATE Core)
simul_set_warnings(test_csensor)
target_compile_definitions(test_csensor PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
add_test(NAME test_csensor COMMAND test_csensor)

add_executable(test_cworld test_cworld.cpp)
target_link_libraries(test_cworld PRIVATE Core)
simul_set_warnings(test_cworld)
target_compile_definitions(test_cworld PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
add_test(NAME test_cworld COMMAND test_cworld)

add_executable(test_curdf_scale test_curdf_scale.cpp)
target_link_libraries(test_curdf_scale PRIVATE Core)
simul_set_warnings(test_curdf_scale)
target_compile_definitions(test_curdf_scale PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
add_test(NAME test_curdf_scale COMMAND test_curdf_scale)

add_executable(test_ckinematics test_ckinematics.cpp)
target_link_libraries(test_ckinematics PRIVATE Core)
simul_set_warnings(test_ckinematics)
add_test(NAME test_ckinematics COMMAND test_ckinematics)

add_executable(test_3d test_3d.cpp)
target_link_libraries(test_3d PRIVATE Core)
simul_set_warnings(test_3d)
add_test(NAME test_3d COMMAND test_3d)

add_executable(test_cmat test_cmat.cpp)
target_link_libraries(test_cmat PRIVATE Core)
simul_set_warnings(test_cmat)
add_test(NAME test_cmat COMMAND test_cmat)

add_executable(test_cmesh test_cmesh.cpp)
target_link_libraries(test_cmesh PRIVATE Core)
simul_set_warnings(test_cmesh)
add_test(NAME test_cmesh COMMAND test_cmesh)

add_executable(test_cmesh_formats test_cmesh_formats.cpp)
target_link_libraries(test_cmesh_formats PRIVATE Core)
simul_set_warnings(test_cmesh_formats)
add_test(NAME test_cmesh_formats COMMAND test_cmesh_formats)

add_executable(test_curdf test_curdf.cpp)
target_link_libraries(test_curdf PRIVATE Core)
simul_set_warnings(test_curdf)
add_test(NAME test_curdf COMMAND test_curdf)

add_executable(test_ccollision test_ccollision.cpp)
target_link_libraries(test_ccollision PRIVATE Core)
target_compile_definitions(test_ccollision PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
simul_set_warnings(test_ccollision)
add_test(NAME test_ccollision COMMAND test_ccollision)

add_executable(test_cdynamics test_cdynamics.cpp)
target_link_libraries(test_cdynamics PRIVATE Core)
simul_set_warnings(test_cdynamics)
target_compile_definitions(test_cdynamics PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
add_test(NAME test_cdynamics COMMAND test_cdynamics)

# 샘플 URDF 를 창 없이 읽어 보는 스모크 테스트 (simul_main 이 만들어졌을 때만)
if(TARGET simul_main)
    add_test(NAME sample_urdf_info COMMAND simul_main --info ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    # AABB 자기충돌: 기본 자세는 겹침 없음, 팔을 아래로 접으면 base/shoulder 와 겹침 3건
    add_test(NAME sample_urdf_aabb_free COMMAND simul_main --info --aabb ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_aabb_free PROPERTIES PASS_REGULAR_EXPRESSION "AABB 겹침 후보 0건")
    add_test(NAME sample_urdf_aabb_folded COMMAND simul_main --info --aabb --joint shoulder_pitch=90 --joint elbow_pitch=-170
             ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_aabb_folded PROPERTIES PASS_REGULAR_EXPRESSION "AABB 겹침 후보 3건")
    # 중력 토크: 수평으로 뻗은 샘플 팔의 어깨/팔꿈치 토크 (손계산: 9.80665 * (1.2*0.3 + 0.8*0.85 + 0.3*1.16) = 13.6116 등)
    add_test(NAME sample_urdf_torque COMMAND simul_main --info --torque ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_torque PROPERTIES PASS_REGULAR_EXPRESSION
        "shoulder_pitch +0.000 deg +토크 +-13.6116 N\\*m.*elbow_pitch +0.000 deg +토크 +-3.6088 N\\*m")
    # 모든 관절의 하중: fixed(tool_mount)와 mimic(finger_right_slide)도 따로 나오고, 링크별 기여 내역이 붙는다
    add_test(NAME sample_gripper_loads COMMAND simul_main --info --torque ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm_gripper.urdf)
    set_tests_properties(sample_gripper_loads PROPERTIES PASS_REGULAR_EXPRESSION
        "tool_mount +\\(fixed\\).*finger_right_slide +\\(mimic finger_left_slide\\).*<- +finger_right")
    # OBB 로 다시 확인하면 3건 중 2건만 실제 충돌 (shoulder-forearm 은 AABB 만 겹침)
    add_test(NAME sample_urdf_obb_folded COMMAND simul_main --info --obb --joint shoulder_pitch=90 --joint elbow_pitch=-170
             ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_obb_folded PROPERTIES PASS_REGULAR_EXPRESSION "OBB 충돌 2건: base_link <-> upper_arm, base_link <-> forearm")
endif()

# 프로젝트 소스에 C++ 표준 라이브러리(std::, 표준 헤더)가 없어야 한다
add_test(NAME no_std COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_SOURCE_DIR} -P ${CMAKE_SOURCE_DIR}/cmake/CheckNoStd.cmake)

# 다른 Qt 프로그램에 붙이는 3D 뷰 부품 (Qt 가 켜져 있을 때만, QT_QPA_PLATFORM=offscreen)
if(SIMUL_BUILD_QT)
    add_executable(test_simulview test_simulview.cpp)
    target_link_libraries(test_simulview PRIVATE CoreQt)
    target_compile_definitions(test_simulview PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
    simul_set_warnings(test_simulview)
    add_test(NAME test_simulview COMMAND test_simulview)
endif()

add_executable(test_curdf_write test_curdf_write.cpp)
target_link_libraries(test_curdf_write PRIVATE Core)
simul_set_warnings(test_curdf_write)
target_compile_definitions(test_curdf_write PRIVATE SIMUL_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/Data")
add_test(NAME test_curdf_write COMMAND test_curdf_write)
