# 별도 프레임워크 없이 동작하는 단순 테스트 (실패 시 종료 코드 1)
# 실행: cmake -S . -B build -DAMR_BUILD_TESTS=ON -DAMR_BUILD_QT=OFF
#       cmake --build build --target test_3d test_cmat && (cd build && ctest)
add_executable(test_3d test_3d.cpp)
target_link_libraries(test_3d PRIVATE Core)
amr_set_warnings(test_3d)
add_test(NAME test_3d COMMAND test_3d)

add_executable(test_cmat test_cmat.cpp)
target_link_libraries(test_cmat PRIVATE Core)
amr_set_warnings(test_cmat)
add_test(NAME test_cmat COMMAND test_cmat)

add_executable(test_cmesh test_cmesh.cpp)
target_link_libraries(test_cmesh PRIVATE Core)
amr_set_warnings(test_cmesh)
add_test(NAME test_cmesh COMMAND test_cmesh)

add_executable(test_cmesh_formats test_cmesh_formats.cpp)
target_link_libraries(test_cmesh_formats PRIVATE Core)
amr_set_warnings(test_cmesh_formats)
add_test(NAME test_cmesh_formats COMMAND test_cmesh_formats)

add_executable(test_curdf test_curdf.cpp)
target_link_libraries(test_curdf PRIVATE Core)
amr_set_warnings(test_curdf)
add_test(NAME test_curdf COMMAND test_curdf)

add_executable(test_ccollision test_ccollision.cpp)
target_link_libraries(test_ccollision PRIVATE Core)
amr_set_warnings(test_ccollision)
add_test(NAME test_ccollision COMMAND test_ccollision)

add_executable(test_cdynamics test_cdynamics.cpp)
target_link_libraries(test_cdynamics PRIVATE Core)
amr_set_warnings(test_cdynamics)
add_test(NAME test_cdynamics COMMAND test_cdynamics)

# 샘플 URDF 를 창 없이 읽어 보는 스모크 테스트 (amr_main 이 만들어졌을 때만)
if(TARGET amr_main)
    add_test(NAME sample_urdf_info COMMAND amr_main --info ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    # AABB 자기충돌: 기본 자세는 겹침 없음, 팔을 아래로 접으면 base/shoulder 와 겹침 3건
    add_test(NAME sample_urdf_aabb_free COMMAND amr_main --info --aabb ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_aabb_free PROPERTIES PASS_REGULAR_EXPRESSION "AABB 겹침 후보 0건")
    add_test(NAME sample_urdf_aabb_folded COMMAND amr_main --info --aabb --joint shoulder_pitch=90 --joint elbow_pitch=-170
             ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_aabb_folded PROPERTIES PASS_REGULAR_EXPRESSION "AABB 겹침 후보 3건")
    # 중력 토크: 수평으로 뻗은 샘플 팔의 어깨/팔꿈치 토크 (손계산: 9.80665 * (1.2*0.3 + 0.8*0.85 + 0.3*1.16) = 13.6116 등)
    add_test(NAME sample_urdf_torque COMMAND amr_main --info --torque ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_torque PROPERTIES PASS_REGULAR_EXPRESSION
        "shoulder_pitch +0.000 deg +토크 +-13.6116 N\\*m.*elbow_pitch +0.000 deg +토크 +-3.6088 N\\*m")
    # 모든 관절의 하중: fixed(tool_mount)와 mimic(finger_right_slide)도 따로 나오고, 링크별 기여 내역이 붙는다
    add_test(NAME sample_gripper_loads COMMAND amr_main --info --torque ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm_gripper.urdf)
    set_tests_properties(sample_gripper_loads PROPERTIES PASS_REGULAR_EXPRESSION
        "tool_mount +\\(fixed\\).*finger_right_slide +\\(mimic finger_left_slide\\).*<- +finger_right")
    # OBB 로 다시 확인하면 3건 중 2건만 실제 충돌 (shoulder-forearm 은 AABB 만 겹침)
    add_test(NAME sample_urdf_obb_folded COMMAND amr_main --info --obb --joint shoulder_pitch=90 --joint elbow_pitch=-170
             ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
    set_tests_properties(sample_urdf_obb_folded PROPERTIES PASS_REGULAR_EXPRESSION "OBB 충돌 2건: base_link <-> upper_arm, base_link <-> forearm")
endif()
