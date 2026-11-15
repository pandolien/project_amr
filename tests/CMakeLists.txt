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

# 샘플 URDF 를 창 없이 읽어 보는 스모크 테스트 (amr_main 이 만들어졌을 때만)
if(TARGET amr_main)
    add_test(NAME sample_urdf_info COMMAND amr_main --info ${CMAKE_SOURCE_DIR}/Data/Object/samples/arm.urdf)
endif()
