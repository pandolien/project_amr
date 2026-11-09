# 별도 프레임워크 없이 동작하는 단순 테스트 (실패 시 종료 코드 1)
# 실행: cmake -S . -B build -DAMR_BUILD_TESTS=ON -DAMR_BUILD_QT=OFF
#       cmake --build build --target test_3d && (cd build && ctest)
add_executable(test_3d test_3d.cpp)
target_link_libraries(test_3d PRIVATE Core)
amr_set_warnings(test_3d)
add_test(NAME test_3d COMMAND test_3d)
