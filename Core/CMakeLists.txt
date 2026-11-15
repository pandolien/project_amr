# Core: CMat, 3d, CMesh, CURDF, CDynamics, CSimul, CSock
# 처음에는 단일 라이브러리로 시작하고, 커지면 모듈별 타겟으로 분리
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(Core STATIC ${CORE_SOURCES})
target_include_directories(Core PUBLIC include)
# 외부에 공개하지 않는 내부 헤더 (FileUtil.h 등)
target_include_directories(Core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/internal)
# 사용: #include "CMat/lMat.h" 처럼 include/ 아래 경로로 가져온다

# ---- LAPACK: CMat(lMat::SVD, Inv) 가 dgesvd_ 를 직접 호출한다 (modified BSD 라이선스)
#   LAPACKE 는 필요 없다 (헤더 없이 dgesvd_ 를 직접 선언해서 쓴다).
#   Ubuntu 20.04: sudo apt install liblapack-dev
#   Windows     : vcpkg install clapack:x64-windows  (f2c 로 변환된 LAPACK, run.bat 이 설치함)
#                 -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake 로 구성
#   직접 지정    : -DLAPACK_LIBRARIES="<lapack>;<blas>..."
find_package(LAPACK REQUIRED)

# ---- tinyxml2: CURDF(URDF) 와 CMesh(COLLADA .dae) 의 XML 파싱 (zlib 라이선스)
#   기본: 시스템에 있으면 그것을 쓰고(Ubuntu: sudo apt install libtinyxml2-dev), 없으면
#         third_party/tinyxml2 (포함본, Windows 기본)를 함께 빌드한다.
option(AMR_USE_BUNDLED_TINYXML2 "시스템 tinyxml2 대신 third_party/tinyxml2 를 빌드" OFF)
if(NOT AMR_USE_BUNDLED_TINYXML2)
    find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
    find_library(TINYXML2_LIBRARY NAMES tinyxml2)
    if(NOT TINYXML2_INCLUDE_DIR OR NOT TINYXML2_LIBRARY)
        message(STATUS "시스템 tinyxml2 를 찾지 못해 포함본(third_party/tinyxml2)을 사용합니다")
        set(AMR_USE_BUNDLED_TINYXML2 ON)
    endif()
endif()
if(AMR_USE_BUNDLED_TINYXML2)
    add_library(amr_tinyxml2 STATIC ${CMAKE_SOURCE_DIR}/third_party/tinyxml2/tinyxml2.cpp)
    target_include_directories(amr_tinyxml2 SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/third_party/tinyxml2)
    set_target_properties(amr_tinyxml2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    set(TINYXML2_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/tinyxml2)
    set(TINYXML2_LIBRARY amr_tinyxml2)
endif()

# ---- cgltf: CMesh 의 GLB/glTF 파싱 (MIT 라이선스, third_party/cgltf 에 포함)
# SYSTEM 으로 지정해 외부 헤더에서 나오는 경고는 표시하지 않는다.
target_include_directories(Core SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/cgltf
    ${TINYXML2_INCLUDE_DIR})

target_link_libraries(Core PUBLIC ${LAPACK_LIBRARIES} ${TINYXML2_LIBRARY})

amr_set_warnings(Core)
