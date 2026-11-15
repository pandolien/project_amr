# project_amr

AMR(이동 로봇) 위에 로봇암을 연결하여 통합 제어하는 시스템.

## 개발 환경

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 20.04 LTS |
| 컴파일러 | gcc (9.x) |
| 빌드 | CMake 3.16+ |
| 언어 | C++11 |
| Qt | Qt 5.12 (LGPLv3, 동적 링크) |

## 디렉토리 구조

```
project_amr/
├── CMakeLists.txt        # 전체 빌드
├── main.cpp              # 메인 시스템
├── Data/Object/          # 오브젝트 관련 자료
├── Core/                 # 라이브러리화되는 핵심 코드 (순수 C++11, Qt 금지)
│   ├── CMakeLists.txt
│   ├── include/          # CMat, 3d, CDynamics, CSimul, CSock
│   ├── src/              # 위 모듈의 구현
│   └── Qt/               # CoreQt: 재사용 가능한 범용 Qt5 위젯 (별도 타겟)
├── src/                  # 프로젝트 생성 파일 (amr_main에 포함)
├── include/              # 알고리즘 헤더
├── tests/
└── build/                # 빌드 산출물 (bin, lib 포함, git 제외)
```

의존 방향: `CoreQt -> Core`, `amr_main -> Core` (역방향 금지, Core는 Qt를 참조하지 않음)

| 모듈 | 역할 | 의존 |
|---|---|---|
| CMat | 행렬 연산 (SVD, 의사역행렬: LAPACK `dgesvd`, double 정밀도로 계산) | LAPACK |
| 3d | 3차원 공간 (dhVector/dhMat/dhQuat/dhEuler/dhVertex) | - |
| CMesh | 3D 모델 읽기: GLB/glTF, STL, OBJ(+MTL), COLLADA(DAE), 기본 도형 | 3d, cgltf, tinyxml2 |
| CURDF | URDF 읽기, 순기구학, URDF 안의 메시 로딩 | 3d, CMesh, tinyxml2 |
| CDynamics | 동역학 연산 | 3d |
| CSimul | 시뮬레이션 | CDynamics |
| CSock | 외부 통신 | - |

헤더는 `#include "CMat/CMat.h"` 형태로 사용합니다.
`Core/src` 아래에 `.cpp`를 추가하면 CMake가 자동으로 인식합니다.
재사용할 Qt 위젯은 `Core/Qt`(`#include "CoreQt/..."`)에 둡니다.

## 3D 뷰어 (amr_main)

```bash
./run.sh -- Data/Object/samples/arm.urdf        # 예제 로봇 팔 (관절 슬라이더 포함)
./build/bin/amr_main 모델.glb                    # GLB / glTF / STL / OBJ / DAE 도 열림
./build/bin/amr_main robot.urdf --joint joint1=30 --package my_pkg=/path/to/my_pkg
./build/bin/amr_main --info robot.urdf          # 창 없이 파일 정보만 출력
./build/bin/amr_main robot.urdf --shot a.png    # 화면을 PNG 로 저장하고 종료
```

- 인자 없이 실행하면 빈 창이 뜨고, **파일 > 열기 (Ctrl+O)** 나 파일을 창에 끌어다 놓아서 열 수 있습니다.
- 마우스: 왼쪽 드래그 = 회전, 오른쪽/가운데 드래그 = 이동, 휠 = 확대/축소, 더블클릭 또는 `F` = 화면에 맞춤.
- URDF 를 열면 오른쪽에 관절 슬라이더가 생깁니다 (revolute/continuous 는 도, prismatic 은 m, limit 범위 안에서).
- 좌표계는 Z-up 입니다. GLB(Y-up)는 열 때 Z-up 으로 돌려서 보여줍니다. OBJ 에 법선이 없으면 평평한(flat) 음영으로 그립니다.
- OpenGL 2.1 이상이면 동작합니다 (`ModelViewer`, `JointPanel` 은 `Core/Qt`, 재사용 가능).
- Qt 없이 빌드(`-DAMR_BUILD_QT=OFF`)하면 창 대신 파일 정보만 출력합니다. Windows 의 `run.bat` 은 Qt 5 를 찾거나(`CMAKE_PREFIX_PATH`) vcpkg 로 설치한 뒤 창 모드로 빌드합니다 (`--no-qt` 로 끔). vcpkg 의 Qt 는 5.15 대입니다 (5.12 이상이면 동작).
- 마우스 조작과 실제 GPU 환경은 아직 사람 손으로 확인하지 못했습니다 (가상 화면의 소프트웨어 렌더링으로 그림은 확인).

## 한 번에 실행 (run.sh / run.bat)

```bash
./run.sh              # Linux: 없는 패키지 설치 -> 구성/빌드 -> 실행
./run.sh --test       # 실행 대신 테스트
./run.sh --no-qt      # Qt 없이 (qtbase5-dev 설치도 생략)
./run.sh --no-install # 설치하지 않고 부족한 패키지 목록만 안내
./run.sh --clean      # build/ 를 비우고 처음부터
./run.sh -- 인자들     # amr_main 에 넘길 인자
```

```bat
run.bat               :: Windows: CMake/Git(winget), Visual Studio 확인, vcpkg+clapack 설치 -> 빌드 -> 실행
run.bat --test        :: 같은 옵션: --test --no-qt --no-install --clean --debug, -- 뒤는 amr_main 인자
```

- 이미 모두 설치되어 있으면 설치 단계는 건너뛰고 바로 빌드/실행합니다. 설치는 첫 실행에서만 시간이 걸립니다.
- 빌드가 끝나면 `amr_main`(3D 뷰어)을 실행합니다. 인자를 넘기려면 `./run.sh -- 파일.urdf`.
- `run.bat` 은 Visual Studio Build Tools 가 없으면 설치 여부를 물어보고(수 GB), 설치 후에는 새 터미널에서 다시 실행해야 합니다.
  vcpkg 는 `VCPKG_ROOT` 가 있으면 그것을, 없으면 프로젝트 안 `.vcpkg/` 에 받아 사용합니다. Qt 는 기본으로 켜져 있고, 직접 설치한 Qt 5 가 있으면 `CMAKE_PREFIX_PATH`(또는 `QT_DIR`)에 그 폴더를 지정하세요. 없으면 vcpkg 로 설치할지 물어봅니다 (30~60분 이상, 수 GB). 창 없이 콘솔 모드만 쓰려면 `--no-qt`.
- `run.bat` 은 아직 실제 Windows 에서 실행해 보지 못했습니다 (아래 Windows 빌드 절은 MinGW 로만 검증).

## 빌드

```bash
sudo apt install build-essential cmake qtbase5-dev liblapack-dev libtinyxml2-dev   # Ubuntu 20.04 (Qt 5.12.8)

cmake -S . -B build
cmake --build build -j

./build/bin/amr_main
```

Qt 없이 빌드: `cmake -S . -B build -DAMR_BUILD_QT=OFF`

테스트: `cmake -S . -B build -DAMR_BUILD_TESTS=ON -DAMR_BUILD_QT=OFF && cmake --build build --target test_3d test_cmat test_cmesh test_cmesh_formats test_curdf && (cd build && ctest)`

## Windows 빌드

Linux 와 같은 소스·CMake 로 빌드됩니다 (MSVC 2019+ 또는 MinGW-w64, C++11).

```bat
:: 의존성은 vcpkg 로 설치 (Qt 는 5.12 설치본 사용: CMAKE_PREFIX_PATH 에 Qt 폴더 지정)
vcpkg install clapack:x64-windows

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake ^
      -DAMR_BUILD_QT=OFF -DAMR_BUILD_TESTS=ON
cmake --build build --config Release
cd build && ctest -C Release
```

- **LAPACK**: `CMat` 은 LAPACK 의 `dgesvd_` 를 직접 호출합니다 (LAPACKE 불필요). Windows 에서는 vcpkg 의 `clapack`
  (f2c 로 변환한 LAPACK, 정적 라이브러리)의 CMake 설정(`lapack`, `f2c` 타깃)을 직접 사용합니다 (vcpkg 의 `find_package(LAPACK)` 는 `x64-windows` 에서 Fortran 이 필요한 `lapack-reference` 를 가리키기 때문). 참고: vcpkg 의 `openblas` 는 BLAS 만 들어 있고 LAPACK 은 없습니다.
  다른 LAPACK 을 쓰려면 `-DLAPACK_LIBRARIES="<lapack>;<blas>"`.
- **tinyxml2**: 시스템에 없으면 `third_party/tinyxml2`(포함본)를 자동으로 빌드합니다 (`-DAMR_USE_BUNDLED_TINYXML2=ON` 으로 강제 가능).
- 산출물: Visual Studio 같은 멀티 구성 생성기는 `build/bin/<Config>/` 아래에 생기고, `ctest` 에는 `-C <Config>` 가 필요합니다.
- 소스는 UTF-8 이며 MSVC 에는 `/utf-8` 을 자동으로 붙입니다.
- 파일 경로는 ANSI 코드 페이지 기준으로 열립니다. 한글처럼 코드 페이지 밖 문자가 들어간 경로의 모델 파일은 열리지 않을 수 있습니다.
- 검증 범위: MinGW-w64(GCC 13) 로 Core 와 테스트 4종(3d, cmesh, cmesh_formats, curdf)을 Windows 용으로 빌드해 Wine 에서 실행해 통과를 확인했습니다
  (경고 0). **MSVC 빌드, 실제 Windows 에서의 LAPACK(clapack) 링크, Qt 빌드와 `test_cmat` 는 아직 확인하지 못했습니다.**

## 3D 모델 / URDF 읽기 (CMesh, CURDF)

```cpp
#include "CMesh/MeshLoader.h"
#include "CURDF/Urdf.h"

amr::Scene scene;
std::string err;
if (!amr::LoadMesh("robot.glb", &scene, &err)) { /* err 에 원인 */ }   // 확장자로 형식 선택

amr::urdf::Robot robot;
amr::urdf::LoadUrdf("robot.urdf", &robot, &err);

amr::urdf::JointValues q;  q["joint1"] = 0.5f;               // rad (prismatic 은 m)
std::vector<dhMat> T = robot.LinkTransforms(q);               // link 별 월드 변환

amr::urdf::RobotMeshes meshes;
amr::urdf::LoadRobotMeshes(robot, amr::urdf::LoadOptions(), &meshes);  // visual 메시 (scale/축 변환 적용됨)
// 그릴 때: 월드 변환 = T[visual.link] * visual.local * (scene 노드의 WorldTransforms())
```

- 지원 형식: `.glb` `.gltf` `.stl`(ASCII/바이너리) `.obj`(+`.mtl`) `.dae`. Draco/meshopt 압축 GLB, 스킨, 애니메이션은 지원하지 않습니다.
- 좌표계: 로더는 파일 좌표를 그대로 두되, **DAE 는 `unit`(미터)과 `up_axis`(→ Z-up)를 읽는 즉시 정규화**합니다.
  GLB 는 Y-up 이라, URDF 안에서 읽을 때만 X축 +90° 회전으로 Z-up 에 맞춥니다 (`LoadOptions::glbYUpToZUp`, 기본 true).
- URDF: rpy 는 URDF 규약(`Rz(yaw)·Ry(pitch)·Rx(roll)`)입니다. 지원 관절은 revolute / continuous / prismatic / fixed 이고,
  floating / planar 는 읽지만 움직이지 않습니다. mimic 지원, limit 은 `LinkTransforms` 에서 기본적으로 적용됩니다.
- **xacro 는 지원하지 않습니다.** `xacro robot.urdf.xacro > robot.urdf` 로 먼저 변환하세요 (감지하면 오류 메시지를 냅니다).
- `package://pkg/...` 는 `LoadOptions::packagePaths`(패키지 이름 → 폴더) → `searchDirs` → URDF 폴더와 상위 폴더 순서로 찾습니다.
- 일부 메시를 못 읽어도 나머지는 채워지며(`LoadRobotMeshes` 가 false + `warnings`), 구조가 잘못된 URDF(루트 둘 이상, 순환,
  이름 중복, 없는 link 참조 등)는 오류로 거부합니다.

## Qt 라이선스 주의 (LGPLv3)

- Qt는 **동적 링크**로만 사용합니다 (정적 링크 금지).
- GPL 전용 모듈(Qt Charts, Qt Data Visualization 등)은 사용하지 않습니다.
- 배포 시 Qt LGPL 고지문과 라이선스 사본을 함께 포함합니다.

## lMat(CMat) 사용 시 주의

`lMat`은 연산 결과(임시 객체)를 `temp` 플래그로 표시하고, 다른 객체가 값 인자로 받을 때 메모리 소유권을 넘기는 방식으로 복사를 줄입니다. 이 방식은 C++11의 복사 생략(copy elision)과 맞지 않는 경우가 있어, 아래 사용법을 지켜야 합니다.

| 사용 방식 | 결과 |
|---|---|
| `lMat C; C = A * B;` | 안전 (권장) |
| `lMat C = A * B;` 후 `C.Out()`, `C.In()` 등으로 접근 | 안전 |
| `lMat C = A * B;` 후 한 번도 접근하지 않고 소멸 | 메모리 누수 |
| `lMat T = A * B;` 후 `R = E * T;` 처럼 T를 오른쪽 피연산자(값 인자)로 처음 사용 | `T`가 비워짐 (`R`은 정상) |
| `lMat C = A * B; lMat D = C * C;` | 널 포인터 접근으로 크래시 |

- 선언과 동시에 초기화(`lMat C = A * B;`)한 변수는, 처음 접근하기 전까지 `temp` 상태입니다. 처음 쓰는 곳이 오른쪽 피연산자나 인자이면 내용을 빼앗깁니다.
- 컴파일 옵션 `-fno-elide-constructors`에서는 위 문제가 모두 사라지므로, 원인은 복사 생략입니다.
- `A.SVD(U, S, Vt)`와 `Inv()`는 내부에서 `double` 로 올려 LAPACK `dgesvd` 로 계산하고 마지막에 `float` 로 한 번만 반올림합니다 (조건수 1e5 행렬의 역행렬 상대오차: float `sgesvd` 약 1e-4 → 약 4e-8). `A` 는 바뀌지 않습니다 (예전에는 덮어썼음).
- 정밀도의 한계는 저장 형식(`float`)입니다. 입력 행렬이 `float` 이므로 특이값이 `max(r,c)·1.2e-7·최대특이값` 이하이면 잡음으로 보고 0 으로 취급합니다.
- `A.SVD()`가 돌려주는 `S`는 `r x c`가 아니라 `p x p` (`p = min(r, c)`) 대각행렬입니다.

## 서드파티 라이선스

| 라이브러리 | 용도 | 라이선스 |
|---|---|---|
| Qt 5.12 | CoreQt | LGPLv3 (위 주의사항 참고) |
| LAPACK (Linux: liblapack, Windows: clapack) | CMat (SVD, 역행렬) | modified BSD |
| cgltf 1.15 | GLB/glTF 읽기 (`third_party/cgltf`, 수정 없이 포함) | MIT |
| tinyxml2 | URDF, COLLADA 의 XML 읽기 (Linux: apt 동적 링크, 없으면 `third_party/tinyxml2` 포함본 10.0.0) | zlib |
