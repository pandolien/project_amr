# Project Simul

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
project_simul/
├── CMakeLists.txt        # 전체 빌드
├── main.cpp              # 메인 시스템
├── Data/Object/          # 오브젝트 관련 자료
├── Core/                 # 라이브러리화되는 핵심 코드 (순수 C++11, 표준 라이브러리·Qt 금지)
│   ├── CMakeLists.txt
│   ├── include/          # CMat, 3d, CMesh, CURDF, CCollision, CDynamics, CSimul, CProcess, CSock
│   ├── src/              # 위 모듈의 구현
│   └── Qt/               # CoreQt: 재사용 가능한 범용 Qt5 위젯 (별도 타겟)
├── src/                  # 프로젝트 생성 파일 (simul_main에 포함)
├── include/              # 알고리즘 헤더
├── tests/
└── build/                # 빌드 산출물 (bin, lib 포함, git 제외)
```

의존 방향: `CoreQt -> Core`, `simul_main -> Core` (역방향 금지, Core는 Qt를 참조하지 않음)

| 모듈 | 역할 | 의존 |
|---|---|---|
| CBase | C++ 표준 라이브러리 대체: `Array`(vector), `String`, `Map`/`Set`(정렬 배열), `HashMap`, `Pair`, `UniquePtr`, `Sort`, 수학 `Abs`/`Sqrt`/`Sin`…, 숫자 파싱·`Tokens`·`Lines`, 파일 읽기/쓰기 | C 런타임 |
| CMat | 행렬 연산 (SVD, 의사역행렬: LAPACK `dgesvd`, double 정밀도로 계산) | LAPACK |
| 3d | 3차원 공간 (dhVector/dhMat/dhQuat/dhEuler/dhVertex), 자세 `dhPose`(위치 m + 쿼터니언), 단위 변환 `Units.h` | - |
| CMesh | 3D 모델 읽기: GLB/glTF, STL, OBJ(+MTL), COLLADA(DAE), 기본 도형 | 3d, cgltf, tinyxml2 |
| CURDF | URDF 읽기, 순기구학, URDF 안의 메시 로딩 | 3d, CMesh, tinyxml2 |
| CCollision | 충돌 감지: AABB, OBB(SAT), 겹침 쌍 찾기(Sweep and Prune), 로봇 링크 자기충돌(AABB -> OBB 2단계) | 3d, CMesh, CURDF |
| CSensor | 센서 시뮬레이션 (2D/3D 라이다: 레이캐스트 부채꼴, 잡음, 범위 / 카메라: BGR 컬러 + 깊이 영상) | CCollision, 3d |
| CKinematics | 범용 수치 역기구학 (감쇠 최소제곱, 관절 한계, mimic) | 3d, CURDF |
| CDynamics | 동역학: 중력 토크, 질량중심 (double 계산, SI 단위) | 3d, CURDF |
| CSimul | 시뮬레이션 | CDynamics |
| CProcess | 물류 공정: 물건을 A 지점에서 B 지점으로 옮긴다 (로봇 없이 직선 이동, 상태/이벤트) | 3d |
| CSock | 외부 통신 | - |

**표준 라이브러리를 쓰지 않습니다.** 프로젝트 코드(Core, Core/Qt, main.cpp, tests)에는 `std::` 와 C++ 표준 헤더(`<vector>`, `<string>`, `<cmath>` …)가 없고, 대신 `Core/include/CBase` 를 씁니다. C 런타임 함수(`printf`, `fopen`, `sqrt`, `memcpy`, `<stdint.h>`, `<math.h>` 등), Qt, tinyxml2, cgltf, LAPACK 은 그대로 씁니다. 예외도 쓰지 않습니다. 이 규칙은 CTest `no_std`(`cmake/CheckNoStd.cmake`)가 검사합니다.
CBase 사용상 주의:
- `Map`/`Set` 은 정렬된 배열이라 넣거나 지우면 이전에 얻은 반복자가 무효가 되고 넣기는 O(n) 입니다. 항목이 많으면(예: 메시 정점 합치기) `HashMap`(키에 `==` 와 `size_t Hash() const` 필요)을 씁니다.
- `Array::clear()` 는 원소를 `T()` 로 되돌립니다. 비운 뒤 이전 원소에 대한 참조를 쓰면 안 됩니다.
- 숫자 파싱(`ParseDouble` 등)은 로케일과 무관하게 항상 `.` 을 소수점으로 읽습니다.
- Qt 와의 변환은 `CoreQt/QtString.h` 의 `simulqt::ToQ` / `FromQ` 를 씁니다 (UTF-8).

헤더는 `#include "CMat/CMat.h"` 형태로 사용합니다.
`Core/src` 아래에 `.cpp`를 추가하면 CMake가 자동으로 인식합니다.
재사용할 Qt 위젯은 `Core/Qt`(`#include "CoreQt/..."`)에 둡니다.

## 단위와 자세 규약

| 구간 | 규약 |
|---|---|
| 내부 계산, 데이터 전송 | 길이 **m**, 각도 **rad**, 방향은 **쿼터니언 (x, y, z, w)**, 관절 값은 rad / m |
| 사람에게 보이는 곳 (화면, 콘솔, 명령행) | 각도는 **deg** (이동 관절은 m 그대로) |

- 자세는 `dhPose`(`3d/dhPose.h`)로 주고받습니다. 전송 형식은 7개 값 `x, y, z, qx, qy, qz, qw` 이고, `ToArray` 는 쿼터니언을 정규화하고 `w >= 0` 으로 맞춥니다(`q` 와 `-q` 는 같은 회전). `FromArray` 는 NaN/무한대와 길이 0 쿼터니언을 거부하고 정규화합니다.
- degree 변환은 사람과 맞닿는 경계에서만 합니다: `simul::units::RadToDeg/DegToRad`, 화면용 방향 `RpyDegFromQuat`(roll/pitch/yaw, URDF rpy 규약), 관절은 `Joint::ToDisplay/FromDisplay`(각도 관절만 변환), `DisplayUnit()`, `TorqueUnit()`.
- 주의: `dhQuat` 의 생성자 순서는 `(x, y, z, w)` 이고 멤버 선언 순서는 `w, x, y, z` 입니다. `dhQuat(1, 0, 0, 0)` 은 항등이 아니라 X축 180도 회전입니다.
- 중력/동역학은 double 로 계산합니다 (입력인 `dhMat`, `Robot` 값은 float 이므로 입력 정밀도는 float 입니다).

## 3D 뷰어 (simul_main)

```bash
./run.sh -- Data/Object/samples/arm.urdf        # 예제 로봇 팔 (관절 슬라이더 포함)
./build/bin/simul_main 모델.glb                    # GLB / glTF / STL / OBJ / DAE 도 열림
./build/bin/simul_main robot.urdf --joint joint1=30 --package my_pkg=/path/to/my_pkg
./build/bin/simul_main --info robot.urdf          # 창 없이 파일 정보만 출력
./build/bin/simul_main robot.urdf --shot a.png    # 화면을 PNG 로 저장하고 종료
```

- 인자 없이 실행하면 빈 창이 뜨고, **파일 > 열기 (Ctrl+O)** 나 파일을 창에 끌어다 놓아서 열 수 있습니다.
- 마우스: 왼쪽 드래그 = 회전, 오른쪽/가운데 드래그 = 이동, 휠 = 확대/축소, 더블클릭 또는 `F` = 화면에 맞춤, 왼쪽 클릭 = 물체 선택.
- URDF 를 열면 오른쪽에 관절 슬라이더가 생깁니다 (revolute/continuous 는 도, prismatic 은 m, limit 범위 안에서).
- 좌표계는 Z-up 입니다. GLB(Y-up)는 열 때 Z-up 으로 돌려서 보여줍니다. OBJ 에 법선이 없으면 평평한(flat) 음영으로 그립니다.
- OpenGL 2.1 이상이면 동작합니다 (`ModelViewer`, `JointPanel` 은 `Core/Qt`, 재사용 가능).
- Qt 없이 빌드(`-DSIMUL_BUILD_QT=OFF`)하면 창 대신 파일 정보만 출력합니다. Windows 의 `run.bat` 은 Qt 5 를 찾거나(`CMAKE_PREFIX_PATH`) vcpkg 로 설치한 뒤 창 모드로 빌드합니다 (`--no-qt` 로 끔). vcpkg 의 Qt 는 5.15 대입니다 (5.12 이상이면 동작).
- **물체 선택과 좌표계**: 화면에서 물체를 **왼쪽 클릭**하면 그 링크(정적 모델이면 모델 전체)가 선택되어 밝게 강조되고, 구조 트리의 같은 링크도 함께 선택됩니다. 선택한 것에는 **좌표계**(가운데 구 + X 빨강 / Y 초록 / Z 파랑 화살표)가 붙고, 아래 상태바에 위치(m), RPY(degree), 쿼터니언이 나옵니다(루트 링크 기준). 빈 곳을 클릭하면 선택이 풀리고, 관절을 움직이면 좌표계도 따라갑니다. `보기 > 선택 좌표계` / `G` 로 켜고 끕니다. 클릭 판정은 삼각형 단위(`CCollision/MeshRaycast.h`)라서 상자가 아니라 실제 모양에 맞습니다. 좌표계 모양은 `Data/Object/gizmo/axes.obj`(+`axes.mtl`)이며, 직접 고치거나 `python3 tools/make_gizmo_obj.py` 로 다시 만들 수 있습니다(화살표 끝까지 길이 1, 화면에서 항상 같은 크기로 보이게 뷰어가 크기를 곱합니다). 확인용으로 `--click x,y` 는 3D 화면의 그 픽셀을 클릭한 것처럼 처리합니다.
- **오브젝트 이동**: 정적 모델이나 로봇의 루트 링크(베이스)를 선택한 상태에서 좌표계 **화살표를 끌면** 그 축 방향으로만 움직입니다(화살표 위에서 누르면 카메라가 아니라 물체가 움직입니다). 로봇은 전체가 AMR 위치처럼 같이 움직이고, 충돌 박스와 좌표계도 따라옵니다. 위치는 월드 기준 m 이며 상태바에 나옵니다. 팔 중간 링크는 관절에 묶여 있어서 이동되지 않고 관절 슬라이더로만 움직입니다(끌어서 움직이려면 IK 가 필요합니다). 관절 값, 충돌 검사, 중력 계산은 로봇 자신의 좌표계(루트 기준)에서 하므로 이동의 영향을 받지 않습니다. `보기 > 오브젝트 위치 초기화`로 되돌리고, `--move x,y,z` 로 시작 위치를 정하며, `--drag x0,y0,x1,y1` 은 확인용으로 마우스 드래그를 흉내 냅니다. **회전**: 모델의 루트 링크를 선택하면 좌표계 화살표 둘레에 **회전 링 3개**(X 빨강 / Y 초록 / Z 파랑, 끄는 동안 노랑)가 나옵니다. 화살표와 링은 기본으로 **월드 X/Y/Z 에 고정**되어 있고(`보기 > 회전 링 기준`에서 **오브젝트 축**으로 바꾸면 모델과 같이 돕니다), 링에 마우스를 올리면 밝아집니다. 끌면 그 축 둘레로 모델 원점(루트 링크의 원점)을 중심으로 돕니다. 링 위에서 잡은 점의 접선 방향으로 마우스를 움직인 만큼 돌아가고(쿼터니언 회전), 링을 비스듬히 볼 때도 값이 튀지 않습니다. 모델이 하나일 때도 되고(처음 돌리면 구조 트리에 `world` 가 생기고 자세가 `<모델>_attach` 관절에 들어갑니다), 우클릭 `위치 지정...` 에서 x, y, z, roll, pitch, yaw(도, R = Rz·Ry·Rx)를 직접 넣을 수도 있습니다.
- **중력 낙하**(모델을 둘 이상 올렸거나 한 번이라도 돌린 장면은 **모델 하나하나**가 따로 떨어지고, 자세와 관절 값이 반영된 가장 낮은 점이 바닥에 닿습니다. 끄는 동안에는 그 모델만 붙잡혀 있습니다): 이동으로 들어 올린 물체(정적 모델, 로봇 전체)는 놓으면 **바닥(z = 0 인 격자 평면)까지 떨어집니다**. 바닥에 닿으면 조금 튕기다 멈추고(`CDynamics/FreeFall.h` 의 `FallBody`, 등가속도 식을 그대로 써서 낙하 시간이 sqrt(2h/g) 와 맞습니다), 가장 낮은 점은 보이는 모양의 꼭짓점으로 잡습니다. 화살표를 끄는 동안에는 떨어지지 않고, 놓는 순간부터 떨어집니다. 중력은 `보기 > 중력 방향...` 의 수직(z) 성분만 쓰고(위로 향하거나 0 이면 떨어지지 않음), 회전과 수평 이동은 아직 없으며, 이미 바닥 아래에 있는 물체는 건드리지 않습니다. `보기 > 중력 낙하` 로 끄고 켜며(`--no-fall` 은 처음부터 끔), `--settle` 은 확인용으로 낙하가 끝날 때까지 진행한 뒤 위치를 출력합니다.
- **모터 끄기 (관절의 중력 동작)**: `보기 > 모터 끄기` / `M` 을 켜면 지금 자세에서 모든 관절이 **중력, 링크 관성, 마찰에 따라 실제로 움직입니다**(수평으로 뻗은 팔이 어깨부터 처지고, 팔꿈치가 흔들리다 가라앉습니다). 관절 limit 에 닿으면 멈추고, 슬라이더를 손으로 옮기면 그 관절만 속도 0 으로 다시 시작합니다. 끄면(모터 켜짐) 지금 자세를 유지합니다. URDF `<dynamics damping friction>` 을 쓰고, 감쇠가 없는 관절에는 영원히 흔들리지 않도록 약한 점성 감쇠(0.2)를 줍니다. 팔이 처지다가 베이스 같은 다른 링크를 통과하는데, 충돌은 빨갛게 표시만 하고 접촉 반응은 아직 없습니다. `--motors-off` 는 처음부터 끈 상태로, `--simulate 초` 는 확인용으로 그 시간만큼 진행한 뒤 관절 값을 출력합니다.
- **구조 트리** (왼쪽, `보기 > 구조 트리` / `T`): URDF 는 루트 링크부터 관절을 따라 이어지는 링크 계층(관절 이름, 종류와 limit, visual/collision 개수, 질량)을, 정적 모델은 노드 계층을 보여줍니다. 링크를 클릭하면 뷰어에서 그 링크가 밝게 강조되고, 충돌 중인 링크는 트리에서 빨갛게 표시됩니다.
- **충돌 박스** (`보기 > OBB 충돌 박스` `O`, `보기 > AABB 충돌 박스` `B`, 또는 `--obb` / `--aabb`): AABB(링크마다, 노랑 / 후보이면 주황)와 OBB(도형마다, 하늘색 / 충돌하면 빨강)를 그립니다. 상태바 오른쪽에 OBB 로 확인된 충돌 쌍과 AABB 후보 수가 나옵니다 (부모-자식으로 직접 이어진 링크는 항상 맞닿으므로 제외). 창 없이 `--info --obb --joint 이름=값` 으로 링크별 AABB 와 충돌 목록을 출력할 수도 있습니다.
- **중력 하중**: 구조 트리의 "중력 하중" 열에 **모든 관절**이 각자 따로 나옵니다(구동 관절은 모터 토크, mimic 은 자기 몫과 합산되는 원본, fixed 는 체결부 하중). 관절 패널의 슬라이더 아래에도 구동 관절의 토크(회전 N·m, 이동 N)가 나오고, URDF `effort` 한계를 넘으면 빨갛게 표시됩니다. `보기 > 중력 방향...` 이나 `--gravity gx,gy,gz` 로 중력 방향을 바꿀 수 있습니다(루트 링크 좌표계, 경사면의 AMR 이면 기울어진 방향). 창 없이 `--info --torque [--joint 이름=도] [--gravity ...]` 로 모든 관절의 하중과 링크별 기여 내역을 출력할 수도 있습니다. `Data/Object/samples/arm_gripper.urdf` 는 fixed 관절과 mimic 관절이 있는 그리퍼 예제입니다.
- `--select 링크이름` 으로 시작할 때 링크를 선택하고, `--shot-window 파일.png` 로 패널을 포함한 창 전체를 저장합니다.
- 마우스 조작과 실제 GPU 환경은 아직 사람 손으로 확인하지 못했습니다 (가상 화면의 소프트웨어 렌더링으로 그림은 확인).

## 한 번에 실행 (run.sh / run.bat)

```bash
./run.sh              # Linux: 없는 패키지 설치 -> 구성/빌드 -> 실행
./run.sh --test       # 실행 대신 테스트
./run.sh --no-qt      # Qt 없이 (qtbase5-dev 설치도 생략)
./run.sh --no-install # 설치하지 않고 부족한 패키지 목록만 안내
./run.sh --clean      # build/ 를 비우고 처음부터
./run.sh -- 인자들     # simul_main 에 넘길 인자
```

```bat
run.bat               :: Windows: CMake/Git(winget), Visual Studio 확인, vcpkg+clapack 설치 -> 빌드 -> 실행
run.bat --test        :: 같은 옵션: --test --no-qt --no-install --clean --debug, -- 뒤는 simul_main 인자
```

- 이미 모두 설치되어 있으면 설치 단계는 건너뛰고 바로 빌드/실행합니다. 설치는 첫 실행에서만 시간이 걸립니다.
- 빌드가 끝나면 `simul_main`(3D 뷰어)을 실행합니다. 인자를 넘기려면 `./run.sh -- 파일.urdf`.
- `run.bat` 은 Visual Studio Build Tools 가 없으면 설치 여부를 물어보고(수 GB), 설치 후에는 새 터미널에서 다시 실행해야 합니다.
  vcpkg 는 `VCPKG_ROOT` 가 있으면 그것을, 없으면 프로젝트 안 `.vcpkg/` 에 받아 사용합니다. Qt 는 기본으로 켜져 있고, 직접 설치한 Qt 5 가 있으면 `CMAKE_PREFIX_PATH`(또는 `QT_DIR`)에 그 폴더를 지정하세요. 없으면 vcpkg 로 설치할지 물어봅니다 (Release 전용 + OpenSSL 제외로 20~40분, 수 GB). 창 없이 콘솔 모드만 쓰려면 `--no-qt`.
- `run.bat` 은 아직 실제 Windows 에서 실행해 보지 못했습니다 (아래 Windows 빌드 절은 MinGW 로만 검증).

## 빌드

```bash
sudo apt install build-essential cmake qtbase5-dev liblapack-dev libtinyxml2-dev   # Ubuntu 20.04 (Qt 5.12.8)

cmake -S . -B build
cmake --build build -j

./build/bin/simul_main
```

Qt 없이 빌드: `cmake -S . -B build -DSIMUL_BUILD_QT=OFF`

테스트: `cmake -S . -B build -DSIMUL_BUILD_TESTS=ON -DSIMUL_BUILD_QT=OFF && cmake --build build --target test_3d test_cmat test_cmesh test_cmesh_formats test_curdf && (cd build && ctest)`

## Windows 빌드

Linux 와 같은 소스·CMake 로 빌드됩니다 (MSVC 2019+ 또는 MinGW-w64, C++11).

```bat
:: 의존성은 vcpkg 로 설치 (Qt 는 5.12 설치본 사용: CMAKE_PREFIX_PATH 에 Qt 폴더 지정)
vcpkg install clapack:x64-windows

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake ^
      -DSIMUL_BUILD_QT=OFF -DSIMUL_BUILD_TESTS=ON
cmake --build build --config Release
cd build && ctest -C Release
```

- **LAPACK**: `CMat` 은 LAPACK 의 `dgesvd_` 를 직접 호출합니다 (LAPACKE 불필요). Windows 에서는 vcpkg 의 `clapack`
  (f2c 로 변환한 LAPACK, 정적 라이브러리)의 CMake 설정(`lapack`, `f2c` 타깃)을 직접 사용합니다 (vcpkg 의 `find_package(LAPACK)` 는 `x64-windows` 에서 Fortran 이 필요한 `lapack-reference` 를 가리키기 때문). 참고: vcpkg 의 `openblas` 는 BLAS 만 들어 있고 LAPACK 은 없습니다.
  다른 LAPACK 을 쓰려면 `-DLAPACK_LIBRARIES="<lapack>;<blas>"`.
- **tinyxml2**: 시스템에 없으면 `third_party/tinyxml2`(포함본)를 자동으로 빌드합니다 (`-DSIMUL_USE_BUNDLED_TINYXML2=ON` 으로 강제 가능).
- 산출물: Visual Studio 같은 멀티 구성 생성기는 `build/bin/<Config>/` 아래에 생기고, `ctest` 에는 `-C <Config>` 가 필요합니다.
- 소스는 UTF-8 이며 MSVC 에는 `/utf-8` 을 자동으로 붙입니다.
- 파일 경로는 ANSI 코드 페이지 기준으로 열립니다. 한글처럼 코드 페이지 밖 문자가 들어간 경로의 모델 파일은 열리지 않을 수 있습니다.
- 검증 범위: MinGW-w64(GCC 13) 로 Core 와 테스트 4종(3d, cmesh, cmesh_formats, curdf)을 Windows 용으로 빌드해 Wine 에서 실행해 통과를 확인했습니다
  (경고 0). **MSVC 빌드, 실제 Windows 에서의 LAPACK(clapack) 링크, Qt 빌드와 `test_cmat` 는 아직 확인하지 못했습니다.**

## 3D 모델 / URDF 읽기 (CMesh, CURDF)

```cpp
#include "CMesh/MeshLoader.h"
#include "CURDF/Urdf.h"

simul::Scene scene;
std::string err;
if (!simul::LoadMesh("robot.glb", &scene, &err)) { /* err 에 원인 */ }   // 확장자로 형식 선택

simul::urdf::Robot robot;
simul::urdf::LoadUrdf("robot.urdf", &robot, &err);

simul::urdf::JointValues q;  q["joint1"] = 0.5f;               // rad (prismatic 은 m)
std::vector<dhMat> T = robot.LinkTransforms(q);               // link 별 월드 변환

simul::urdf::RobotMeshes meshes;
simul::urdf::LoadRobotMeshes(robot, simul::urdf::LoadOptions(), &meshes);  // visual 메시 (scale/축 변환 적용됨)
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

## 동역학 (CDynamics: Dynamics.h)

운동방정식 `M(q) q'' + h(q, q') = tau - 마찰` 을 구동 관절(mimic 은 마스터에 합쳐짐) 좌표로 풉니다. 모두 double, SI 단위, 루트 링크 좌표계입니다.

| 함수 | 설명 |
|---|---|
| `InverseDynamics(robot, q, qd, qdd, g)` | 재귀 뉴턴-오일러(RNEA). 그 가속도를 내려면 각 관절이 내야 하는 토크. qd = qdd = 0 이면 `GravityTorques` 와 같다 |
| `ComputeMassMatrix(robot, q)` | 질량 행렬 M(q) (대칭, 양의 정부호). 링크의 `<inertial>` 질량, 질량중심, 관성 텐서를 쓴다 |
| `ForwardDynamics(robot, q, qd, tau, g, opt)` | 모터 토크를 줬을 때의 관절 가속도 (감쇠와 쿨롱 마찰 포함) |
| `Simulator` | RK4(1/500 초 이하)로 (q, qd) 적분. limit 에서 멈춤, 모터 토크 기본 0 |

검증: 정지 상태 RNEA = 중력 토크, M 이 대칭이고 양의 정부호, `RNEA(qdd) = M qdd`, 코리올리/원심력/자이로 항을 **질량 행렬의 수치 미분**으로 독립 계산해서 대조, 감쇠 없는 에너지 보존(회전/이동/mimic/회전된 관성 텐서), 진자 주기 `2π sqrt(M / (m g L))`, 점성 감쇠 `exp(-d t / M)`, 쿨롱 마찰 감속 `f / M`, limit 정지. 변이 테스트 13종을 모두 잡았습니다.

## 역기구학 (CKinematics: IK.h)

URDF 만 있으면 어떤 로봇이든 쓰는 범용 수치 IK 입니다. `SolveIk(robot, 시작 관절값, 링크, 목표 자세, 옵션)`. 자코비안을 관절 축/원점에서 만들고 감쇠 최소제곱(Levenberg-Marquardt)으로 풉니다.
- 위치만(기본) 또는 위치 + 자세(`useOrientation`), mimic 관절은 마스터에 합쳐서 풉니다.
- 관절 한계를 지킵니다 (한계에 닿은 관절은 고정하고 나머지로 다시 풉니다). 닿을 수 없으면 `converged = false` 와 함께 가장 가까운 값을 돌려줍니다.
- 시작 값에 가까운 해로 수렴하고, 중복 자유도(7축 등)는 가장 적게 움직이는 해를 고릅니다.
- 일직선 같은 특이 자세 근처에서는 국소 최솟값에 멈출 수 있습니다. 한 번에 해를 찾아야 하면 `restarts` (한계 안의 무작위 값으로 다시 시작, 시드 고정)를 씁니다.
- 로봇 전용 해석적 IK 가 필요하면 같은 시그니처로 따로 추가하고 호출부만 바꾸면 됩니다.
- 뷰어: 팔 링크를 선택하고 좌표계 화살표를 끌면 위치만 맞추는 IK 로 관절이 따라옵니다.

## 센서 (CSensor: Lidar2D.h)

**2D/3D 라이다** (`Lidar`): 센서에서 광선을 쏘고, 광선마다 메시와 가장 가까운 교차점까지의 거리를 잽니다. 광선-삼각형 교차는 CCollision 의 `RaycastTriangles` (Möller-Trumbore, 외적 2번)를 그대로 씁니다.
- 광선 방향은 방위각 θ (`angleMin ~ angleMax`, `angleIncrement` 간격)와 고각 φ (`elevationMin ~ elevationMax` 사이에 `channels` 개)로 정한다: `d = (cosφ cosθ, cosφ sinθ, sinφ)`. 2D 라이다는 `channels = 1`, φ = 0 (센서의 수평면), 3D 라이다는 `channels > 1`. 광선 번호는 `채널 * azimuthCount + 방위 번호`.
- 각도는 센서 +x 에서 시작해 +z 둘레로 반시계 방향 (ROS `LaserScan` 규약). 못 맞은 광선의 거리는 무한대, `IsHit(i)` 로 확인합니다.
- `rangeMin` 보다 가까운 면(사각지대), `rangeMax` 보다 먼 면은 안 잡힙니다. 가장 가까운 면이 사각지대면 그 뒤의 물체가 대신 보이지 않고 그 광선은 무효입니다.
- 가우시안 잡음(`noiseStdDev`, 시드 고정이라 재현됩니다).
- `LidarTarget` 하나가 메시 하나입니다 (정점 배열을 복사하지 않고 가리킵니다, `world` 로 자세를 줍니다). 센서가 붙은 로봇 자신은 대상에서 빼야 자기 몸이 안 잡힙니다.
- 속도: `LidarTarget::bvh` 에 메시의 BVH(아래)를 넣으면 광선마다 BVH 로 검사합니다 (삼각형 20만 개의 벽, 3D 16 채널 8656 개 광선이 스캔 한 번에 약 1.3 ms. BVH 없이는 약 4.4 초). BVH 가 없으면 2D(수평 한 줄)는 센서 평면을 가로지르는 삼각형만 검사하고(약 3 ms), 경계 상자로도 거릅니다. 3D 에서 BVH 없이 큰 메시를 스캔하면 느립니다.
- 뷰어: `--lidar 링크[,x,y,z]` 로 그 링크에 달면 (`--lidar-channels 16,-15,15` 로 3D) 맞은 점(빨강 십자)과 광선(옅은 청록)이 매 프레임 갱신됩니다. 로봇이 열려 있으면 센서가 붙은 링크를 뺀 다른 링크가 보입니다.
- 설정은 URDF 에 적습니다 (Gazebo 형식). `<gazebo reference="링크"><sensor type="ray">` 안의 `<pose>`(링크 기준 자세), `<ray><scan><horizontal|vertical><samples/min_angle/max_angle>`(방위각 광선 수와 범위, 수직 채널 수와 고각), `<range><min/max>`, `<noise><stddev>` 를 읽습니다 (`type` 은 ray, gpu_ray, lidar, gpu_lidar. 카메라 등 다른 센서는 건너뜀). 수직 `samples` 가 1 이면 2D, 늘리면 3D 입니다. 파일을 열면 설정을 읽어 두고(여러 개면 첫 번째), 왼쪽 구조 트리에서 라이다가 달린 링크를 마우스 오른쪽 버튼으로 누르면 "레이저 표시 (3D 화면)" 와 "그래프 표시" 가 따로 켜고 끌 수 있는 메뉴로 나옵니다. 레이저는 3D 화면의 광선과 맞은 점, 그래프는 오른쪽 도크이며 둘 중 하나만 켜도 됩니다. 값이 잘못되면(샘플 0, 범위 min>=max, 없는 링크 등) 읽기가 실패합니다. 명령행 `--lidar 링크`, `--lidar-channels` 는 URDF 에 없을 때나 덮어쓸 때 씁니다.
- 그래프: 스캔 표시를 켜면(위) 오른쪽에 "라이다" 도크가 생기고(`CoreQt/LidarPlot.h`, QPainter 만 사용), 위에서 본 지도(맞은 점, 멀수록 진하게, 센서 방향 표시)와 방위각-거리 그래프가 스캔할 때마다 갱신됩니다. 슬라이더로 기둥을 움직이면 그래프도 같이 바뀝니다.
- 3D 예제: `Data/Object/samples/lidar_room_3d.urdf` 는 같은 방에 수직 채널 16 개(고각 -20~+15 도), 방위각 360 개(광선 5760 개)짜리 3D 라이다를 단 것입니다. 아래를 보는 채널이 낮은 상자와 바닥을 잡고, 그래프 도크는 고각 0 에 가장 가까운 채널을 그립니다. `tests/test_csensor.cpp` 의 `testLidarRoom3dExample` 이 검증합니다.
- 예제: `Data/Object/samples/lidar_room.urdf` 는 8×6 m 방(벽, 상자, 원기둥, 라이다 높이 아래의 낮은 상자)과 그 안을 x, y 로 미끄러지고 회전하는 라이다 기둥입니다. `simul_main Data/Object/samples/lidar_room.urdf --joint x_slide=-0.5 --joint yaw=-20 --lidar-csv scan.csv` (라이다 설정은 이 URDF 의 `<gazebo>` 에 있습니다) 로 스캔을 CSV 로 저장하고, `python3 tools/plot_lidar_scan.py scan.csv out.png --room-example --sensor=-0.5,0,-20` 으로 위에서 본 지도와 방위각-거리 그래프를 그립니다 (matplotlib 필요). 정답 거리는 `tests/test_csensor.cpp` 의 `testLidarRoomExample` 이 검증합니다.
- 아직 없는 것: 정적 모델(장애물)과 로봇을 한 화면에 같이 여는 것, 스캔 도중 센서가 움직이는 효과, 반사 강도.

## 카메라 (CSensor: Camera.h)

핀홀 카메라. 픽셀마다 광선을 하나씩 쏴서(CPU, 메시 BVH 사용) 가장 가까운 면의 색과 깊이를 잰다.

- **좌표계**: 라이다와 같은 센서 좌표계. +x 가 보는 방향, +y 왼쪽, +z 위. 영상의 u 는 오른쪽, v 는 아래쪽으로 커지고 원점은 왼쪽 위다 (OpenCV 영상 규약). OpenCV/ROS 광학 좌표계(z 가 앞)와는 축이 다르다.
- **컬러 영상은 BGR 8비트**: 픽셀마다 B, G, R, 행 우선, 행 사이 빈틈 없음(`stride = width*3`). `CameraImage::bgr` 를 `cv::Mat(height, width, CV_8UC3, image.bgr.data())` 로 복사 없이 감쌀 수 있다.
- **깊이 영상은 float(CV_32FC1 배치), 단위 m**: 센서 +x 방향의 거리(z-depth)이고 광선 길이가 아니다. 안 보이거나 near/far 밖이면 0. `CameraImage::PixelPoint(u, v, &p)` 가 센서 좌표의 점을 준다. 내부 변수 `intrinsics`(fx, fy, cx, cy)는 OpenCV 카메라 행렬과 같은 뜻이다.
- **설정**: `CameraSettings` 가로/세로 픽셀, 가로 시야각(rad, 세로는 비율로 결정), near/far, 깊이 잡음 표준편차(시드 고정), 배경색.
- **색**: 메시의 기본 색에 카메라 방향 조명(밝기 = 0.3 + 0.7×|법선·광선|)을 곱한다. 텍스처, 정점 색, 그림자, 반사는 아직 없다 (1단계). 나중에 GPU 렌더로 바꿀 수 있게 `Camera::Render` 한 곳만 바꾸면 되도록 두었다.
- **뷰어**: `ModelViewer::SetSensorCamera(link, settings, mount)` 로 링크에 달고(그 링크 자신은 안 찍는다), `RenderSensorCamera()` 를 부를 때만 찍는다 (매 프레임 찍지 않는다. 호스트가 원하는 주기로 부르고, `sensorCameraRendered` 시그널로도 받는다). 3D 화면에는 시야 절두체(노랑, 위쪽 변에 작은 삼각형)가 나온다. 3D 화면을 보는 시점(`CameraYaw` 등)과 이름이 비슷하니 헷갈리지 말 것.
- **영상 창**: `CoreQt/CameraView` (컬러/깊이 보기, 마우스를 올리면 픽셀의 깊이와 센서 좌표 표시). Qt 5.12 에는 `QImage::Format_BGR888` 이 없어 표시할 때만 채널을 바꿔 복사하고, 데이터는 BGR 그대로다.
- **앱**: `simul_main 모델.urdf --camera 링크,x,y,z --camera-size 640x480 --camera-fov 70 --camera-range 0.05,10 --camera-save out/c` (영상 창이 같이 뜬다. `--camera-save` 는 `c.png` 컬러, `c_depth.png` 회색 깊이, `c_depth16.png` 16비트 mm 를 저장한다). 메뉴 센서 > 카메라 영상 으로 창을 켜고 끈다.
- **검증**: `test_ccamera` (깊이 해석해, BGR 순서, 영상 방향, 가림, near/far, 잡음 재현성, BVH 일치, 내부 변수, 예외), `test_simulview` (링크에 달아 찍기, 시그널).

## 충돌 감지 (CCollision)

2단계 구조입니다. (1) **AABB**(축 정렬 경계 상자)로 빠르게 후보를 거르고, (2) 후보만 **OBB**(방향 경계 상자, 물체와 같이 회전)로 다시 검사합니다. AABB 는 축에 고정이라 기울어진 물체에서는 실제보다 훨씬 커지므로 "겹침 후보"일 뿐이고, OBB 는 물체에 딱 맞아서 훨씬 정확합니다 (거짓 음성은 없도록 설계, 거짓 양성은 OBB 가 크게 줄임). 더 정밀한 판정이 필요하면 후보를 GJK 등으로 넘기는 구조를 염두에 두었습니다.

| 헤더 | 내용 |
|---|---|
| `CCollision/Aabb.h` | `Aabb`: Expand / Merge / Overlaps / Contains / Distance / PenetrationDepth / Inflated / Transformed / Raycast |
| `CCollision/Obb.h` | `Obb`: FromAabb / ToAabb / Corners / Contains / Inflated / Transformed, `Overlaps` (분리축 정리 SAT, 15축) |
| `CCollision/MeshRaycast.h` | `RaycastTriangles`: 광선과 삼각형 메시의 교차(화면 클릭 선택용, 양면, 위치+법선 섞인 배열 지원) |
| `CCollision/Broadphase.h` | `FindOverlappingPairs`: Sweep and Prune (한 집합, 두 집합). 비교용 `...BruteForce` 포함 |
| `CCollision/RobotCollision.h` | URDF `<collision>` 도형 모으기(`BuildCollisionModel`), 링크별 월드 AABB(`ComputeLinkAabbs`), 도형별 월드 OBB(`ComputeShapeObbs`), 자기충돌(`FindSelfCollisions`: AABB 만, `CheckSelfCollisions`: AABB -> OBB 2단계) |

```cpp
simul::CollisionModelOptions opt;           // opt.fallbackToVisual = true 면 <collision> 이 없는 링크는 <visual> 로 대신함
simul::RobotCollisionModel model;
simul::BuildCollisionModel(robot, opt, &model);          // 메시는 이때 한 번만 읽는다

simul::SelfCollisionReport rep = simul::CheckSelfCollisions(robot, model, q);   // q: 관절 값
rep.aabbCandidates;   // 1단계: 링크 AABB 가 겹친 쌍 (링크 인덱스)
rep.collisions;       // 2단계: 그중 도형 OBB 까지 겹친 쌍
rep.shapePairs;       // 그 원인이 된 도형 쌍 (model.shapes 인덱스)
rep.linkAabbs, rep.shapeObbs;                   // 그릴 때 쓰는 상자들
// 장애물(AABB 목록)과의 충돌 후보: simul::FindOverlappingPairs(rep.linkAabbs, obstacles) -> (링크, 장애물) 쌍
```

- 면이 닿기만 해도 겹침으로 봅니다 (닫힌 구간). float 입력이라 정확히 닿는 경우는 반올림 영향을 받으므로 여유가 필요하면 `Inflated()` 나 `SelfCollisionOptions::margin`(두 단계 모두 적용)을 씁니다. OBB SAT 는 double 로 계산하고 반올림 때문에 겹침을 놓치지 않도록 아주 작은 여유를 둡니다.
- AABB: box / sphere / cylinder 는 기울어져 있어도 **정확한** 값이고, mesh 는 로컬 AABB 를 변환한 값이라 회전하면 조금 더 큽니다. OBB: box 는 정확, cylinder 는 (지름 x 지름 x 길이) 상자, sphere 는 지름 정육면체, mesh 는 로컬 AABB 를 도형 방향으로 돌린 상자입니다 (모두 실제 모양을 완전히 감싸는 보수적인 상자).
- 자기충돌은 부모-자식 링크를 기본으로 제외하고, `SelfCollisionOptions::ignoredPairs` 로 이름 쌍을 더 뺄 수 있습니다.
- 멤버 이름이 `min`/`max` 가 아니라 `lo`/`hi` 인 것은 Windows 의 `min`/`max` 매크로와 충돌하지 않게 하려는 것입니다.

**TriangleBvh** (`CCollision/Bvh.h`): 삼각형 메시의 BVH(경계 상자 계층). binned SAH(구간 16개)로 만들고, 광선이 닿지 않는 삼각형 무리를 통째로 건너뜁니다. 교차 판정은 `RaycastTriangles` 와 같은 코드(`internal/RayTriangle.h`)를 공유해 결과가 똑같습니다 (양면, 평행/퇴화/NaN/범위 밖 인덱스 삼각형은 건너뜀). 삼각형 20만 개에서 만드는 데 약 65 ms (한 번), 광선 하나가 약 0.75 µs (브루트포스는 약 1.3 ms). 메시 좌표에서 한 번 만들어 두면 링크가 움직여도 그대로 씁니다. 정점 배열은 복사하지 않으므로 쓰는 동안 살아 있어야 합니다.

## 중력 / 동역학 (CDynamics)

`CDynamics/Gravity.h` (`simul::dynamics`), 계산은 double, 단위는 SI 입니다.

```cpp
simul::dynamics::Gravity g;                                   // 기본 (0, 0, -9.80665) m/s^2, 루트 링크 좌표계
simul::dynamics::JointTorques t = simul::dynamics::GravityTorques(robot, q, g);   // 관절 이름 -> N*m (이동 관절은 N)
double com[3], mass;
simul::dynamics::CenterOfMass(robot, q, com, &mass);          // 로봇 전체 질량중심 (AMR 전도 여부 확인 등에 사용)

// 모든 관절(fixed, mimic 포함)이 각자 받는 하중. robot.joints 와 같은 순서
std::vector<simul::dynamics::JointLoad> loads = simul::dynamics::GravityLoads(robot, q, g);
//   loads[i].torque  : 움직이는 관절의 자기 몫 토크/힘 (mimic 도 합치지 않은 자기 몫)
//   loads[i].force / .moment : 그 관절이 아래쪽 링크 전체를 버티려고 전달하는 힘(N)과 모멘트(N·m), 루트 링크 좌표계
//                              (fixed 관절이면 체결부가 받는 하중)
// 한 관절의 토크에 링크마다 얼마씩 기여했는지 (합 = 그 관절의 torque)
std::vector<simul::dynamics::LinkContribution> parts = simul::dynamics::GravityContributions(robot, q, "shoulder_pitch", g);
```

- `GravityTorques` 는 정지 상태에서 각 관절이 중력을 버티려고 내야 하는 토크, 즉 운동방정식 `M q'' + C + g(q) = tau` 의 `g(q) = dU/dq` 입니다. 부호는 관절 좌표(axis 방향 회전, 이동)의 + 방향 기준입니다. 수평으로 뻗은 팔이 +y 축 관절에 달려 있으면 처지는 방향이 +y 회전이므로 음수가 나옵니다.
- `GravityTorques` 는 **모터가 내야 할 토크**라서 구동 관절(움직이면서 mimic 이 아닌 것)만 들어가고, mimic 관절의 토크는 `multiplier` 배로 원본 관절에 합쳐집니다. 관절마다 따로 보려면 `GravityLoads`(모든 관절, 합산 전 자기 몫)와 `GravityContributions`(링크별 기여)를 씁니다. 질량은 URDF `<inertial>` 의 `mass` 와 `origin`(질량중심)을 씁니다. 중력 토크에는 관성 텐서가 필요 없어서 아직 쓰이지 않습니다 (RNEA 로 확장할 때 쓸 예정).
- URDF `<dynamics damping friction>` 은 `Joint::damping/friction` 으로 읽어 두었고, 아직 계산에는 쓰이지 않습니다 (마찰 토크는 다음 단계).
- 검증: 진자/슬라이더 해석해와 비교하고, 가지가 있는 로봇(회전+이동+fixed+mimic+continuous, 회전된 관절/관성 원점)은 float FK 의 위치에너지를 수치 미분한 `dU/dq` 와 대조합니다.

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

## 조명 방식 (셰이더)

뷰어의 셰이더(`Core/Qt/src/ModelViewer.cpp` 위쪽, GLSL 1.20)에는 세 가지 조명 방식이 들어 있고, 실행 중에 `보기 -> 조명 방식` 에서 고르거나 `--shading lambert|phong|pbr` 로 정합니다. 기본은 PBR 입니다.

- Lambert: 주변광 + 난반사(cos). 무광.
- Blinn-Phong: Lambert + 하이라이트(하프벡터와 광택도).
- PBR: Cook-Torrance (D: GGX, G: Smith, F: Schlick) + 하늘/바닥 그라데이션 환경 반사. 재질은 금속성/거칠기 두 값이며, glTF 파일에 있으면 그 값을 쓰고 없으면(URDF 기본 도형, STL 등) 금속성 0, 거칠기 0.45 입니다. 셰이더에 항마다 한국어 주석이 있습니다.
- glTF/GLB 색: 재질 색(baseColorFactor), 정점 색(`COLOR_0`), 기본 색 텍스처(GLB 안에 든 이미지, data URI, 파일 옆의 PNG/JPEG)를 곱해서 그린다. 텍스처는 `TEXCOORD_0` 가 있어야 입혀지고, 읽지 못하면 경고에 이유가 남는다. URDF 에서 `<material><color>` 를 주면 그 색이 우선이다. 아직 안 읽는 것: 금속/거칠기 텍스처, 노멀 맵, 발광, `KHR_texture_transform`.
- 큰 파일 로드: 같은 파일은 한 번만 파싱한다(`LoadMeshCached`, 크기와 수정 시각이 같으면 메모리에서 재사용, 약 1GB 한도). 장면에 모델을 더할 때마다 앞의 모델을 다시 읽지 않고, 같은 텍스처를 쓰는 조각은 이미지 하나와 GL 텍스처 하나를 나눠 쓴다.

## 다른 Qt 프로그램에 3D 뷰 붙이기 (`SimulView`)

3D 뷰는 `CoreQt` 라이브러리의 `SimulView` 위젯 하나로 가져다 쓸 수 있습니다. 3D 화면(`ModelViewer`)과 클릭한 물체 옆 선택 패널만 들어 있고, **메뉴, 도크, 로그 같은 바깥 UI 는 만들지 않습니다.** 어떻게 보여주고 무엇을 켤지는 호스트가 고릅니다.

```cpp
#include "CoreQt/SimulView.h"

SimulView* view = new SimulView(parent, SimulView::kSelectionPanel);   // 켤 기능을 고른다
view->viewer()->AddFile("arm.urdf");                                   // 모든 기능은 viewer() 로
view->EnableFeature(SimulView::kMouseEdit, true);                      // 실행 중에도 켜고 끌 수 있다
connect(view->viewer(), &ModelViewer::modelDragFinished, host, &Host::onMoved);   // 이벤트는 시그널로
```

| 기능 (`Feature`) | 켜면 |
|---|---|
| `kSelectionPanel` | 클릭한 물체 옆에 정보와 관절 슬라이더 패널이 3D 화면 위에 뜬다 |
| `kMouseEdit` | 좌표계 화살표 이동, 회전 링, IK 끌기 (끄면 보기와 선택만) |
| `kDropFiles` | 파일을 끌어다 놓아 추가 (`filesDropped` 를 받는 쪽이 없으면 바로 추가) |
| `kMenu` | 3D 화면 **위쪽에 뜨는 메뉴줄** (기본: 파일 열기/모두 닫기, 보기, 물리). 호스트가 `SetMenus(QList<QMenu*>)` 로 자기 메뉴를 그대로 보이거나 `AddMenu(title)` 로 하나 더할 수 있다 |
| `kTree` | 3D 화면 안에 뜨는 **구조 트리 창** (제목줄을 끌어 옮기고, 오른쪽 아래로 크기 조절, ✕ 로 닫기). 클릭하면 선택, 충돌 링크는 빨갛게. 보여 줄 모델이 없으면 저절로 숨는다. 오른쪽 클릭은 `linkContextMenuRequested` 로 알려 주고, 받는 쪽이 없으면 "제거" 메뉴를 쓴다 |

GL 안에 들어 있는 UI 는 프로그램에서 함수로 켜고 끈다:

```cpp
view->SetMenuVisible(false);            // 메뉴줄    (MenuVisible() 로 확인)
view->SetTreeVisible(true);             // 구조 트리  (TreeVisible())
view->SetSelectionPanelVisible(false);  // 선택 패널  (SelectionPanelVisible())
view->SetUiScale(1.5);                  // 메뉴줄/트리/선택 패널 크기 배율 (0.6~2.5, UiScale() 로 확인, uiScaleChanged 신호)
connect(view, &SimulView::featuresChanged, ...);   // 창의 ✕ 로 닫혀도 알려 준다 (호스트 체크 표시용)
```

`viewer()` 로 모델 추가/제거(`AddFile`, `RemoveModel`, `CloseAll`), 위치/자세(`SetModelPlacement6`, `SetModelPose`), 관절 값(`SetJointValues`), 선택, 카메라, 표시 옵션(격자, 충돌 박스, 조명 방식, 라이다)을 모두 다룰 수 있습니다. 호스트가 받는 시그널: `robotChanged`, `selectionChanged`/`selectionPicked`, `jointValuesChanged`, `modelDragFinished`, `collisionsChanged`, `lidarScanned`, `structureChanged` 와 `SimulView::jointEdited`. 최소 예제는 `examples/embed_host.cpp`(빌드하면 `embed_host`), 동작 확인은 `tests/test_simulview.cpp`(창 없이, `QT_QPA_PLATFORM=offscreen`)입니다. 지금의 `simul_main` 도 같은 `SimulView` 를 쓰는 호스트 하나입니다.

## 단위와 카메라 (m, radian, 쿼터니언)

엔진 안쪽은 길이 m, 각도 radian, 방향 쿼터니언(x, y, z, w)이고, degree 는 사람이 보는 화면과 입력에서만 쓴다 (`3d/Units.h`, `3d/dhPose.h`).

- 카메라: `ModelViewer::SetCamera(target, distance, yaw, pitch)` (m, radian), `CameraTarget/CameraDistance/CameraYaw/CameraPitch`, 카메라 자신의 자세는 `CameraPose()` (눈의 위치 m + 방향 쿼터니언, x 가 앞, z 가 위). 마우스 조작량은 화면 기준이라 모델이 작든 크든 한 번 끌면 같은 만큼 움직인다.
- 모델 배치: `SetModelPlacement(index, dhPose)` / `ModelPlacementPose(index)` (위치 m + 쿼터니언). degree 로 주고받는 `SetModelPlacement6` 은 입력 칸용이다.
- 길이 기준 표시: 3D 화면 왼쪽 아래에 "격자 한 칸 10 cm · 카메라 거리 1.56 m". 처음 화면은 모델에 맞춰 자동으로 잡히므로(큰 건물도 작은 상자도 비슷한 크기로 보인다) 이 표시가 실제 크기를 알려 준다. `SetScaleInfoVisible(false)` 로 끈다. 아래 상태줄의 크기도 m 단위이고, 200 m 보다 크면 단위가 mm 가 아닌지 알려 준다.
- 파일 단위 변환: mm/cm/inch 로 만든 모델 파일이 너무 크게 나오면 메뉴 **파일 > 선택한 모델의 길이 단위 변환**, 또는 `SetModelUnitScale(index, 0.001)`. 원래 크기 기준의 절대 배율이라 겹쳐지지 않는다. 기본 도형(상자/원통/구)에는 적용하지 않는다.

## 모델 크기 배율

모델 전체를 원래 크기의 s 배로 키우거나 줄인다. 길이, 관절 위치, 도형, 메시가 모두 s 배가 되고, 질량은 같은 밀도로 보아 s^3 배, 관성은 s^5 배, 관절의 힘/토크 한계는 질량에 맞춰 따라간다 (회전 관절의 각도, 색, 라이다의 측정 범위는 그대로). 원래 크기 기준의 절대 배율이라 여러 번 바꿔도 겹쳐지지 않고, 1 로 돌리면 원래대로다. 기본 도형(상자/원통/구)도 된다.

- 화면: 오른쪽 속성 도크의 **크기 배율 (x)** 칸 (모델을 선택하면 값이 나온다)
- 명령행: `--scale 모델=배율` (모델 이름은 파일 이름, 예: `--scale arm=0.5`)
- 함수: `ModelViewer::SetModelScale(index, s, massScales = true)`, `ModelScale(index)`. 질량을 그대로 두려면 `massScales=false`. 엔진 밖 코드는 `urdf::ScaleRobot(robot, s, massScales, &out)` 를 쓴다.
- 파일 단위(mm, inch)만 맞추는 `SetModelUnitScale` 은 메시만 곱한다 (URDF 의 숫자는 그대로 m 인 경우).

## 화면 구성 (도크, 메뉴 숨기기, 3D 화면만)

- 구조, 관절, 로그, 라이다 그래프 도크는 제목줄을 끌어 **떼어 내서(플로팅) 아무 데나 놓거나**, 창 가장자리/다른 도크 위에 놓아 다시 붙이거나 탭으로 겹칠 수 있고, X 로 닫을 수 있습니다(`보기` 메뉴에서 다시 켭니다).
- **F10**: 메뉴 막대를 숨기고 보입니다. 숨기면 메뉴의 단축키는 동작하지 않지만 F10/F11 은 창에 직접 달려 있어 계속 먹습니다.
- **F11**: 메뉴, 상태바, 모든 도크(떼어 낸 것 포함)를 숨기고 **3D 화면만** 남깁니다. 이때 같은 메뉴(파일/보기/물리/센서/도움말, 동작과 체크 상태 그대로)가 **3D 화면 위쪽 메뉴줄**에 나옵니다. 다시 F11 이나 **Esc** 를 누르면 보이던 것만 원래 모습(떼어 낸 도크는 떼어 낸 채로)으로 돌아옵니다. 3D 화면 안의 구조 트리 창과 선택 패널은 3D 화면의 일부라서 그대로 남습니다 (필요하면 T 키나 `SetTreeVisible` 로 끕니다).
- **UI 크기**: 보기 > 화면 UI 크기 (크게 Ctrl+=, 작게 Ctrl+-, 기본 Ctrl+0), 명령행 `--ui-scale 1.5`. 트리 창은 오른쪽 아래 손잡이로 따로 크기를 바꿀 수 있다.
- 명령행: `--only-view` (처음부터 3D 화면만), `--hide-menu` (메뉴만 숨김).

## 로그 창

`보기 > 로그` (`L`) 로 아래쪽에 로그 창을 엽니다. 파일 열기/제거/모두 닫기, 모델 이동·회전과 위치 지정, 선택, OBB 충돌이 생기거나 사라질 때, 라이다 표시, 모터/중력 변경, 그리고 읽기 경고와 오류를 시간순으로 쌓습니다(수준별로 색이 다르고, 위의 콤보로 `경고 이상`/`오류만` 걸러 볼 수 있으며 `지우기` 로 비웁니다). 파일을 열지 못하는 것 같은 오류가 나면 로그 창이 저절로 열립니다.

## 선택한 물체 패널 (3D 화면 위)

물체를 클릭하면 그 옆에 패널이 떠서 이름, 소속 모델, 월드 위치/RPY, 조작 방법을 보여주고, **그 물체를 움직이는 관절 슬라이더**를 함께 보여줍니다(팔의 중간 링크면 그 링크의 관절 하나, 모델의 루트 링크면 그 모델의 모든 관절). 카메라를 돌려도 물체를 따라다니고, 빈 곳을 클릭하면 사라집니다. 패널의 슬라이더가 아닌 곳을 **마우스로 잡고 끌면 위치를 옮길 수** 있고(3D 화면 안으로 제한), 옮긴 자리는 물체 기준 상대 위치로 기억해 계속 따라다닙니다. 오른쪽 `관절` 도크는 기본으로 숨겨 두었고 `보기 > 관절 도크 (전체 관절)` 로 켜면 모든 관절과 중력 토크를 한꺼번에 볼 수 있습니다.

## 여러 모델 한 장면에 올리기

파일을 열면 이전 모델을 지우지 않고 **추가**합니다 (여러 파일을 한꺼번에 끌어다 놓아도 됩니다). 둘 이상이면 `world` 라는 가상 루트 아래에 각 모델이 `<이름>_attach` 고정 관절로 붙고, 이름이 겹치는 링크/관절은 `<모델이름>/` 접두어가 붙습니다. URDF 뿐 아니라 STL/OBJ/DAE/glTF 파일도 모델로 올릴 수 있습니다.

- 구조 트리에서 모델의 루트 링크를 **우클릭** → `위치 지정...`(x, y, z, z축 회전) / `제거`
- 파일 > 모두 닫기: 전부 지웁니다
- 명령행: `simul_main room.urdf arm.urdf --place arm=-2.5,1.5,0.02,40`

## 상단 메뉴

- 파일: 열기(Ctrl+O, 현재 장면에 **추가**), 모두 닫기, 종료
- 보기 > 메뉴 표시(F10), 3D 화면만 보기(F11)
- 보기: 화면에 맞춤(F), 격자, 구조 트리(T), 선택 좌표계(G) | 조명 방식(Lambert, Blinn-Phong, PBR) | 충돌 박스(OBB 는 O, AABB 는 B)
- 물리: 중력 방향, 중력 낙하, 모터 끄기(M) | 오브젝트 위치 초기화
- 센서: 라이다(레이저 표시, 그래프 표시). 모델에 라이다가 없으면 꺼져 있습니다.
- 도움말: 조작 방법(F1)

## 물류 공정 (CProcess)

물건을 A 지점에서 B 지점으로 옮기는 가장 단순한 공정이다. 로봇은 쓰지 않고, 물건이 직선으로 일정한 속도로 간다.
3D 화면과 독립이라 호스트가 `Step` 을 부르고 물건의 `position` 을 화면 모델에 넣어 준다 (`ModelViewer::SetModelPlacement`).
로봇이나 AMR 이 물건을 옮기게 되면 그쪽이 물건의 이동만 제어하고, 지점/상태/이벤트는 그대로 쓴다.

```cpp
#include "CProcess/Process.h"
using namespace simul::process;

Process p;
p.AddPoint("A", dhVector(0, 0, 0));
p.AddPoint("B", dhVector(3, 0, 0));          // m
int box = p.AddItem("box", "A");             // A 에 놓인 물건
p.Order(box, "B", 0.5);                      // B 로 0.5 m/s (출발 이벤트)
while (!p.AllIdle()) p.Step(0.02);           // 도착하면 AllIdle() 이 참 (도착 이벤트)
Array<Event> ev = p.TakeEvents();            // 출발/도착 이벤트와 시각 (도착 시각은 틱 안의 정확한 순간)
```

- 물건 상태: `kItemWaiting`(지점에 놓임) → `kItemMoving` → `kItemArrived` (다시 `Order` 하면 또 보낼 수 있다)
- `TimeToArrival(item)` 으로 남은 시간, `Time()` 으로 공정 시계를 본다.
- 이름이 겹치거나, 없는 지점이거나, 속도가 0 이하이거나, 같은 지점으로 보내거나, 이동 중에 다시 보내면 `false` 와 이유를 돌려준다.

### 3D 파일로 보기 (logistics_ab.urdf)

`Data/Object/samples/logistics_ab.urdf` 는 같은 공정을 3D 파일 하나로 만든 것이다 (로봇 없음, 링크 2개 + 관절 1개).
바닥판 위에 A 지점(초록, x=0)과 B 지점(주황, x=3 m)이 있고, 물건(파란 상자)이 직선 레일 관절 `item_slide`(0~3 m)에 달려 있다.
관절 값이 곧 A 에서 간 거리라서, 공정의 `travelled` 를 그대로 넣으면 화면의 물건이 공정과 같은 위치에 온다.

```
./build/bin/simul_main Data/Object/samples/logistics_ab.urdf --joint item_slide=1.5   # 물건이 A 와 B 의 가운데
```

## 바닥 모델 지정

모델을 둘 이상 올렸을 때, 그중 하나를 바닥으로 지정하면 다른 모델은 z=0 평면 대신 그 모델의 윗면 위로 떨어져 멈춘다.
물체의 꼭짓점마다 바로 아래 바닥 모델의 윗면(위를 향한 삼각형) 높이를 찾아 어느 한 점이라도 닿으면 멈추므로, 평면이든 곡면이든 같은 방식이다.
바닥 모델 바깥으로 나간 점은 z=0 평면까지 내려간다. 미끄러짐, 회전, 마찰은 없다 (닿은 자리에서 멈춘다).

- 3D 화면에서 모델 구조 트리의 모델 루트를 오른쪽 클릭 → "바닥으로 지정" / "바닥 지정 해제"
- 명령행: `--floor 모델이름` (예: `simul_main table.urdf box.urdf --floor table`)
- 코드: `viewer->SetFloorModel(index)`, `SetFloorByLink("링크이름")`, `FloorModel()`, `FloorHeightAt(x, y, &z)` (그 자리의 바닥 높이)

## 컨셉 문서

- [Python 스크립트 + DLL 제어기 구조 (컨셉 초안)](docs/concept_scripting_dll.md): 아직 코드에 반영되지 않은 설계 논의. 결정/제안/열린 질문을 구분해 둔다.
