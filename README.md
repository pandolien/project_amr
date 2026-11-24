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
| CEnv | 환경: 태양 위치 (날짜/시각/위치 -> 방위각, 고도, 일출/일몰) | 3d |
| CSensor | 센서 시뮬레이션 (2D/3D 라이다: 레이캐스트 부채꼴, 잡음, 범위 / 카메라: BGR 컬러 + 깊이 영상 / 힘·토크 센서: 관절 전달 하중 / IMU: 가속도·자이로·자세 / 엔코더·차동 구동 오도메트리: 휠·관절, 양자화·잡음, 위치 적분 / 모두 편향·잡음·포화) | CCollision, CDynamics, 3d |
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
- **중력 낙하**(모델을 둘 이상 올렸거나 한 번이라도 돌린 장면은 **모델 하나하나**가 따로 떨어지고, 자세와 관절 값이 반영된 가장 낮은 점이 바닥에 닿습니다. 끄는 동안에는 그 모델만 붙잡혀 있습니다): 이동으로 들어 올린 물체(정적 모델, 로봇 전체)는 놓으면 **바닥(z = 0 인 격자 평면)까지 떨어집니다**. 바닥에 닿으면 조금 튕기다 멈추고(`CDynamics/FreeFall.h` 의 `FallBody`, 등가속도 식을 그대로 써서 낙하 시간이 sqrt(2h/g) 와 맞습니다), 가장 낮은 점은 보이는 모양의 꼭짓점으로 잡습니다. 화살표를 끄는 동안에는 떨어지지 않고, 놓는 순간부터 떨어집니다. 중력은 `보기 > 중력 방향...` 의 수직(z) 성분만 쓰고(위로 향하거나 0 이면 떨어지지 않음), 이미 바닥 아래에 있는 물체는 건드리지 않습니다. **회전**: 모델을 돌려 놓았거나 둘 이상 올린 장면(모델마다 자세를 갖는 장면)에서 바닥이 z = 0 평면이면 모델은 **강체**로 떨어집니다(`CDynamics/RigidFall.h` 의 `RigidFall`). 기울어진 채 떨어지면 모서리로 닿아 구르다가 마찰로 한 면에 서서 멈추고, `SetModelMotion(model, 속도, 각속도)` 로 던져 준 속도와 회전은 공중에서 이어집니다. 질량 중심과 관성은 모양의 범위(균일한 상자)로 잡고, 접촉 후보는 꼭짓점의 볼록 껍질 근처 점들입니다(1/480 초 이하로 나눠 반발 + 쿨롱 마찰 충격량을 순차로 풉니다). **물체끼리 충돌**: 강체로 떨어지는 모델들은 서로 겹치지 않고 부딪힙니다(`StepRigidBodies`: 볼록 껍질의 면으로 접촉점을 구해 한꺼번에 푸는 충격량 방식이라 상자 위에 상자를 쌓을 수 있고, 잠든 모델은 세게 부딪히면 깨어나 밀려납니다). 끄는 동안의 모델은 움직이지 않는 장애물이고, 이미 바닥 아래에 있는 모델은 충돌에서 빠집니다. 바닥 모델(`SetFloorModel`)을 지정한 장면은 지금처럼 수직으로만 떨어집니다. 모델 하나만 올린 장면도 모양이 기울어져 있으면(위가 아니라 모서리로 닿으면) 구르며 떨어지고, 구르는 순간부터 모델마다 자세를 갖는 장면으로 바뀝니다. 시험용 URDF: `Data/Object/samples/fall_*.urdf` (`fall_box_tilted`, `fall_box_corner`, `fall_plate`, `fall_bar`, `fall_cylinder` — 열면 1~1.5 m 위에 기울어진 채 떠 있다가 떨어집니다. 예: `simul_main Data/Object/samples/fall_bar.urdf`, 확인용 `--settle --view-angle 30,35 --shot out.png`). `보기 > 중력 낙하` 로 끄고 켜며(`--no-fall` 은 처음부터 끔), `--settle` 은 확인용으로 낙하가 끝날 때까지 진행한 뒤 위치를 출력합니다.
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
- **설정**: `CameraSettings` 가로/세로 픽셀, 가로 시야각(rad, 세로는 비율로 결정), near/far, 깊이 잡음 표준편차(시드 고정), 배경색, **안티앨리어싱**.
- **화질(안티앨리어싱)**: `CameraSettings::antialias` 1~4 (1 = 끔). 픽셀을 n x n 개로 나눠 찍어 색을 평균하므로 경계의 계단이 부드러워지고, 시간은 n^2 배 걸린다. 깊이와 `targetIds` 는 픽셀 가운데에 가장 가까운 하위 픽셀의 값을 써서 경계에서 깊이가 섞여 허공에 뜬 점이 생기지 않는다. 해상도는 `width/height` 로 따로 정한다. 앱: `--camera-aa 3`, 영상 창의 안티앨리어싱 칸. 늘리면 영상 창 갱신(0.2초마다)이 느려질 수 있으니 큰 영상에서는 1~2 를 권한다.
- **렌즈 왜곡**: `CameraSettings::distortion[5]` = k1, k2, p1, p2, k3 (OpenCV/ROS `plumb_bob` 과 같은 순서와 뜻, 모두 0 이면 없음). 카메라 행렬 K 는 왜곡된 영상 기준이라 ROS `CameraInfo` 의 K, D 에 그대로 쓸 수 있다. k1 < 0 은 통 모양, k1 > 0 은 실패 모양. 영상의 픽셀마다 왜곡을 되돌려 광선을 쏘므로 깊이는 그대로 센서 +x 방향 거리(z-depth)이고, `CameraImage::PixelPoint` 도 왜곡을 되돌려 점을 구한다. 되돌릴 수 없는 픽셀(왜곡이 너무 커서 영상 구석이 겹치는 곳)은 못 본 것으로 둔다. 앱: `--camera-distort -0.3,0.1,0,0,0`, URDF `<camera><distortion><k1/><k2/><k3/><p1/><p2/></distortion></camera>`(Gazebo 와 같은 이름), 영상 창의 입력칸. 3D 뷰의 시야 표시선(프러스텀)은 왜곡 없는 핀홀 기준이다.
- **잡음**(기본 켜짐): 색 잡음 `colorNoiseStdDev`(0..1 비율, 기본 0.02 = 8비트로 약 5)는 모든 픽셀의 B, G, R 에 따로 가우시안 잡음을 더하고, 깊이 잡음 `noiseStdDev`(m, 기본 0.005)는 물체에 맞은 픽셀의 깊이에 더한다. 같은 `Camera` 로 계속 찍으면 프레임마다 다른 잡음이 나온다 (같은 시드로 새로 만들면 같은 잡음). 안티앨리어싱 때도 줄인 영상에 한 번만 더한다. 끄려면 `--camera-noise 0,0`, 영상 창의 "잡음" 입력칸, 또는 둘 다 0. URDF: `<camera><noise><type>gaussian</type><stddev>색</stddev><depth_stddev>깊이(m, 확장)</depth_stddev></noise></camera>` (없으면 기본 잡음). 정확한 값이 필요한 테스트는 잡음을 0 으로 둔다.
- **색**: 메시의 기본 색에 카메라 방향 조명(밝기 = 0.3 + 0.7×|법선·광선|)을 곱한다. 텍스처, 정점 색, 그림자, 반사는 아직 없다 (1단계). 나중에 GPU 렌더로 바꿀 수 있게 `Camera::Render` 한 곳만 바꾸면 되도록 두었다.
- **뷰어**: `ModelViewer::SetSensorCamera(link, settings, mount)` 로 링크에 달고(그 링크 자신은 안 찍는다), `RenderSensorCamera()` 를 부를 때만 찍는다 (매 프레임 찍지 않는다. 호스트가 원하는 주기로 부르고, `sensorCameraRendered` 시그널로도 받는다). 3D 화면에는 시야 절두체(노랑, 위쪽 변에 작은 삼각형)가 나온다. 3D 화면을 보는 시점(`CameraYaw` 등)과 이름이 비슷하니 헷갈리지 말 것.
- **영상 창**: `CoreQt/CameraView` (컬러/깊이 보기, 마우스를 올리면 픽셀의 깊이와 센서 좌표 표시). Qt 5.12 에는 `QImage::Format_BGR888` 이 없어 표시할 때만 채널을 바꿔 복사하고, 데이터는 BGR 그대로다.
- **URDF 에서 읽기**: Gazebo 형식 `<gazebo reference="링크"><sensor type="camera">` (`depth_camera`, `rgbd_camera` 도 같은 카메라로 읽음) 의 `<pose>`, `<camera><horizontal_fov>`, `<image><width/><height>`, `<clip><near/><far>` 를 읽는다 (`Robot::cameras`, `CameraDef`). 없는 값은 기본값(320x240, 60도, 0.05~10 m). 여럿이면 첫 번째만 쓴다. 앱은 URDF 의 카메라를 자동으로 달아 두고 영상 창은 메뉴 센서 > 카메라 영상 으로 켠다. 예제: `Data/Object/samples/camera_scene.urdf` (탁자 위 색 물체 4개를 보는 팬/틸트 카메라, 슬라이더 `pan`, `tilt` 로 돌려 본다).
- **앱**: `simul_main 모델.urdf --camera 링크,x,y,z --camera-size 640x480 --camera-fov 70 --camera-range 0.05,10 --camera-save out/c` (명령행이 URDF 보다 우선하고 영상 창이 같이 뜬다. `--camera-save` 는 `c.png` 컬러, `c_depth.png` 회색 깊이, `c_depth16.png` 16비트 mm 를 저장한다). 메뉴 센서 > 카메라 영상 으로 창을 켜고 끈다.
- **검증**: `test_ccamera` (깊이 해석해, BGR 순서, 영상 방향, 가림, near/far, 잡음 재현성, BVH 일치, 내부 변수, 예외), `test_simulview` (링크에 달아 찍기, 시그널).

## 힘/토크 센서 (CSensor: ForceTorque.h)

로봇암 손목이나 AMR 과 암의 체결부에 다는 6축 F/T 센서입니다. 센서는 관절 하나(보통 fixed)에 붙고, 그 관절이 **자식 쪽 전체를 붙들려고 전달하는 하중**을 읽습니다 (`측정값 = 아래쪽 링크의 관성력 + 중력 반력 - 아래쪽에 걸린 외력`).

- 계산: `dynamics::JointWrenches` (재귀 뉴턴-오일러) 로 관절 하중을 구하고, 센서 좌표계(`FtSettings::pose`, 자식 링크 기준)와 센서 원점 기준 모멘트로 옮긴 뒤 편향, 가우시안 잡음, 측정 범위 포화를 적용합니다. 단위는 N, N·m.
- 부호: 기본 `kFtParentToChild` (부모가 자식에게 가하는 하중, 정지한 센서는 +z 로 무게를 받침). `kFtChildToParent` 는 부호를 뒤집습니다.
- 외력: `FtExternalWrench` (링크, 작용점, 힘, 토크, 루트 좌표계) 를 주면 센서 아래쪽 링크의 것만 반영합니다. 위쪽 링크에 건 힘은 센서가 못 느낍니다.
- 중력: 루트 좌표계 벡터로 넘깁니다. AMR 이 기울어 있으면 IMU 로 구한 방향을 넣으면 됩니다.
- URDF: `<gazebo reference="관절"><sensor type="force_torque">` 의 `<pose>`, `<force_torque><measure_direction>`, `<noise><stddev>` (힘), `<noise><torque_stddev>` (토크, 확장) 를 읽어 `Robot::forceTorques` 에 담고 `FtSettingsFromDef` 로 설정으로 바꿉니다. 없는 관절을 가리키면 읽기가 실패합니다.
- 검증: `tests/test_cforcetorque.cpp` (정지 무게와 모멘트의 해석해, 센서 자세, 가속 관성력, 외력, 부호, 편향/잡음/포화, URDF).

```cpp
simul::sensor::FtSettings s;
s.joint = "wrist";                 // fixed 관절
s.forceNoiseStdDev = 0.1f;
simul::sensor::ForceTorque ft(s);
simul::sensor::FtReading m = ft.MeasureStatic(robot, q);   // m.force[3] (N), m.torque[3] (N·m)
```

## IMU (CSensor: Imu.h)

3축 가속도계 + 3축 자이로 + 자세. AMR 본체나 로봇암 링크에 답니다.

- 가속도계는 "비력" (a - g) 을 잽니다. 수평으로 정지하면 +z 로 +9.80665 m/s^2, 자유낙하하면 0. 센서가 링크 원점에서 떨어져 있으면 그 점의 가속도(각가속도, 구심 가속도 포함)를 잽니다. 자이로는 rad/s, 자세는 센서 -> 루트 좌표계 쿼터니언 (w, x, y, z).
- 계산: `dynamics::LinkMotions` (RNEA 의 앞방향 전파, 링크마다 각속도/각가속도/비력). AMR 이 달리는 중이면 `dynamics::RootMotion` (루트 링크의 각속도, 각가속도, 선가속도) 을 넘기면 그 위의 링크 IMU 에 반영됩니다.
- 설정: 센서 링크, `pose`, 가속도/자이로/자세 잡음 표준편차, 가속도/자이로 편향, 측정 범위(포화), 시드. 같은 시드면 같은 잡음.
- URDF: `<gazebo reference="링크"><sensor type="imu">` 의 `<pose>`, `<imu><linear_acceleration><x><noise><stddev>`, `<angular_velocity><x><noise><stddev>` (x 축 값을 세 축에 같이 씀), 확장 `<imu><orientation><noise><stddev>` 를 읽어 `Robot::imus` 에 담고 `ImuSettingsFromDef` 로 설정으로 바꿉니다.
- 검증: `tests/test_cimu.cpp` (정지 +g, 기울기, 자이로, 구심/접선 가속도, 루트 운동, 자유낙하, 자세, 편향/잡음/포화, URDF).

```cpp
simul::sensor::ImuSettings s;
s.link = "base_link";
s.accelNoiseStdDev = 0.05f;
simul::sensor::Imu imu(s);
simul::sensor::ImuReading m = imu.Measure(robot, q);   // m.accel, m.gyro, m.orientation
```

## 엔코더와 오도메트리 (CSensor: Encoder.h, Odometry.h)

**엔코더**는 관절 하나의 회전/이동을 정수 카운트로 셉니다. 구동륜(휠 엔코더)과 로봇암 관절(관절 엔코더)에 같이 씁니다.

- 엔코더는 보통 모터 축에 달리므로 관절 1회전(이동 관절은 1 m)당 카운트 = `resolution * gearRatio` 입니다. 카운트는 내림으로 양자화하고, 증분형(기본)은 첫 측정 자리가 0, `absolute` 이면 관절 0 자리가 0 입니다 (`Reset()` 으로 영점을 다시 잡음).
- `invert` 로 카운트 방향을 뒤집습니다. 결과(`EncoderReading`)의 `position`, `velocity` 는 카운트 방향 기준이라, 좌우 바퀴 축 방향이 반대인 모델도 `invert` 만 맞추면 "앞으로 구르면 +" 가 됩니다. `dt` 를 주면 속도는 카운트 차이로(양자화됨), 안 주면 `qd` 로 구합니다.
- 잡음은 카운트 단위 표준편차(`noiseCounts`), 같은 시드면 같은 잡음.
- URDF: `<gazebo reference="관절"><sensor type="encoder">` 의 `<encoder><resolution|gear_ratio|invert|absolute>`, `<noise><stddev>` (Gazebo 표준이 아닌 이 프로젝트의 확장) 를 `Robot::encoders` 에 읽고 `EncoderSettingsFromDef` 로 설정으로 바꿉니다.

**차동 구동 오도메트리** (`DiffDriveOdometry`): 좌우 바퀴 회전 변화로 평면 위치 (x, y, yaw) 와 속도 (v, w) 를 중점법으로 적분합니다. 바퀴 반지름과 윤거를 줍니다. 바퀴 슬립은 없다고 보므로 실제와 어긋나는 만큼이 오도메트리 오차입니다. `Update(좌 엔코더 측정, 우 엔코더 측정, dt)` 로 엔코더 값을 바로 먹일 수 있습니다 (첫 호출은 기준만 잡음).

- 검증: `tests/test_cencoder.cpp` (양자화, 방향, 증분/절대, 감속비, 속도, 이동 관절, 잡음, URDF, 직진/제자리 회전/반원 주행, 엔코더 -> 오도메트리 전체 경로).

```cpp
simul::sensor::EncoderSettings s;
s.joint = "left_wheel"; s.resolution = 2048; s.gearRatio = 10; s.invert = true;
simul::sensor::Encoder left(s);   // right 도 같은 방식
simul::sensor::DiffDriveGeometry g; g.wheelRadius = 0.1; g.trackWidth = 0.5;
simul::sensor::DiffDriveOdometry odom(g);
odom.Update(left.Measure(robot, q, qd, dt), right.Measure(robot, q, qd, dt), dt);   // odom.Pose().x, .y, .yaw
```

## 태양 (CEnv: Sun.h)

태양 위치를 고정하지 않고 **날짜/시각과 위치로 계산**한다 (NOAA 태양 위치 계산식, 오차 약 0.01~0.05도, 대기 굴절은 없음).

- **계산**: `simul::env::ComputeSun(SunTime, SunSite, &SunState)`. `SunTime` 은 현지 날짜/시각과 UTC 오프셋(한국 +9), `SunSite` 는 위도/경도(기본 서울 37.5665, 126.978)와 월드 +y 가 가리키는 방위. 결과는 방위각(북에서 시계 방향)/고도/적위/균시차와 월드 좌표의 태양 방향 `direction` (표면 -> 태양). `SunriseSunset` 은 일출/일몰(현지 분), 백야/극야는 false, `AddMinutes` 는 시간을 더하고(날짜 넘김, 윤년 처리) `SunTimeNow` 는 컴퓨터 시각.
- **월드 방위**: 기본은 월드 +x 가 동, +y 가 북, +z 가 위 (ROS ENU). 월드 +x 가 정북이면 `northBearingDeg = 270`.
- **뷰어**: `ModelViewer::SetSun({enabled, time, site})`, `SetSunTime` (시간 흐름용), `SetSunEnabled`, `sunChanged` 시그널. 켜면 주광이 카메라에 고정된 빛 대신 태양이 되어 화면을 돌려도 태양은 월드에 고정이다. 해 고도에 따라 세기, 색(낮을수록 붉음), 하늘색, 주변광이 바뀌고 해가 지면 어두운 밤이 된다 (3 가지 조명 방식 모두). **태양 조명은 기본으로 켜져 있다**(오늘 날짜 10:00, 서울). 끄려면 `--no-sun`, `SetSunEnabled(false)`, 태양 패널. **3D 화면에도 그림자**가 나온다 (바닥 모델이 없어도 바닥(z = 0) 바로 아래의 투명한 면에 그림자만 반투명으로 그린다): 태양 쪽에서 본 깊이 맵(2048x2048, RGBA8 에 나눠 담아 깊이 텍스처 확장이 필요 없다)을 그리고, 셰이더가 3x3 칸을 평균해 가려진 곳의 태양빛(난반사와 정반사)만 뺀다. 주변광은 남는다. 반투명 물체는 그림자를 드리우지 않고, 해가 졌거나 태양이 꺼져 있으면 그림자 맵을 그리지 않는다. `SetShadowsEnabled(false)`, `--no-shadow`, 태양 패널의 그림자 체크로 끈다. **태양 위치 표시**(기본은 꺼짐, 켜면 태양 조명일 때 보임): 3D 화면에 장면 중심에서 태양으로 가는 노란 선, 바닥 투영선, 북/동/남/서 눈금과 글자(월드 +y 의 방위 `northBearingDeg` 에 맞춰 돈다), 그날의 태양 경로 곡선과 정시 눈금/글자(평소에는 숨기고 태양 표시에 마우스를 올리거나 끄는 동안에만 보임), 위쪽에 시각·방위각·고도 글자를 그린다. **태양 끝의 십자를 왼쪽 버튼으로 끌면** 그날의 경로를 따라 시각이 바뀌고(날짜와 위치는 그대로) 태양 패널과 그림자도 같이 바뀐다. `SetSunGuideVisible(true)`, `--sun-guide`, 태양 패널의 "태양 위치 표시" 체크로 켠다. **하늘**: 기본으로 켜져 있다 (태양 조명이 꺼져 있어도 낮 하늘과 구름을 그린다, 태양 원반은 없음). 배경에 하늘색 그라데이션, 구름(프랙탈 값 잡음), 태양 빛무리를 그린다 (낮엔 흰 구름, 해 질 녘엔 붉게, 밤엔 어둡게). `SetSkyEnabled(false)`, `--no-sky`, 태양 패널 체크로 끈다. 구름은 위를 볼 때만 보이므로 `--view-angle 방위각,고도각`(음수 고도각 = 올려다봄)으로 시점을 정해 확인할 수 있다. 3D 화면 전용이며 센서 카메라 영상의 하늘은 단색 하늘색이다. 태양 모드에서는 어두운 면이 덜 어둡도록 보조광과 주변광을 키웠다. 그림자 범위는 모든 불투명 물체의 경계 구라서 아주 큰 바닥이 있으면 가장자리가 거칠어진다.
- **센서 카메라**: 태양이 켜져 있으면 카메라 영상도 같은 태양으로 비추고 **그림자**를 그린다 (태양 쪽으로 광선을 하나 더 쏴서 가려지면 태양 빛을 뺀다). `Camera::SetSun(SunLight)` 로 코어에서 직접 줄 수도 있다. 하늘색이 배경이 된다.
- **앱**: 보기 > 태양 (Ctrl+S) 도크에서 켜기, 날짜/시각, 시간대, 위도/경도, 월드 +y 의 방위, "지금 시각으로", 시간 흐르게 하기(초당 N분)를 바꾸고 방위각/고도/일출/일몰을 본다. 명령행: `--sun 2026-06-21T14:30` (현지 시각, 생략하면 오늘 날짜의 10:00. 기본 시각은 현재 시각을 따라가지 않는다), `--sun-now` (지금 시각), `--sun-site 위도,경도[,UTC오프셋[,월드+y방위]]` (위치만 주고 오프셋을 안 주면 경도/15 반올림). 예: `simul_main Data/Object/samples/camera_scene.urdf --sun 2026-06-21T19:15 --camera-save out/c --shot out/s.png`.
- **검증**: `test_cenv` (서울 하지/동지/춘분 정오 고도와 방위, 아침/오후 방위, 밤, 적도/남반구, 월드 방위 회전, 일출/일몰, 백야/극야, 날짜 넘김과 잘못된 입력), `test_ccamera` (그림자, 밤, 낮은 태양, 하늘색 배경), `test_simulview` (뷰어/패널 연동).

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

## ArUco / ChArUco 마커 (CMarker)

센서 카메라가 볼 수 있는 ArUco 마커와 ChArUco 보드를 만든다. OpenCV 의 `DICT_4X4/5X5/6X6/7X7_1000` 과 같은 마커이고 (표는 `tools/gen_aruco_tables.py` 로 OpenCV 에서 뽑아 `Core/src/CMarker/ArucoTables.inc` 에 넣어 두었다. 빌드에 OpenCV 는 필요 없다), ChArUco 배치도 OpenCV 4.7 이후 기본(`legacyPattern=false`)과 같다.
마커는 "흰 판" 메시와 "검은 칸" 메시 두 개로 만들어진다 (검은 칸은 판 위로 0.4 mm 띄운다). 센서 카메라는 물체마다 색 하나로 그리므로, 두 메시에 흰/검은 재질을 주면 영상(BGR)에도 3D 화면에도 마커가 정확히 보인다.

```
simul_main --make-marker aruco,4x4,0,0.15 --marker-dir Data/Object/markers
simul_main --make-marker charuco,4x4,5,7,0.04,0.03 --marker-dir Data/Object/markers
```

- `aruco,사전,id,한변(m)`: 한 변은 검은 테두리 포함. 흰 여백은 칸 하나. 좌표계는 OpenCV 와 같다 (원점 가운데, x 오른쪽, y 위, z 보는 쪽).
- `charuco,사전,가로칸,세로칸,칸길이(m),마커한변(m)[,첫id]`: 원점 왼쪽 위, x 오른쪽, y 아래, z 보드 안쪽. 왼쪽 위 칸이 검은색이고 흰 칸에 id 가 위에서 아래로 0, 1, 2 ... 순서로 들어간다.
- 결과는 `이름_plate.stl`, `이름_ink.stl` 과 이 둘을 쓰는 `이름.urdf` 이다. 코드에서는 `simul::marker::BuildAruco / BuildCharuco / WriteMarkerFiles` (`Core/include/CMarker/Marker.h`).
- 기본 제공: `Data/Object/markers/` (ArUco 4x4~7x7 id 0, 4x4 id 20, ChArUco 4x4 / 5x5 5x7 보드), 장면 예제 `Data/Object/samples/marker_scene.urdf` (`--camera-save` 로 찍으면 OpenCV `ArucoDetector` / `CharucoDetector` 가 18개 마커와 보드 코너 24개를 모두 찾는다).
- 마커 칸이 몇 픽셀은 되어야 읽히므로 카메라 해상도와 거리를 맞춘다 (예제는 1280x960, 가로 시야각 40도, 약 1.5 m).

## 시뮬레이션 루프 (CSimul)

- **엔진 (`Core/include/CSimul/Engine.h`, Qt 무관)**: 오브젝트 등록부(이름 -> 핸들, 지워졌다 다시 생겨도 옛 핸들은 거부), 명령 큐, 시간(틱), `move_to`/`move_joint`(속도로 이동), 이벤트(`added removed arrived joint_reached command_failed`). 한 틱 순서: 등록부 맞춤 -> 큐의 명령 적용 -> 이동 진행 -> 시간 증가 -> 이벤트 전달. 낸 명령은 다음 틱에 적용되므로 호출하는 쪽이 느려도 결과는 같다(lockstep).
- 엔진은 `Backend` 인터페이스(오브젝트 읽기/쓰기, 충돌 읽기)로 장면을 다룬다. 3D 창 뒤의 `ViewerBackend`(Qt)와 창 없는 `HeadlessScene`(`CSimul/Headless.h`)이 있고, 후자를 공유 라이브러리로 내보낸 것이 아래의 `simul_core` 이다.
- **python 은 앱 안에 두지 않는다.** 시뮬레이션 코어는 공유 라이브러리(다음 절)로 나가고, python 쪽(에디터)은 별도 브랜치다.

### 오브젝트 / 관절 값 패널

3D 창의 "오브젝트 / 관절 값" 도크 (Ctrl+J, 시작할 때 `--object-panel`). 장면의 오브젝트마다 이름, 종류, 위치(m)와 방향(roll pitch yaw, 도)이 나오고, 펼치면 관절 이름, 지금 값(rad 또는 m), 한계가 나온다. 오브젝트 이름과 관절 값을 확인하는 용도다.

## 공유 라이브러리 (DLL / so): simul_core, simul_gui

Main 의 결과물은 3D View 와 공유 라이브러리다. 다른 프로그램(python 에디터 등)이 C 인터페이스로 불러 쓴다.

| 라이브러리 | 내용 | 헤더 | 의존 |
|---|---|---|---|
| `simul_core` (`libsimul_core.so` / `simul_core.dll`) | 모델 올리기, 오브젝트·관절 상태, 충돌, 명령(이동/관절), 시간 진행, 이벤트 | `api/include/simul_api.h` | Qt 없음 |
| `simul_gui` (`libsimul_gui.so` / `simul_gui.dll`) | 3D 화면: 별도 창(`simul_window_open`) 또는 호출한 쪽 창에 넣는 위젯(`simul_window_create_embedded` + `simul_window_widget`). 보기 전용이고 상태는 simul_core 에서 읽는다 | `api/include/simul_gui.h` | Qt 5 |

- 빌드하면 `build/lib`(Windows 는 `build/bin/<구성>`)에 생긴다. 단위: 길이 m, 각도 rad, 시간 s, 자세 쿼터니언 (x, y, z, w).
- 명령(`simul_set_pose`, `simul_set_joint`, `simul_move_to`, `simul_move_joint`, `simul_stop_motion`)은 큐에 들어가 다음 `simul_step` 시작에 적용된다. 이벤트(추가, 삭제, 도착, 관절 도달, 명령 실패)는 `simul_take_events` 로 받는다.
- 사용 예와 시험: `tests/test_capi.cpp`. python 에서 쓰는 쪽(ctypes 래퍼, 에디터)은 `editor` 브랜치에 있다.

## 프로젝트 파일 (project.simul)

장면(모델과 배치)을 폴더 하나로 묶어 `simul_main --project my_project/project.simul` 로 연다.

```
my_project/
  project.simul          <- 설명 파일 (XML)
  models/arm.urdf ...    <- 장면에 올릴 모델 (mesh 파일 포함, 상대 경로)
```

```xml
<simul_project version="1" name="arm_demo">
  <model file="models/arm.urdf" x="0" y="0" z="0" roll="0" pitch="0" yaw="0"/>   <!-- 위치 m, 자세 rad (생략하면 0) -->
</simul_project>
```

- 예제: `Data/Projects/arm_demo/`. 파일 안의 경로는 프로젝트 폴더 밖을 가리킬 수 없다 (`..`, 절대 경로 거부).
- python 등록, 프로젝트 저장 묶기(`--pack`)는 없다. python 프로젝트(main.py + URDF + 오브젝트별 python)는 에디터 쪽의 일이다.

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
- **F11**: 메뉴, 상태바, 모든 도크(떼어 낸 것 포함)를 숨기고 **3D 화면만** 남깁니다. 이때 같은 메뉴(파일/보기/물리/센서/도움말, 동작과 체크 상태 그대로)가 **3D 화면 위쪽 메뉴줄**에 나옵니다. 다시 F11 이나 **Esc** 를 누르면 보이던 것만 원래 모습(떼어 낸 도크는 떼어 낸 채로)으로 돌아옵니다. 3D 화면 안의 구조 트리 창과 선택 패널은 3D 화면의 일부라서 그대로 남습니다 (필요하면 Ctrl+T 나 `SetTreeVisible` 로 끕니다).
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
- 보기: 화면에 맞춤(Ctrl+F), 격자, 구조 트리(Ctrl+T), 선택 좌표계(Ctrl+G) | 조명 방식(Lambert, Blinn-Phong, PBR) | 충돌 박스(OBB 는 Ctrl+Shift+O, AABB 는 Ctrl+B)
- 물리: 중력 방향, 중력 낙하, 모터 끄기(Ctrl+M) | 오브젝트 위치 초기화
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

## 주민 순찰 (마을 기본 장면)

`simul_main` 을 인자 없이 켜면(`./run.sh` 도 같다) `Data/Object/merchen_village.glb`(동화풍 마을 지도)와 주민 4명(`Data/Object/villagers/villager_N.glb`)이 열리고, 주민이 바로 걷기 시작한다. 빈 뷰어로 켜려면 `--empty`. 주민은 엔진이 움직이므로 이 기본 장면에서는 중력 낙하가 꺼져 있다.

이동은 단순하다. `Data/Object/villagers/waypoints.txt` 의 지점(x y z, Z-up, m)을 위에서 아래로 직선으로 차례로 가고, 도착 이벤트가 오면 다음 지점으로 보낸다. 끝에 닿으면 처음으로 돌아가고, 가는 방향을 바라본다.

주민끼리 겹칠 것 같으면 A*(`Core/include/CSimul/AStar.h`, 0.25 m 격자)로 비켜 간다. 0.25초마다 앞길이 다른 주민(반지름 0.9 m)에게 막혔는지 보고, 막히면 가는 방향의 오른쪽으로 둘러 가는 길을 다시 짠다. 마주 오는 둘이 서로 오른쪽으로 비켜 엇갈린다. 나무, 집, 성, 풍차, 연못, 가로등 같은 움직이지 않는 장애물도 피한다. `Data/Object/villagers/obstacles.txt`(한 줄에 `x y 반지름`, 엔진 좌표 m)에 원으로 적혀 있고, 지점 파일과 같은 폴더에 있으면 자동으로 읽는다. 지도(`merchen_village.glb`)와 같은 생성기가 같은 값으로 내보냈고 126개다. 주민 몸 반지름 0.4 m 를 더해서 피한다. 코드에서는 `patrol.ParseObstacles(text, &err)`.
마을 기본 경로 4명을 300초 돌려 본 결과, 주민끼리의 최소 간격은 피하기를 끄면 0.00 m(겹침), 주민만 피하면 0.65 m, 장애물까지 주면 0.53 m 였다(장애물이 있으면 비킬 자리가 좁아진다; 아래 밀어내기를 더한 뒤 수치는 아래에 있다). 기본 경로는 원래 길을 따라 설계돼서 나무와 집에는 거의 닿지 않는다 (장애물 안쪽에 들어간 프레임 0, 최소 여유 0.34 m → 0.40 m). 장애물을 피하는 효과가 뚜렷한 것은 나무 사이를 지나는 경로를 직접 적었을 때이고 `tests/test_cpatrol.cpp` 에서 확인한다. 서로 다른 방향에서 동시에 모이는 경우에는 지금 위치만 보고 짜므로 0.9 m 까지는 못 벌린다.

**겹치거나 붙어서 출발한 경우**: 주민마다 같은 규칙으로 따로 계산하므로 입력이 똑같으면 결과도 똑같아서(대칭) 영영 안 갈라질 수 있다. 그래서 두 가지를 더했다. (1) 서로 0.6 m 안으로 붙으면 번호가 작은 쪽과 큰 쪽을 반대로 밀어 떼어 놓는다 (정확히 같은 자리면 번호가 작은 주민의 가는 방향에 수직으로 가른다). 한 틱에 최대 1.5 m/s 로 민다. (2) 누가 반경 안에 붙어 있으면 길 확인을 0.25초가 아니라 0.05초마다 한다. 붙은 둘을 네 가지로 시험하면(같은 방향, 마주 보기, 같은 목표, 완전히 같은 자리) 모두 0.2초 안에 0.6 m 이상 벌어지고 그 뒤 0.6 m 밑으로 좁아지지 않는다 (`tests/test_cpatrol.cpp` 의 `testSeparation`). 마을 기본 경로 300초에서 주민끼리의 최소 간격은 장애물 없이 0.75 m, 장애물을 주면 0.67 m 였다. `SetSeparation(0)` 으로 밀기를 끈다. 코드에서 `patrol.SetAvoidance(false)` 로 끈다.

**A\* 시각화**: 파일 메뉴 → "A\* 시각화"(Ctrl+K)를 열면 주민이 비켜 가려고 길을 새로 짤 때마다 그 탐색 과정이 위에서 본 그림으로 재생된다. 파란 칸은 A*가 꺼내 살펴본 칸(꺼낸 순서대로 진해지고 주황 칸이 방금 꺼낸 칸), 빨간 원은 피할 구역(다른 주민 반경 0.9 m), 노란 칸은 격자 위 최단 경로, 초록 선은 직선으로 줄인 최종 경로다. 재생/일시정지, 처음부터, 속도(느리게/보통/빠르게), 슬라이더로 한 칸씩 돌려 볼 수 있다. 계산은 0.25초마다 나오므로 재생이 끝나고 2.5초 뒤에 가장 최근 계산으로 바뀐다(체크를 끄면 바뀌지 않는다). 도크를 닫으면 기록을 남기지 않는다. 주민끼리 마주치기 전에는 비어 있다(기본 마을에서는 보통 1분쯤 걸린다).
주민을 클릭하면 그 주민을 **위에서 내려다보는 탑뷰**로 바뀌어 따라가고, 그 주민을 키보드로 직접 움직일 수 있다. **W/A/S/D** 는 화면 위/왼/아래/오른쪽으로 걷고(**Shift** 로 달리기), 캐릭터는 가는 쪽을 본다. **Q/E**(또는 ←/→)는 화면을 돌린다. 나무·집·연못 등과 다른 주민에게는 막혀서 지나가지 못한다. 조종하는 동안 그 주민은 순찰·대화에서 빠지고, 끝내면 가던 길을 다시 간다. 다른 주민을 클릭하면 그쪽으로 옮긴다. 3D 화면 오른쪽 위의 "◀ 원래 시점으로 (Esc)" 버튼이나 Esc 로 클릭하기 전의 시점(대상, 거리, 각도)으로 돌아온다. 구현은 `main.cpp` 의 `enterFirstPerson` / `drive` / `updateFirstPerson`.
주민을 늘리거나 길을 바꾸려면 지점 파일에 `[모델 이름]`, `speed`(m/s), 지점 줄을 더하면 된다. 모델 이름은 파일 이름이다.

```
./build/bin/simul_main                                              # 마을 + 주민 (기본)
./build/bin/simul_main 지도.glb 주민.glb --patrol 지점파일.txt      # 내 모델과 내 지점 파일로
./build/bin/villager_patrol Data/Object 60                          # 창 없이 60초 걷게 하고 1초마다 위치를 찍는다
```

코드: `Core/include/CSimul/AStar.h`(A* 와 탐색 기록 `AStarTrace`), `Core/Qt/include/CoreQt/AStarView.h`(시각화 위젯), `Core/include/CSimul/Patrol.h`(엔진 `Listener`, 지점 파일 읽기 + `move_to` 연결 + 비켜 가기, 시험은 `tests/test_cpatrol.cpp`, `tests/test_castar.cpp`), 뷰어 연결은 `main.cpp` 의 `startPatrol`, 창 없는 예제는 `examples/villager_patrol.cpp`.

## 주민 대화 (무료 로컬 LLM)

마을의 주민끼리 마주치면(2.2 m 안) 둘 다 멈춰서 서로를 보고, 번갈아 4마디쯤 이야기한 뒤 다시 걷는다. 대사는 머리 위 말풍선으로 보이고(읽을 시간만큼 3~7초), 같은 둘은 30초 동안 다시 대화하지 않는다. 대화 내용은 로그 도크(보기 메뉴)에도 쌓인다.

대사는 **내 PC 에서 돌리는 무료 모델**(Ollama)에 물어서 만든다. 키도 요금도 없고 인터넷이 필요 없다. 주민마다 성격이 있어서(`Data/Object/villagers/personas.txt`: `[모델 이름]`, `name`, `persona`), 모델은 그 성격과 지금까지 나눈 말을 보고 다음 한 마디를 만든다. 성격 파일은 마음대로 고치면 된다.

```
# Ollama 설치 (Linux) 와 모델 받기: 1.5B 모델은 CPU 만으로도 한 마디에 몇 초 걸린다
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:1.5b
./build/bin/simul_main                                   # 기본: http://127.0.0.1:11434, 모델 qwen2.5:1.5b
./build/bin/simul_main --llm-model qwen2.5:3b            # 더 큰 모델 (한국어가 더 낫지만 느리다)
./build/bin/simul_main --llm-url http://다른PC:11434     # 다른 PC 에서 돌리는 Ollama
./build/bin/simul_main --no-llm                          # LLM 없이 정해 둔 대사만
./build/bin/simul_main --no-chat                         # 대화 없이 걷기만
```

- **LLM 서버가 없거나 실패해도 말은 한다.** 연결 실패, 모델 없음, 35초 시간 초과, 이상한 응답이면 그 한 마디는 정해 둔 대사(인사, 맞장구, 작별)로 대신하고, 1분 동안은 LLM 에 묻지 않고 정해 둔 대사만 쓴 뒤 다시 시도한다. 처음 실패하면 로그에 이유가 나온다 (로그 기록에는 "(정해 둔 대사)" 가 붙는다).
- **한 번에 하나씩 묻는다.** 작은 모델은 동시 요청을 못 견디므로 줄을 세우고, 4개를 넘게 밀리면 오래된 것부터 정해 둔 대사로 넘긴다.
- **답은 정리해서 쓴다.** 첫 줄만, 따옴표와 앞에 붙은 "이름:" 을 떼고, 40글자를 넘으면 문장 끝에서 자른다.
- **서버 형식.** Ollama 의 `/api/generate` (`stream:false`) 로 보내고, 응답의 `response`(없으면 `content`)를 읽는다. llama.cpp 서버(`/completion`)는 주소가 달라서 바로는 안 된다.
- Windows 에서는 Qt5Network 가 필요하다 (windeployqt 가 같이 넣어 준다). 기존 빌드 폴더에서는 CMake 를 다시 구성해야 한다.

코드: `Core/include/CSimul/Chat.h`(만남 감지, 차례, 프롬프트와 응답 글 처리 — 네트워크와 Qt 가 없다, 시험은 `tests/test_cchat.cpp`), `Core/Qt/include/CoreQt/LlmClient.h`(Ollama 호출), `SpeechBubbles.h`(말풍선), 연결은 `main.cpp` 의 `setupChat`. 만나면 `Patrol::Hold` 로 세우고 끝나면 `Patrol::Release` 로 다시 걷게 한다.

## 바닥 모델 지정

모델을 둘 이상 올렸을 때, 그중 하나를 바닥으로 지정하면 다른 모델은 z=0 평면 대신 그 모델의 윗면 위로 떨어져 멈춘다.
물체의 꼭짓점마다 바로 아래 바닥 모델의 윗면(위를 향한 삼각형) 높이를 찾아 어느 한 점이라도 닿으면 멈추므로, 평면이든 곡면이든 같은 방식이다.
바닥 모델 바깥으로 나간 점은 z=0 평면까지 내려간다. 미끄러짐, 회전, 마찰은 없다 (닿은 자리에서 멈춘다).

- 3D 화면에서 모델 구조 트리의 모델 루트를 오른쪽 클릭 → "바닥으로 지정" / "바닥 지정 해제"
- 명령행: `--floor 모델이름` (예: `simul_main table.urdf box.urdf --floor table`)
- 코드: `viewer->SetFloorModel(index)`, `SetFloorByLink("링크이름")`, `FloorModel()`, `FloorHeightAt(x, y, &z)` (그 자리의 바닥 높이)

## 컨셉 문서

- [Python 스크립트 + DLL 제어기 구조 (컨셉 초안)](docs/concept_scripting_dll.md): 아직 코드에 반영되지 않은 설계 논의. 결정/제안/열린 질문을 구분해 둔다.
