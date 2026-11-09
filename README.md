# project_amr

AMR(이동 로봇) 위에 로봇암을 연결하여 통합 제어하는 시스템.

## 개발 환경

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 20.04 LTS |
| 컴파일러 | gcc (9.x) |
| 빌드 | CMake 3.16+ |
| 언어 | C++11 |
| GUI | Qt 5.12 (LGPLv3, 동적 링크) |

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
├── Gui/                  # 이 프로젝트 전용 Qt5 화면
├── tests/
└── build/                # 빌드 산출물 (bin, lib 포함, git 제외)
```

의존 방향: `Gui -> CoreQt -> Core`, `amr_main -> Core` (역방향 금지, Core는 Qt를 참조하지 않음)

| 모듈 | 역할 | 의존 |
|---|---|---|
| CMat | 행렬 연산 | - |
| 3d | 3차원 공간 | CMat |
| CDynamics | 동역학 연산 | 3d |
| CSimul | 시뮬레이션 | CDynamics |
| CSock | 외부 통신 | - |

헤더는 `#include "CMat/CMat.h"` 형태로 사용합니다.
`Core/src` 아래에 `.cpp`를 추가하면 CMake가 자동으로 인식합니다.
재사용할 Qt 위젯은 `Core/Qt`(`#include "CoreQt/..."`), 이 프로젝트 전용 화면은 `Gui/`에 둡니다.

## 빌드

```bash
sudo apt install build-essential cmake qtbase5-dev   # Ubuntu 20.04 (Qt 5.12.8)

cmake -S . -B build
cmake --build build -j

./build/bin/amr_main
./build/bin/amr_gui
```

GUI 없이 빌드: `cmake -S . -B build -DAMR_BUILD_GUI=OFF`

## Qt 라이선스 주의 (LGPLv3)

- Qt는 **동적 링크**로만 사용합니다 (정적 링크 금지).
- GPL 전용 모듈(Qt Charts, Qt Data Visualization 등)은 사용하지 않습니다.
- 배포 시 Qt LGPL 고지문과 라이선스 사본을 함께 포함합니다.
