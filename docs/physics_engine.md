# 물리엔진: 저장소 현황과 구성 (초안)

작성: 2026-11-23 · 상태: **현황 정리 + 설계 초안 (코드 미반영)**
기준: `pandolien/project_amr` main, 커밋 `90a3c41` (Add IMU sensor) 을 읽고 정리했다.

항목 표시: **[확인]** 코드에서 읽은 사실, **[제안]** Claude 제안(미확정), **[열림]** 사용자 결정 필요.

## 1. 지켜야 할 규약 [확인]
- C++11, gcc 9 / CMake 3.16+, Ubuntu 20.04. MSVC 빌드도 유지한다.
- `Core` 는 **표준 라이브러리 금지**: `std::` 와 `<vector>`, `<string>`, `<cmath>` 등을 쓰지 않고 `CBase`(`Array`, `String`, `Map`, `HashMap`, `Sqrt`…)를 쓴다. C 런타임(`<math.h>`, `<stdint.h>`, `memcpy`)은 허용. 예외 금지. CTest `no_std` 가 검사한다.
- 네임스페이스 `simul::dynamics`, `simul::sim`. 단위는 SI (m, rad, s), 자세는 단위 쿼터니언(`dhPose`).
- `Core/src` 에 `.cpp` 를 추가하면 CMake 가 자동 인식 (GLOB). 테스트는 `tests/CMakeLists.txt` 에 `add_executable` / `add_test` 로 추가하고 `simul_set_warnings` 를 건다.
- `3d` 의 `dhVector`/`dhQuat`/`dhMat` 은 **float**, 동역학은 **double** 로 계산한다 (`RigidFall` 은 `double[3]`, `double[9]` 배열).

## 2. 이미 있는 것 [확인]
**`Core/CDynamics/FreeFall`** (`FallBody`): 바닥 위 수직 운동 하나. 등가속도 정확식, 반발, 정지 판정.

**`Core/CDynamics/RigidFall`** (665줄, `tests/test_cdynamics.cpp` 에 테스트):
- 6자유도 강체 하나. 모양은 몸체 좌표의 접촉 후보 점들(`SelectContactPoints` 가 꼭짓점에서 최대 76개를 고름) + 볼록 껍질 면 + 경계 구.
- 한 스텝을 1/480 s 이하로 쪼개 중력 + 자이로 효과로 속도 갱신, 평면 아래 점마다 **순차 임펄스**(반발 + 쿨롱 마찰), 위치를 평면 위로 밀어 올림.
- 질량 중심, 단위 질량당 관성(점들 범위의 상자 근사), 각속도 감쇠(기본 0.05), 수면(`asleep`), `fixed`(밀리지 않는 장애물), `collide`, `moved`.
- `StepRigidBodies`: 여러 몸체가 서로 충돌. 한 몸체의 접촉 점이 다른 몸체의 볼록 껍질 안에 들어오면 가장 가까운 면으로 밀어낸다 (점-면 접촉). 반발은 평균, 마찰은 작은 값. 잠든 몸체는 장애물처럼 막다가 0.2 m/s 이상 부딪히면 깨어난다.
- 질량은 "단위 질량당"으로만 다룬다 (몸체마다 다른 질량비로 서로 밀고 밀리는 계산이 없다).

**`Core/CCollision`**: `Aabb`, `Obb`(SAT 15축), `Broadphase`(Sweep and Prune), `Bvh`, `MeshRaycast`, `RobotCollision`(링크 자기충돌 AABB → OBB). 모두 겹침 **판정** 위주이고 접촉점/법선은 AABB `PenetrationDepth` 외에는 없다.

**`Core/CDynamics/Dynamics`, `Gravity`**: 로봇 관절 동역학(RNEA, 중력 토크, 질량중심). `JointWrenches` 는 F/T 센서가 쓴다.

**`Core/CSimul/Engine`**: 틱 루프(lockstep), `Backend` 인터페이스, 명령 큐, 이벤트. 움직임은 `move_to`(직선), `move_joint` 뿐이다. `Backend::ReadCollisions` 로 **겹침 쌍**(OBB 까지)만 읽는다. 물리 적분은 없다.

**`Core/CSensor`**: Lidar, Camera, ForceTorque(`FtExternalWrench` 를 호출자가 줌), IMU.

기타: `CProcess`(A→B 직선 이동 물류), `docs/concept_scripting_dll.md`(Python + DLL 컨셉), `api/`(`simul_core` 공유 라이브러리 C API).

## 3. 없는 것 (빈 곳) [확인]
1. 도형 종류: 평면, 구, 박스, 캡슐. (현재는 점 + 볼록 껍질뿐)
2. 몸체별 **질량과 관성 텐서**, 서로 밀고 밀리는 임펄스 (질량비 반영)
3. **접촉 매니폴드** (접촉점 여러 개, 법선, 침투 깊이)와 접촉 지속(warm start). 쌓기 안정성에 필요.
4. **키네마틱 바디** (속도를 외부가 정함): 컨베이어 표면 속도, 로봇 링크
5. **접촉 이벤트/접촉력 공개**: F/T 센서의 `FtExternalWrench` 에 넣을 값, `CSimul` 이벤트
6. `CSimul::Engine` / `Backend` 와의 연결 (지금 물리는 엔진 루프와 별개)
7. 관절/구속 (몸체 사이 힌지 등), 모터 토크 입력 경로 (`concept_scripting_dll.md` 8절의 열린 질문)

## 4. 구성 제안 [제안, 방향은 아직 미정]
`RigidFall` 은 기존 테스트가 있으므로 건드리지 않는 쪽을 기본으로 한다.

| 요소 | 역할 |
|---|---|
| `CPhysicsWorld` | 몸체 등록/삭제, 중력, `Step(dt)`, 접촉 목록과 이벤트 제공 |
| `CBody` | 질량, 관성 텐서, 자세, 선속도/각속도, 종류(동적/정적/키네마틱), 재료(반발, 마찰) |
| `CShape` | 평면, 구, 박스, 캡슐, (이후) 볼록 껍질. 기존 `Obb`/`Aabb` 재사용 |
| 브로드페이즈 | 기존 `FindOverlappingPairs` (Sweep and Prune) 재사용 |
| 내로페이즈 | 도형 쌍별 접촉 매니폴드. 박스-박스는 SAT 로 면/모서리 접촉점 생성 |
| 접촉 해결 | 순차 임펄스 반복 + 마찰 + 반발 + 위치 보정, 수면 |
| 접촉 공개 | 접촉점/법선/임펄스 → F/T 센서 `FtExternalWrench`, 이벤트 |
| Engine 연결 | `Backend` 확장 또는 어댑터: 틱 안에서 `Step` 호출, 로봇 링크는 키네마틱 바디 |

한 틱: 명령 소비 → 외력 → 속도 적분 → 충돌 검출 → 접촉 해결 → 위치 적분 → 접촉 정보 공개. 고정 dt, 같은 입력이면 같은 결과.

## 5. 구현 순서 제안 [제안]
1. `CPhysics` 뼈대: `CBody`, `CShape`(구/박스/평면), 중력 + 적분, 해석해 테스트
2. 접촉: 구-평면, 박스-평면, 구-구, 박스-박스 (매니폴드), 반발/마찰/수면
3. 쌓기 안정성, 경사면 마찰 한계각, 결정성(같은 입력 → 같은 결과) 테스트
4. 키네마틱 바디 (컨베이어 표면 속도, 로봇 링크)
5. 접촉 공개 → F/T 센서 연결
6. `CSimul::Engine` 연결
7. 기존 `RigidFall` 은 유지하거나, 동작이 같음을 확인한 뒤 새 엔진 래퍼로 대체 [열림]

## 6. 테스트 기준 [제안]
- 자유낙하 위치/속도가 해석해와 일치, 반발 계수 0 / 1 에서 정지 / 에너지 보존 경향
- 경사면: tanθ ≈ μ 에서 정지 ↔ 미끄러짐 전환
- 박스 쌓기가 수 초간 유지
- 같은 입력 두 번 실행 시 결과가 같음
- `no_std` 검사와 기존 테스트 전부 통과

## 7. 빌드 메모 [확인]
- 빌드에 LAPACK 이 필요하다 (`liblapack-dev`). tinyxml2 가 없으면 `third_party` 포함본을 쓴다.
- `-DSIMUL_BUILD_QT=OFF` 로 구성하면 `main.cpp` 가 `CoreQt` 를 참조해서 `simul_main` 이 실패한다 (테스트 타깃은 `cmake --build . -- -k` 로 빌드 가능). 그 상태에서 `simul_main` 이 필요한 `sample_*` 6개 테스트는 실행되지 않는다.
- 그 외 기준선 테스트는 모두 통과: test_cdynamics, test_ccollision, test_bvh, test_csensor, test_cforcetorque, test_cimu, test_cworld, test_csimul 등.

## 8. 열린 질문 [열림]
1. 새 `CPhysics` 모듈을 만들지, `RigidFall` 을 확장할지, 대체할지 (현재는 새 모듈 + 기존 유지를 기본으로 가정)
2. 정확도 목표: 게임 수준 안정성 vs 로봇 시뮬레이션 수준 정밀도
3. 고정 dt 값 (`RigidFall` 은 1/480 s 이하 서브스텝, 테스트는 1/240 s)
4. 첫 도형 범위: 구/박스/평면으로 충분한가, 메시(볼록 껍질) 충돌이 필요한가
5. 컨베이어를 표면 속도를 가진 키네마틱 바디로 다룰지
6. 엔진에 모터 토크 입력 경로를 둘지
