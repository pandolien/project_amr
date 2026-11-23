# 강화학습 학습 가이드 (feature/reinforcement-learning)

공부용으로 만든 브랜치다. 우리 시뮬레이터(`CSimul::Engine` + `HeadlessScene`) 위에서 로봇 팔이 **시행착오로** 목표에
도달하는 법을 배우게 하고, 그 과정에서 강화학습의 기본 개념 두 가지(가치 기반 / 정책 기반)를 코드로 읽을 수 있게 했다.
`main` 에는 들어가지 않는다.

## 1. 한 장 요약

```
            행동 a (관절을 한 칸 움직여라)
  에이전트 ───────────────────────────────▶ 환경 (ArmReachEnv)
  (QLearning /                                  │  Engine.MoveJoint + Step(dt)  ← 시뮬레이터
   Reinforce)  ◀───────────────────────────────┘  순기구학으로 팔 끝 위치 계산
            상태 s', 보상 r, 끝났는가
```

- **상태 s**: 두 관절 각도를 칸으로 나눈 번호 (기본 13 x 17 = 221개)
- **행동 a**: 4개 (관절 A +/-, 관절 B +/- 한 칸)
- **보상 r**: 매 스텝 `-0.05 - (팔 끝과 목표 사이 거리 m)`, 목표에 닿으면 `+10` 하고 끝
- **목표**: 학습으로 "각 상태에서 어떤 행동을 할지" (정책)를 얻는 것. 보상 합이 커지는 쪽으로 정책이 좋아진다.

## 2. 실행해 보기

```bash
cmake -S . -B build -DSIMUL_BUILD_QT=OFF -DSIMUL_BUILD_TESTS=ON
cmake --build build --target rl_arm_reach test_crl
./build/bin/rl_arm_reach                      # Q-러닝
./build/bin/rl_arm_reach --algo reinforce     # 정책 기울기
./build/bin/rl_arm_reach --episodes 300       # 덜 학습시키면 정책이 어떻게 달라지나?
ctest --test-dir build -R test_crl
```

출력의 **학습 곡선**(성공률이 올라가는지), **정책 지도**(각 칸에서 고른 행동 화살표가 `G` 로 모이는지)를 본다.

## 3. 코드 읽는 순서

| 순서 | 파일 | 볼 것 |
|---|---|---|
| 1 | `Core/include/CRL/Env.h` | `Env`(reset/step), `StepResult`(보상, terminal, truncated), `Policy` |
| 2 | `Core/src/CRL/ArmReachEnv.cpp` | 시뮬레이터에 명령(`MoveJoint`)을 내리고 상태를 읽는 부분. `Step()` 의 보상 계산 |
| 3 | `Core/src/CRL/QLearning.cpp` | `Update()` 한 줄이 Q-러닝의 전부 |
| 4 | `Core/src/CRL/Reinforce.cpp` | `UpdateEpisode()`: 누적 보상 G, 기준선, 확률 갱신 |
| 5 | `tests/test_crl.cpp` | 각 부분이 맞게 동작한다는 확인 (한 칸 이동, 경계, truncated, 재현성) |
| 6 | `examples/rl_arm_reach.cpp` | 학습 루프를 돌리고 결과를 출력 |

모두 C++11, 표준 라이브러리 없음(`CBase::Array` 사용), `Core` 규칙 그대로다.

## 4. 핵심 개념과 코드 위치

**에피소드와 보상 합.** 에이전트는 한 번 시작해서 끝날 때까지가 한 에피소드. 목표는 순간 보상이 아니라
`G = r0 + γ r1 + γ² r2 + ...` (할인된 합)을 키우는 것이다. `γ`(gamma)가 작으면 눈앞만 보고, 1에 가까우면 먼 미래를 본다.

**Q-러닝 (가치 기반).** `Q(s,a)` 표를 학습한다. 갱신식은 `QLearning::Update`:
`Q += α (r + γ max Q(s') - Q)`. "내 예측"을 "실제 보상 + 다음 상태의 예측"으로 조금씩 맞춘다. 행동은 `ε`-탐욕:
확률 ε 으로 무작위(탐험), 아니면 Q 최대(활용). ε 은 에피소드가 지날수록 줄인다.

**REINFORCE (정책 기반).** 행동의 **확률**을 직접 학습한다(softmax). 한 에피소드가 끝난 뒤, 그 에피소드에서 한
행동이 "평균보다 좋았으면" 확률을 올리고 나빴으면 내린다 (`advantage = G - 기준선`). Q 표 없이 정책만 있다.

**terminal 과 truncated.** 목표 도달(terminal)이면 "그 뒤 미래 보상은 0" 이지만, 시간 제한(truncated)은 그냥
멈춘 것이라 미래가 있다. 그래서 Q-러닝은 terminal 일 때만 다음 상태 항을 뺀다 (`TrainQLearning`).

**탐험의 필요.** 보상이 모두 음수라 Q 초기값 0 은 "낙관적"이다. 안 가 본 행동이 좋아 보여서 자연히 이곳저곳 시도한다.

## 5. 실험해 볼 것 (공부 과제)

1. `alpha`, `gamma`, `epsilonDecay` 를 바꿔 학습 곡선이 어떻게 변하는지 본다 (`QLearningConfig`).
2. `ArmReachConfig::distanceWeight` 를 0 으로 하면 (거리 힌트 없음, 목표 도달 보너스만) 학습이 얼마나 느려지나? (보상 설계)
3. `collisionPenalty` 를 키우고, 목표를 팔이 접히는 쪽 칸으로 옮겨서 자기충돌을 피하는 경로를 배우는지 본다.
4. `binsA`, `binsB` 를 키우면 상태 수가 늘어 학습이 느려진다 (차원의 저주). 이 한계가 함수 근사(신경망)가 필요한 이유다.
5. Q-러닝과 REINFORCE 를 같은 seed 로 비교: 어느 쪽이 먼저 100%에 가까워지나? 결과가 seed 마다 얼마나 흔들리나?
6. 참고: 팔 끝이 목표 지점 0.05 m 안에 드는 관절 조합이 목표 칸 말고 하나 더 있다 (같은 점에 닿는 팔꿈치 올림/내림 두 자세). 그 칸에 닿으면 이 환경은 성공으로 끝낸다. 목표를 '칸'이 아니라 '위치'로 정의했기 때문이다.

## 6. 한계와 다음 단계

- 목표가 고정이다. 표 방식은 상태 번호만 보므로 목표가 바뀌면 다시 학습해야 한다. 목표를 입력으로 넣으려면
  연속 관측 + 신경망이 필요하다 (다음 단계: DQN, 또는 정책 기울기 + 신경망).
- 관절 각도를 칸으로 쪼개 한 칸씩 움직인다. 연속 행동(토크/속도)은 다루지 않는다. 엔진에 모터 토크 입력 경로가
  생기면(`docs/physics_engine.md`, `docs/concept_scripting_dll.md` 열린 질문) 동역학 기반 환경으로 확장할 수 있다.
- 물리 적분은 아직 없다 (엔진은 `move_joint` 를 속도로 따라간다). 물리엔진이 들어오면 `ArmReachEnv::Step` 의
  "명령 -> 틱" 부분만 바뀌고 학습 코드는 그대로 쓴다. 이것이 `Env` 인터페이스로 나눈 이유다.
- 학습은 오프라인(로봇 없이 시뮬레이터)에서만 한다. 시뮬레이터와 실제 로봇의 차이(sim-to-real)는 다루지 않는다.
