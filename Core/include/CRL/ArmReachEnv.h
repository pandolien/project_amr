// 시뮬레이션 기반 환경: 로봇 팔 끝(tool)을 목표 지점에 보낸다.
//
//   - 로봇은 CSimul 의 HeadlessScene(Backend) 위에서 Engine 이 움직인다 (move_joint 명령 + 틱).
//     즉 학습 에이전트가 "시뮬레이터에 명령을 내리고 결과 상태를 읽는" 구조다.
//   - 상태: 관절 A, B 각도를 각각 binsA, binsB 칸으로 나눈 칸 번호 (상태 수 = binsA * binsB)
//   - 행동 4가지: A 관절 +한 칸, A -한 칸, B +한 칸, B -한 칸
//   - 보상: 매 스텝 -stepCost - distanceWeight * (팔 끝과 목표 사이 거리 m), 목표 근처면 +goalBonus 하고 종료.
//           collisionPenalty > 0 이면 팔 링크끼리 겹칠 때(자기충돌) 그만큼 깎는다.
//   - 목표: goalA, goalB 칸에 해당하는 자세에서 팔 끝이 있는 위치 (순기구학으로 구한다)
//   - 시간 제한 maxSteps 를 넘으면 truncated
// 관절 각도는 칸의 가운데에서 시작하고 한 번에 정확히 한 칸 움직이므로, 칸 번호가 결정적으로 바뀐다.
#ifndef CORE_CRL_ARMREACHENV_H
#define CORE_CRL_ARMREACHENV_H

#include "3d/dhVector.h"
#include "CBase/String.h"
#include "CRL/Env.h"
#include "CSimul/Engine.h"
#include "CSimul/Headless.h"
#include "CURDF/Urdf.h"

namespace simul {
namespace rl {

using sim::HeadlessScene;
using sim::ObjectInfo;

struct ArmReachConfig {
    String jointA, jointB, endLink;
    int binsA, binsB;
    int goalA, goalB;
    int maxSteps;
    double goalTolerance;    // m
    double dt;               // 시뮬레이션 틱 (s)
    double jointSpeed;       // 관절 이동 속도 (rad/s)
    double stepCost, distanceWeight, goalBonus, collisionPenalty;
    ArmReachConfig()
        : jointA("shoulder_pitch"), jointB("elbow_pitch"), endLink("tool"),
          binsA(13), binsB(17), goalA(10), goalB(4), maxSteps(60),
          goalTolerance(0.05), dt(0.05), jointSpeed(2.0),
          stepCost(0.05), distanceWeight(1.0), goalBonus(10.0), collisionPenalty(0.0) {}
};

class ArmReachEnv : public Env {
public:
    ArmReachEnv();
    ArmReachEnv(const ArmReachEnv&) = delete;
    ArmReachEnv& operator=(const ArmReachEnv&) = delete;

    // urdfPath 의 로봇(예: Data/Object/samples/arm.urdf)을 올린다. 실패하면 false 와 이유.
    bool Init(const String& urdfPath, const ArmReachConfig& config, String* error);

    int NumStates() const override { return cfg_.binsA * cfg_.binsB; }
    int NumActions() const override { return 4; }
    int Reset(Rng* rng) override;
    int Step(int action, StepResult* result) override;
    bool Succeeded() const override { return success_; }

    const ArmReachConfig& Config() const { return cfg_; }
    int StateOf(int binA, int binB) const { return binA * cfg_.binsB + binB; }
    int GoalState() const { return StateOf(cfg_.goalA, cfg_.goalB); }
    int BinA(int state) const { return state / cfg_.binsB; }
    int BinB(int state) const { return state % cfg_.binsB; }
    double BinCenterA(int bin) const { return loA_ + (bin + 0.5) * widthA_; }
    double BinCenterB(int bin) const { return loB_ + (bin + 0.5) * widthB_; }

    dhVector EndEffector() const { return ee_; }      // 지금 팔 끝 위치 (m)
    dhVector GoalPosition() const { return goal_; }
    double DistanceToGoal() const { return dist_; }
    // 지금 자기충돌 중인 링크 쌍의 수 (시뮬레이터의 충돌 검사)
    int SelfCollisions();

private:
    int readState();                                              // 시뮬레이터에서 읽어 ee_, dist_ 를 갱신하고 상태 번호를 돌려준다
    dhVector endEffectorFrom(const ObjectInfo& info) const;       // 관절 값으로 순기구학

    ArmReachConfig cfg_;
    HeadlessScene scene_;
    sim::Engine engine_;
    sim::Handle handle_;
    urdf::Robot robot_;
    int endLinkIndex_;
    double loA_, loB_, widthA_, widthB_;
    dhVector goal_, ee_;
    double dist_;
    int steps_;
    bool success_;
    bool ready_;
};

}  // namespace rl
}  // namespace simul

#endif  // CORE_CRL_ARMREACHENV_H
