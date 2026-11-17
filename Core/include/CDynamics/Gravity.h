#ifndef CORE_CDYNAMICS_GRAVITY_H
#define CORE_CDYNAMICS_GRAVITY_H

#include "CBase/Map.h"
#include "CBase/String.h"
#include "CBase/Array.h"

#include "CURDF/Urdf.h"

// 중력 관련 동역학. 계산은 모두 double 이고, 단위는 SI (m, kg, N, N·m, rad) 이다.
//
// 좌표계: 로봇의 루트 링크 좌표계. 중력 벡터도 이 좌표계로 준다.
//   AMR 이 기울어져 있으면 중력 방향이 루트 링크 기준으로 달라지므로, IMU 등으로 구한 중력 방향을
//   루트 링크 좌표계로 바꿔서 넘기면 된다 (기본값은 아래쪽 -Z).
namespace simul {
namespace dynamics {

const double kStandardGravity = 9.80665;  // m/s^2

// 중력 가속도 벡터 (m/s^2). 방향이 "떨어지는 방향"이다. 기본은 (0, 0, -9.80665).
struct Gravity {
    double x, y, z;
    Gravity() : x(0), y(0), z(-kStandardGravity) {}
    Gravity(double gx, double gy, double gz) : x(gx), y(gy), z(gz) {}
};

typedef simul::Map<simul::String, double> JointTorques;  // 관절 이름 -> 토크(회전 관절 N·m) 또는 힘(이동 관절 N)

// 정지 상태(속도와 가속도가 0)에서 각 관절이 중력을 버티려고 내야 하는 토크.
//   운동방정식 M(q) q'' + C(q, q') + g(q) = tau 의 g(q) 이다. 즉 g(q) = dU/dq (U 는 중력 위치에너지)이고,
//   부호는 관절 좌표(axis 방향 회전, 이동)의 + 방향 기준이다.
//   예: 수평으로 뻗은 팔이 +y 축 관절에 달려 있으면 팔이 아래로 처지려는 방향이 +y 회전이므로 토크는 음수다.
// 결과에는 구동 관절(Robot::ActuatedJoints, 즉 움직이는 관절 중 mimic 이 아닌 것)만 들어간다.
//   mimic 관절의 토크는 가상일의 원리로 "원본" 관절의 토크에 multiplier 배로 합쳐진다.
// 질량이 없는(<inertial> 이 없거나 mass 0) 링크는 무시한다. 관절 값은 clampToLimits 이면 limit 으로 제한한다.
JointTorques GravityTorques(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g = Gravity(),
                            bool clampToLimits = true);

// ---- 관절마다 따로 보기
// GravityTorques 는 모터가 내야 할 토크(구동 관절만, mimic 은 원본에 합산)이고, 아래는 "모든 관절이 각자 받는 하중"이다.
struct JointLoad {
    simul::String name;
    bool movable;  // 회전/이동 관절인가 (fixed 등은 false)
    bool mimic;    // mimic 관절인가
    // 움직이는 관절만 의미 있음: 자기 좌표(axis 방향 회전, 이동)의 + 방향 기준 토크(N·m) 또는 힘(N).
    // mimic 관절도 자기 몫만 담고 원본에 합치지 않는다. fixed 등은 0.
    double torque;
    // 이 관절이 자기 아래쪽(자식 쪽) 링크 전체를 버티려고 전달하는 하중. 루트 링크 좌표계.
    //   force  = -M g                        (N, 아래쪽 전체 질량 M)
    //   moment = -sum (p_i - o) x (m_i g)    (N·m, 관절 원점 o 기준)
    // fixed 관절의 용접/체결부가 받는 하중이 이 값이다. 움직이는 관절의 torque 는 이 값을 축에 투영한 것과 같다
    // (회전: axis . moment, 이동: axis . force).
    double force[3];
    double moment[3];
    JointLoad() : movable(false), mimic(false), torque(0) {
        force[0] = force[1] = force[2] = 0;
        moment[0] = moment[1] = moment[2] = 0;
    }
};

// robot.joints 와 같은 순서로 모든 관절(fixed, mimic 포함)의 하중을 돌려준다.
simul::Array<JointLoad> GravityLoads(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g = Gravity(),
                                    bool clampToLimits = true);

// 한 관절의 torque 에 링크마다 얼마씩 기여했는지. 그 관절 아래쪽(자식 쪽)에 있는 질량 있는 링크만 나오고,
// value 의 합이 GravityLoads 의 그 관절 torque 와 같다. 관절이 없거나 움직이지 않는 관절이면 빈 목록.
struct LinkContribution {
    simul::String link;
    double value;  // N·m 또는 N
    LinkContribution() : value(0) {}
};
simul::Array<LinkContribution> GravityContributions(const urdf::Robot& robot, const urdf::JointValues& q,
                                                   const simul::String& jointName, const Gravity& g = Gravity(),
                                                   bool clampToLimits = true);

// 로봇 전체의 질량중심(루트 링크 좌표계)과 총 질량. 질량이 있는 링크가 없으면 false.
// AMR 위에 팔을 올렸을 때 전체 무게중심이 어디로 가는지 볼 때 쓴다.
bool CenterOfMass(const urdf::Robot& robot, const urdf::JointValues& q, double outCom[3], double* outMass,
                  bool clampToLimits = true);

}  // namespace dynamics
}  // namespace simul

#endif  // CORE_CDYNAMICS_GRAVITY_H
