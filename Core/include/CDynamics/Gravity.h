#ifndef CORE_CDYNAMICS_GRAVITY_H
#define CORE_CDYNAMICS_GRAVITY_H

#include <map>
#include <string>

#include "CURDF/Urdf.h"

// 중력 관련 동역학. 계산은 모두 double 이고, 단위는 SI (m, kg, N, N·m, rad) 이다.
//
// 좌표계: 로봇의 루트 링크 좌표계. 중력 벡터도 이 좌표계로 준다.
//   AMR 이 기울어져 있으면 중력 방향이 루트 링크 기준으로 달라지므로, IMU 등으로 구한 중력 방향을
//   루트 링크 좌표계로 바꿔서 넘기면 된다 (기본값은 아래쪽 -Z).
namespace amr {
namespace dynamics {

const double kStandardGravity = 9.80665;  // m/s^2

// 중력 가속도 벡터 (m/s^2). 방향이 "떨어지는 방향"이다. 기본은 (0, 0, -9.80665).
struct Gravity {
    double x, y, z;
    Gravity() : x(0), y(0), z(-kStandardGravity) {}
    Gravity(double gx, double gy, double gz) : x(gx), y(gy), z(gz) {}
};

typedef std::map<std::string, double> JointTorques;  // 관절 이름 -> 토크(회전 관절 N·m) 또는 힘(이동 관절 N)

// 정지 상태(속도와 가속도가 0)에서 각 관절이 중력을 버티려고 내야 하는 토크.
//   운동방정식 M(q) q'' + C(q, q') + g(q) = tau 의 g(q) 이다. 즉 g(q) = dU/dq (U 는 중력 위치에너지)이고,
//   부호는 관절 좌표(axis 방향 회전, 이동)의 + 방향 기준이다.
//   예: 수평으로 뻗은 팔이 +y 축 관절에 달려 있으면 팔이 아래로 처지려는 방향이 +y 회전이므로 토크는 음수다.
// 결과에는 구동 관절(Robot::ActuatedJoints, 즉 움직이는 관절 중 mimic 이 아닌 것)만 들어간다.
//   mimic 관절의 토크는 가상일의 원리로 "원본" 관절의 토크에 multiplier 배로 합쳐진다.
// 질량이 없는(<inertial> 이 없거나 mass 0) 링크는 무시한다. 관절 값은 clampToLimits 이면 limit 으로 제한한다.
JointTorques GravityTorques(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g = Gravity(),
                            bool clampToLimits = true);

// 로봇 전체의 질량중심(루트 링크 좌표계)과 총 질량. 질량이 있는 링크가 없으면 false.
// AMR 위에 팔을 올렸을 때 전체 무게중심이 어디로 가는지 볼 때 쓴다.
bool CenterOfMass(const urdf::Robot& robot, const urdf::JointValues& q, double outCom[3], double* outMass,
                  bool clampToLimits = true);

}  // namespace dynamics
}  // namespace amr

#endif  // CORE_CDYNAMICS_GRAVITY_H
