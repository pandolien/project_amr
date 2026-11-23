#ifndef CORE_CDYNAMICS_RIGIDFALL_H
#define CORE_CDYNAMICS_RIGIDFALL_H

#include "CBase/Array.h"

namespace simul {
namespace dynamics {

// 바닥(수평면 z = planeZ) 위에 떨어지는 강체 하나의 6 자유도 운동 (회전 포함). FallBody 의 수직 운동에 회전을 더한 것이다.
//  - 모양은 몸체 좌표계(모델 원점 기준)의 접촉 후보 점들로 준다 (SelectContactPoints 로 꼭짓점에서 볼록 껍질 근처의 점만 고른다).
//  - 질량 중심 com 과 관성(단위 질량당, 몸체 좌표축에 나란한 상자 근사)을 쓴다 (SetShape 가 점들의 범위로 채운다).
//  - 한 스텝은 1/480 초 이하로 나눠, 중력 + 자이로 효과로 속도를 갱신하고, 평면 아래로 들어간 점마다
//    충격량(반발 + 쿨롱 마찰)을 순차적으로 풀고, 위치를 평면 위로 밀어 올린다. 점들이 가만히 서면 잠든다(AtRest).
// 좌표는 월드, 단위는 SI (m, rad, s). 회전은 행 우선 3x3 행렬(몸체 -> 월드).
struct RigidFall {
    // 모양 (몸체 좌표)
    simul::Array<double> points;     // x y z 를 이어 붙인 접촉 후보 점
    double com[3];                   // 질량 중심
    double inertia[3];               // 질량 중심 기준 단위 질량당 주관성 모멘트 (몸체 x, y, z 축)
    // 상태
    double pos[3];                   // 몸체 원점의 월드 위치
    double rot[9];                   // 몸체 -> 월드 회전 (행 우선)
    double vel[3];                   // 질량 중심의 속도
    double omega[3];                 // 각속도 (월드)
    // 재료와 설정
    double planeZ;                   // 바닥 높이. 기본 0
    double restitution;              // 0 = 튕기지 않음, 1 = 완전 탄성. 기본 0.3
    double friction;                 // 쿨롱 마찰 계수. 기본 0.6
    double restSpeed;                // 닿을 때 이 속도(m/s) 미만이면 튕기지 않는다. 기본 0.4 (FallBody 와 같다)
    double angularDamping;           // 공기 저항 같은 각속도 감쇠 (1/s). 기본 0.05
    bool asleep;                     // 멈춰 서 있다

    RigidFall();

    // 점들을 정하고 질량 중심(점들 범위의 중심)과 관성(그 범위의 균일한 상자)을 채운다. 상태는 건드리지 않는다.
    void SetShape(const simul::Array<double>& bodyPoints);
    // 몸체 원점 위치와 회전을 정하고 속도를 0 으로 하고 깨운다.
    void SetPose(const double position[3], const double rotation[9]);
    void SetVelocity(const double linear[3], const double angular[3]);   // 질량 중심 속도와 각속도 (월드)

    // gz: 중력 가속도의 수직 성분 (m/s^2, 아래로 떨어지면 음수. 0 이상이면 떨어지지 않는다). 아직 움직이는 중이면 true.
    // dt 는 0 보다 커야 하고, 한 번에 너무 길면(> 0.1 s) 0.1 s 로 자른다.
    bool Step(double dt, double gz);
    bool AtRest() const { return asleep; }

    double LowestPointZ() const;                 // 점들 중 가장 낮은 월드 z
    int ContactCount(double tol) const;          // 바닥에서 tol 이내인 점의 수
    void WorldCom(double out[3]) const;          // 질량 중심의 월드 위치
    double KineticEnergy() const;                // 단위 질량당 운동 에너지

private:
    int stillSteps_;
};

// 꼭짓점(x y z, n 개)에서 접촉 후보 점을 고른다: 여러 방향(약 76 개)으로 가장 멀리 있는 꼭짓점들 (겹치는 것은 하나로).
// 어떤 모양이든 아래로 떨어졌을 때 닿을 수 있는 껍질 근처의 점은 모두 남는다. 점이 76 개를 넘지 않는다.
void SelectContactPoints(const double* xyz, size_t count, simul::Array<double>* out);

}  // namespace dynamics
}  // namespace simul

#endif  // CORE_CDYNAMICS_RIGIDFALL_H
