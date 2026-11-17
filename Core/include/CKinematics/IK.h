#ifndef CORE_CKINEMATICS_IK_H
#define CORE_CKINEMATICS_IK_H

#include "3d/dhMat.h"
#include "CBase/Array.h"
#include "CBase/String.h"
#include "CURDF/Urdf.h"

// 범용 수치 역기구학 (URDF 만 있으면 어떤 로봇이든 쓴다). 링크 하나의 월드(루트 링크 좌표계) 자세를 목표로 구동 관절
// 값을 구한다. 자코비안을 관절 축/원점에서 만들고, 감쇠 최소제곱(Levenberg-Marquardt):  dq = J^T (J J^T + lambda^2 I)^-1 e.
// 특이점 근처에서도 안정적이고, 중복 자유도(7축 등)는 현재 값에서 가장 적게 움직이는 해를 고른다.
// 한계:
//  - 일직선으로 펴진 자세 같은 특이 자세 근처에서는 국소 최솟값에 멈출 수 있다 (restarts 로 완화).
//  - 해는 하나로 정해지지 않는다. 시작 관절 값(start)에 가까운 해로 수렴한다.
//  - 로봇 전용 해석적 IK 보다 느리다 (반복 계산). 뷰어에서 끌기, 경로 계획용으로는 충분하다.
//  - 관절 한계는 지킨다 (한계에 닿은 관절은 고정하고 나머지로 다시 푼다). 한계 때문에 못 닿으면 converged = false 이고
//    가장 가까운 값을 돌려준다.
// 단위: m, rad (관절 값도 내부 단위). mimic 관절은 마스터에 합쳐서 푼다.
namespace simul {
namespace kinematics {

struct IkOptions {
    bool useOrientation;          // false 이면 위치(3)만, true 이면 위치 + 자세(6)
    int maxIterations;            // 반복 상한
    double positionTolerance;     // m
    double orientationTolerance;  // rad (useOrientation 일 때만)
    double orientationWeight;     // 자세 오차를 위치 오차(m)와 견주는 가중치 (기본 1: 1 rad = 1 m)
    double initialDamping;        // lambda 초기값. 반복 중 적응적으로 바뀐다.
    double maxStep;               // 한 번에 바꿀 수 있는 관절 변화의 최댓값 (rad 또는 m)
    bool respectLimits;           // 관절 한계를 지킨다
    int restarts;                 // 수렴하지 못하면 한계 안의 무작위 값에서 다시 시도할 횟수 (기본 0). 시드가 고정이라 결과는 매번 같다.
                                  // 끌기처럼 연속성이 필요한 곳에는 쓰지 말고, 한 번에 해를 찾아야 할 때 쓴다.
    IkOptions()
        : useOrientation(false), maxIterations(200), positionTolerance(1e-4), orientationTolerance(1e-3), orientationWeight(1.0),
          initialDamping(0.05), maxStep(0.3), respectLimits(true), restarts(0) {}
};

struct IkResult {
    bool converged;
    int iterations;
    double positionError;     // 최종 위치 오차 (m)
    double orientationError;  // 최종 자세 오차 (rad). useOrientation 이 아니면 0
    urdf::JointValues q;      // 결과 관절 값 (모든 구동 관절 + start 에 있던 값)
    IkResult() : converged(false), iterations(0), positionError(0), orientationError(0) {}
};

// link: 목표를 맞출 링크의 인덱스. target: 그 링크 좌표계가 가야 할 루트 기준 자세(dhMat, 이동 + 회전).
// 목표 링크가 움직이는 관절의 자식(후손)이 아니면 (움직일 수 없으면) converged = false.
IkResult SolveIk(const urdf::Robot& robot, const urdf::JointValues& start, int link, const dhMat& target,
                 const IkOptions& options = IkOptions());

// 링크의 현재 자세 (루트 기준)와 목표 사이의 오차. 위치 오차(m)와 자세 오차(rad, 회전 벡터의 크기).
void PoseError(const urdf::Robot& robot, const urdf::JointValues& q, int link, const dhMat& target, double* positionError,
               double* orientationError);

}  // namespace kinematics
}  // namespace simul

#endif  // CORE_CKINEMATICS_IK_H
