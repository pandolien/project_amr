#ifndef CORE_CSENSOR_FORCETORQUE_H
#define CORE_CSENSOR_FORCETORQUE_H

#include "3d/dhMat.h"
#include "CBase/Array.h"
#include "CBase/String.h"
#include "CDynamics/Dynamics.h"
#include "CDynamics/Gravity.h"
#include "CURDF/Urdf.h"

// 6축 힘/토크(F/T) 센서 시뮬레이션. 로봇암 손목(엔드이펙터 앞)이나 AMR 과 암의 체결부에 달아
// 접촉력, 파지력, 매단 물건의 무게를 잰다.
//
// 원리: 센서는 관절 하나에 붙는다 (보통 fixed 관절). 센서가 읽는 값은 그 관절이 자식 쪽(자식 링크와 그 아래 전체)을
// 붙들려고 전달하는 하중이다. 정지 상태면 아래쪽 전체 무게를 받치는 힘이고, 움직이면 관성력(가속도, 코리올리, 원심력)이
// 더해진다. 아래쪽에 외력(접촉 등)이 작용하면 그만큼 전달하는 하중이 달라진다.
//     측정값 = (아래쪽 링크의 관성력 + 중력 반력) - (아래쪽에 작용하는 외력)
// 계산은 CDynamics 의 재귀 뉴턴-오일러(JointWrenches)를 쓰고, 그 뒤에 센서 좌표계로 옮기고 편향, 잡음, 포화를 적용한다.
// 단위는 SI (N, N·m). 좌표 규약은 라이다/카메라와 같다 (<pose> 로 링크 기준 자세를 준다).
namespace simul {
namespace sensor {

// 측정 부호. Gazebo 의 <measure_direction> 과 같은 뜻이다.
enum FtDirection {
    kFtParentToChild,  // 부모가 자식에게 가하는 하중 (기본. 정지한 센서는 +z 로 무게를 받친다)
    kFtChildToParent   // 자식이 부모에게 가하는 하중 (위의 부호를 뒤집은 것. 센서 몸체가 받는 하중)
};

struct FtSettings {
    simul::String joint;       // 센서가 붙는 관절 이름 (보통 fixed 관절)
    dhMat pose;                // 자식 링크 좌표계 -> 센서 좌표계 (측정값을 이 좌표계로 표현하고 이 점 기준으로 모멘트를 구한다)
    FtDirection direction;
    float forceNoiseStdDev;    // 힘 잡음 표준편차 (N). 0 이면 없음
    float torqueNoiseStdDev;   // 토크 잡음 표준편차 (N·m)
    float forceBias[3];        // 힘 편향 (N, 센서 좌표계). 영점이 어긋난 센서
    float torqueBias[3];       // 토크 편향 (N·m)
    float forceRange;          // 힘 한 축의 측정 범위 +-forceRange (N). 0 이면 제한 없음. 넘으면 포화된다
    float torqueRange;         // 토크 한 축의 측정 범위 (N·m)
    unsigned seed;             // 잡음 난수 시드 (같으면 같은 잡음)
    FtSettings()
        : direction(kFtParentToChild), forceNoiseStdDev(0.0f), torqueNoiseStdDev(0.0f), forceRange(0.0f), torqueRange(0.0f), seed(1u) {
        forceBias[0] = forceBias[1] = forceBias[2] = 0.0f;
        torqueBias[0] = torqueBias[1] = torqueBias[2] = 0.0f;
    }
};

// 센서 아래쪽 링크에 작용하는 외력 하나 (접촉력, 밀기 등). 점 point 에 힘 force 와 순수 토크 torque 가 걸린다.
// 모두 루트 링크 좌표계. 센서 위쪽(부모 쪽) 링크에 걸린 외력은 센서가 못 느끼므로 무시한다.
struct FtExternalWrench {
    simul::String link;        // 외력이 걸린 링크
    double point[3];           // 작용점 (루트 좌표계)
    double force[3];           // N
    double torque[3];          // N·m (작용점에 걸리는 순수 토크. 보통 0)
    FtExternalWrench() {
        point[0] = point[1] = point[2] = 0;
        force[0] = force[1] = force[2] = 0;
        torque[0] = torque[1] = torque[2] = 0;
    }
};

struct FtReading {
    double force[3];           // N, 센서 좌표계
    double torque[3];          // N·m, 센서 좌표계 (센서 원점 기준)
    bool saturated;            // 어느 축이든 측정 범위에 걸렸는가
    bool valid;                // false 면 관절을 못 찾은 것 (값은 0)
    FtReading() : saturated(false), valid(false) {
        force[0] = force[1] = force[2] = 0;
        torque[0] = torque[1] = torque[2] = 0;
    }
    double ForceNorm() const;
    double TorqueNorm() const;
};

class ForceTorque {
public:
    explicit ForceTorque(const FtSettings& settings = FtSettings());
    const FtSettings& Settings() const { return settings_; }
    void SetSettings(const FtSettings& s);  // 난수도 새 시드로 다시 시작한다

    // q: 관절 값, qd/qdd: 구동 관절 속도/가속도 (없는 관절은 0). 중력 g 는 루트 좌표계 (AMR 이 기울어 있으면 그 방향으로).
    // external 은 센서 아래쪽에 작용하는 외력들 (없으면 NULL, 0).
    FtReading Measure(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::JointTorques& qd, const dynamics::JointTorques& qdd,
                      const dynamics::Gravity& g = dynamics::Gravity(), const FtExternalWrench* external = 0, size_t externalCount = 0);

    // 정지 상태 측정 (qd = qdd = 0)
    FtReading MeasureStatic(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::Gravity& g = dynamics::Gravity(),
                            const FtExternalWrench* external = 0, size_t externalCount = 0);

private:
    float gaussian();
    unsigned nextRandom();
    FtSettings settings_;
    unsigned rng_;
};

// URDF 의 <gazebo reference="관절"><sensor type="force_torque"> 정의 -> 설정
FtSettings FtSettingsFromDef(const urdf::FtDef& def);

// 링크 link 가 관절 joint 의 자식 쪽(자식 링크 자신 또는 그 자손)인가
bool IsBelowJoint(const urdf::Robot& robot, const simul::String& joint, const simul::String& link);

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_FORCETORQUE_H
