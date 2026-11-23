#ifndef CORE_CSENSOR_IMU_H
#define CORE_CSENSOR_IMU_H

#include "3d/dhMat.h"
#include "CBase/Array.h"
#include "CBase/String.h"
#include "CDynamics/Dynamics.h"
#include "CDynamics/Gravity.h"
#include "CURDF/Urdf.h"

// IMU(관성 측정 장치) 시뮬레이션: 3축 가속도계 + 3축 자이로 + 자세. AMR 본체나 로봇암 링크에 단다.
//
// 가속도계는 "비력" (a - g) 을 잰다. 정지해 있으면 중력의 반대 방향, 곧 센서가 수평이면 +z 로 +9.80665 m/s^2 가 읽힌다
// (자유낙하하면 0). 자이로는 각속도, 자세는 센서 좌표계가 루트(월드) 좌표계에서 얼마나 돌아 있는지다.
// 센서가 링크 원점에서 떨어져 있으면 그 점의 가속도(각가속도, 구심 가속도 포함)를 잰다.
// 좌표 규약은 라이다/카메라와 같다 (<pose> 로 링크 기준 자세를 준다). 단위는 m/s^2, rad/s, 쿼터니언은 (w, x, y, z).
//
// AMR 이 달리는 중이면 RootMotion (루트 링크의 각속도, 각가속도, 선가속도) 을 같이 넘긴다. 암 링크에 단 IMU 는 암의 움직임과 AMR 의 움직임을 모두 읽는다.
namespace simul {
namespace sensor {

struct ImuSettings {
    simul::String link;          // 센서가 붙는 링크
    dhMat pose;                  // 링크 좌표계 -> 센서 좌표계
    float accelNoiseStdDev;      // 가속도 잡음 표준편차 (m/s^2). 0 이면 없음
    float gyroNoiseStdDev;       // 각속도 잡음 표준편차 (rad/s)
    float orientationNoiseStdDev;  // 자세 잡음 (rad, 세 축 회전벡터의 표준편차)
    float accelBias[3];          // 가속도 편향 (m/s^2, 센서 좌표계)
    float gyroBias[3];           // 자이로 편향 (rad/s)
    float accelRange;            // 가속도 한 축의 측정 범위 +-accelRange (m/s^2). 0 이면 제한 없음
    float gyroRange;             // 자이로 한 축의 측정 범위 (rad/s)
    unsigned seed;               // 잡음 난수 시드
    ImuSettings() : accelNoiseStdDev(0), gyroNoiseStdDev(0), orientationNoiseStdDev(0), accelRange(0), gyroRange(0), seed(1u) {
        for (int i = 0; i < 3; i++) accelBias[i] = gyroBias[i] = 0.0f;
    }
};

struct ImuReading {
    double accel[3];        // m/s^2, 센서 좌표계 (비력)
    double gyro[3];         // rad/s, 센서 좌표계
    double orientation[4];  // 센서 좌표계 -> 루트 좌표계 회전 (w, x, y, z)
    bool saturated;         // 가속도 또는 자이로가 측정 범위에 걸렸는가
    bool valid;             // false 면 링크를 못 찾은 것
    ImuReading() : saturated(false), valid(false) {
        accel[0] = accel[1] = accel[2] = gyro[0] = gyro[1] = gyro[2] = 0;
        orientation[0] = 1;
        orientation[1] = orientation[2] = orientation[3] = 0;
    }
};

class Imu {
public:
    explicit Imu(const ImuSettings& settings = ImuSettings());
    const ImuSettings& Settings() const { return settings_; }
    void SetSettings(const ImuSettings& s);  // 난수도 새 시드로 다시 시작한다

    // q: 관절 값, qd/qdd: 구동 관절 속도/가속도 (없는 관절은 0), g: 루트 좌표계 중력, root: 루트 링크의 운동 (기본 정지)
    ImuReading Measure(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::JointTorques& qd = dynamics::JointTorques(),
                       const dynamics::JointTorques& qdd = dynamics::JointTorques(), const dynamics::Gravity& g = dynamics::Gravity(),
                       const dynamics::RootMotion& root = dynamics::RootMotion());

private:
    float gaussian();
    unsigned nextRandom();
    ImuSettings settings_;
    unsigned rng_;
};

// URDF 의 <gazebo reference="링크"><sensor type="imu"> 정의 -> 설정
ImuSettings ImuSettingsFromDef(const urdf::ImuDef& def);

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_IMU_H
