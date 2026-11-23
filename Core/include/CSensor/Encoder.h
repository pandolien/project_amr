#ifndef CORE_CSENSOR_ENCODER_H
#define CORE_CSENSOR_ENCODER_H

#include <stdint.h>

#include "CBase/Array.h"
#include "CBase/String.h"
#include "CURDF/Urdf.h"

// 엔코더 시뮬레이션: 관절 하나의 회전/이동을 정수 카운트(틱)로 센다. 구동륜(휠 엔코더)과 로봇암 관절(관절 엔코더)에 같이 쓴다.
//
// 엔코더는 보통 모터 축에 달리므로 관절 1회전에 motor 가 gearRatio 번 돈다 (감속기 감속비).
//   관절 1회전(이동 관절은 1 m)당 카운트 = resolution * gearRatio
// 카운트는 내림(floor)으로 양자화한다. 증분형(기본)은 첫 측정(또는 Reset)한 자리를 0 으로 세고, absolute 이면 관절 0 자리가 0 이다.
// 관절 값(float)에서 계산하므로 아주 오래 도는 바퀴는 float 정밀도(1000 rad 에서 약 6e-5 rad)가 한계다.
// 각도는 rad, 이동은 m, 속도는 rad/s 또는 m/s.
namespace simul {
namespace sensor {

struct EncoderSettings {
    simul::String joint;       // 센서가 달린 관절 이름
    double resolution;         // 엔코더 축 1회전당 카운트 (이동 관절은 1 m 당). 예: 2048
    double gearRatio;          // 엔코더 축 회전수 / 관절 회전수 (감속기 감속비. 직결이면 1)
    bool invert;               // 카운트 방향 반대
    bool absolute;             // true: 관절 0 이 카운트 0. false: 첫 측정 자리가 0
    float noiseCounts;         // 카운트에 더할 가우시안 잡음의 표준편차 (카운트 단위. 0 이면 없음)
    unsigned seed;             // 잡음 난수 시드
    EncoderSettings() : resolution(1024.0), gearRatio(1.0), invert(false), absolute(false), noiseCounts(0.0f), seed(1u) {}
};

struct EncoderReading {
    int64_t ticks;       // 누적 카운트 (방향 반영)
    double position;     // 카운트에서 되돌린 관절 값 (rad 또는 m, 카운트 방향 기준: invert 면 관절 값과 부호 반대, 증분형은 영점 기준). 양자화로 한 카운트까지 다르다
    double velocity;     // 위 position 과 같은 방향의 속도. dt 를 줬고 직전 측정이 있으면 카운트 차이 / dt (양자화됨), 아니면 qd
    bool valid;          // false 면 관절을 못 찾았거나 설정이 이상한 것
    EncoderReading() : ticks(0), position(0), velocity(0), valid(false) {}
};

class Encoder {
public:
    explicit Encoder(const EncoderSettings& settings = EncoderSettings());
    const EncoderSettings& Settings() const { return settings_; }
    void SetSettings(const EncoderSettings& s);  // 영점과 난수를 새로 시작한다
    void Reset();                                // 증분형 영점을 다시 잡는다 (다음 측정 자리가 0)

    // 관절 1회전(또는 1 m)당 카운트. 설정이 이상하면 0
    double CountsPerUnit() const;

    // q: 관절 값, qd: 구동 관절 속도 (없으면 0), dt: 직전 측정부터 지난 시간 (0 이하면 속도는 qd 를 쓴다)
    EncoderReading Measure(const urdf::Robot& robot, const urdf::JointValues& q, const simul::Map<simul::String, double>& qd = simul::Map<simul::String, double>(),
                           double dt = 0.0);

private:
    float gaussian();
    unsigned nextRandom();
    EncoderSettings settings_;
    unsigned rng_;
    bool hasZero_, hasPrev_;
    int64_t zero_, prev_;
};

// URDF 의 <gazebo reference="관절"><sensor type="encoder"> 정의 -> 설정
EncoderSettings EncoderSettingsFromDef(const urdf::EncoderDef& def);

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_ENCODER_H
