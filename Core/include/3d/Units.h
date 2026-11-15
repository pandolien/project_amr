#ifndef CORE_3D_UNITS_H
#define CORE_3D_UNITS_H

#include "3d/dhEuler.h"
#include "3d/dhMat.h"
#include "3d/dhQuat.h"

// 단위 규약
//   내부 계산과 데이터 전송 : 길이 m, 각도 radian, 방향은 쿼터니언 (x, y, z, w)
//   사람에게 보여주거나 입력받을 때 : 각도 degree
// 변환은 사람과 맞닿는 경계(화면, 콘솔, 명령행)에서만 하고, 그 안쪽에서는 degree 를 쓰지 않는다.
namespace amr {
namespace units {

const double kPi = 3.14159265358979323846;

inline double DegToRad(double deg) { return deg * kPi / 180.0; }
inline double RadToDeg(double rad) { return rad * 180.0 / kPi; }

// 화면에 보여줄 방향: roll(X) pitch(Y) yaw(Z) degree. R = Rz(yaw) * Ry(pitch) * Rx(roll)  (URDF rpy 와 같은 규약)
// pitch 가 +-90 도(짐벌 락)에 가까우면 roll 과 yaw 는 하나로 정해지지 않는다 (표시용으로만 쓴다).
struct RpyDeg {
    double roll, pitch, yaw;
    RpyDeg() : roll(0), pitch(0), yaw(0) {}
    RpyDeg(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
};

inline RpyDeg RpyDegFromQuat(const dhQuat& q) {
    const dhEuler e = q.Normalized().GetEuler();  // a = yaw, b = pitch, r = roll (radian)
    return RpyDeg(RadToDeg(static_cast<double>(e.r)), RadToDeg(static_cast<double>(e.b)), RadToDeg(static_cast<double>(e.a)));
}

inline dhQuat QuatFromRpyDeg(const RpyDeg& d) {
    return dhEuler(static_cast<float>(DegToRad(d.yaw)), static_cast<float>(DegToRad(d.pitch)), static_cast<float>(DegToRad(d.roll))).GetQuat();
}

}  // namespace units
}  // namespace amr

#endif  // CORE_3D_UNITS_H
