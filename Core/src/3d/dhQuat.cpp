#include "3d/dhQuat.h"

#include "CBase/Math.h"

#include "3d/dhEuler.h"
#include "3d/dhMat.h"

dhQuat::dhQuat() : w(1), x(0), y(0), z(0) {
    // 회전 없음(항등 쿼터니언)
}

dhQuat::dhQuat(float px, float py, float pz, float pw) : w(pw), x(px), y(py), z(pz) {}

dhMat dhQuat::GetMat() const {
    // dhMat::Quat(const dhQuat&) 의 변환 공식을 그대로 재사용
    return dhMat::Quat(*this);
}

dhEuler dhQuat::GetEuler() const {
    // 쿼터니언 -> 행렬 -> 오일러각(dhMat::RPY) 순서로 변환
    return GetMat().RPY();
}

dhQuat dhQuat::FromAxisAngle(float angle, const dhVector& axis) {
    const float n = axis.Norm();
    if (!(n > 0.0f) || !simul::IsFinite(n)) return dhQuat();
    const float h = angle * 0.5f;
    const float s = simul::Sin(h) / n;
    return dhQuat(axis.x * s, axis.y * s, axis.z * s, simul::Cos(h));
}

float dhQuat::Norm() const { return simul::Sqrt(w * w + x * x + y * y + z * z); }

float dhQuat::Dot(const dhQuat& o) const { return w * o.w + x * o.x + y * o.y + z * o.z; }

dhQuat dhQuat::Normalized() const {
    const float n = Norm();
    if (!(n > 0.0f) || !simul::IsFinite(n)) return dhQuat();
    return dhQuat(x / n, y / n, z / n, w / n);
}

dhQuat dhQuat::Conjugate() const { return dhQuat(-x, -y, -z, w); }

dhQuat dhQuat::operator*(const dhQuat& o) const {
    return dhQuat(w * o.x + x * o.w + y * o.z - z * o.y,   //
                  w * o.y - x * o.z + y * o.w + z * o.x,   //
                  w * o.z + x * o.y - y * o.x + z * o.w,   //
                  w * o.w - x * o.x - y * o.y - z * o.z);
}

dhVector dhQuat::Rotate(const dhVector& v) const {
    // v' = v + 2w (u x v) + 2 u x (u x v),  u = (x, y, z)
    const dhVector u(x, y, z);
    const dhVector t = (u * v) * 2.0f;
    return v + t * w + u * t;
}
