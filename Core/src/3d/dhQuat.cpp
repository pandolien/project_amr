#include "3d/dhQuat.h"

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
