#ifndef CORE_3D_DHQUAT_H
#define CORE_3D_DHQUAT_H

#include "3d/dhVector.h"

class dhMat;
class dhEuler;

// 쿼터니언. 주의: 생성자 인자 순서는 (x, y, z, w) 이고 멤버 선언 순서는 w, x, y, z 이다.
// 따라서 dhQuat(1, 0, 0, 0) 은 항등이 아니라 X축 180도 회전이다. 항등은 기본 생성자를 쓴다.
class dhQuat {
public:
    dhQuat();
    dhQuat(float px, float py, float pz, float pw);

    float w, x, y, z;

    // axis 축으로 angle(radian) 회전. axis 는 단위벡터가 아니어도 된다 (영벡터이면 항등)
    static dhQuat FromAxisAngle(float angle, const dhVector& axis);

    float Norm() const;
    float Dot(const dhQuat& o) const;
    dhQuat Normalized() const;          // 길이가 0 이거나 유한하지 않으면 항등 쿼터니언
    dhQuat Conjugate() const;           // 단위 쿼터니언이면 역회전
    dhQuat operator*(const dhQuat& o) const;  // Hamilton 곱: (this * o) 는 o 를 먼저, this 를 나중에 적용하는 회전
    dhVector Rotate(const dhVector& v) const; // 단위 쿼터니언으로 벡터를 회전

    dhMat GetMat() const;      // 단위 쿼터니언 -> 회전행렬
    dhEuler GetEuler() const;  // 쿼터니언 -> 오일러각 (dhMat::RPY 규약)
};

#endif  // CORE_3D_DHQUAT_H
