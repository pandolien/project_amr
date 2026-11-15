#ifndef CORE_3D_DHQUAT_H
#define CORE_3D_DHQUAT_H

class dhMat;
class dhEuler;

// 쿼터니언. 주의: 생성자 인자 순서는 (x, y, z, w) 이고 멤버 선언 순서는 w, x, y, z 이다.
// 따라서 dhQuat(1, 0, 0, 0) 은 항등이 아니라 X축 180도 회전이다. 항등은 기본 생성자를 쓴다.
class dhQuat {
public:
    dhQuat();
    dhQuat(float px, float py, float pz, float pw);

    float w, x, y, z;

    dhMat GetMat() const;      // 단위 쿼터니언 -> 회전행렬
    dhEuler GetEuler() const;  // 쿼터니언 -> 오일러각 (dhMat::RPY 규약)
};

#endif  // CORE_3D_DHQUAT_H
