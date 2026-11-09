#ifndef CORE_3D_DHMAT_H
#define CORE_3D_DHMAT_H

#include "3d/dhVector.h"

// 각도 변환 (기존 PI / RAD / DEG / MIN / MAX / ABS 매크로를 대체)
const double dhPI = 3.14159265358979323846;
inline double dhRad(double deg) { return deg * dhPI / 180.0; }  // degree -> radian
inline double dhDeg(double rad) { return rad * 180.0 / dhPI; }  // radian -> degree

typedef struct {
    float a;  // Z축 회전 (yaw)
    float b;  // Y축 회전 (pitch)
    float r;  // X축 회전 (roll)
} _RPY;

typedef struct {
    float w, x, y, z;
} dhQuat;

// 4x4 동차변환행렬. column-major: v[0..3]=1열, v[12..14]=이동(x,y,z)
// 모든 각도의 단위는 radian.
class dhMat {
public:
    dhMat();  // 단위행렬
    ~dhMat();

    float v[16];

    static dhMat Trans(float x, float y, float z);
    static dhMat RotX(float angle);
    static dhMat RotY(float angle);
    static dhMat RotZ(float angle);
    static dhMat Scale(float s);
    static dhMat Scale(float sx, float sy);
    static dhMat Scale(float sx, float sy, float sz);
    static dhMat Scale(const dhVector& s);
    static dhMat Quat(float angle, const dhVector& axis);  // axis 축으로 angle 회전
    static dhMat Quat(const dhQuat& q);                    // 단위 쿼터니언 -> 회전행렬

    // 강체변환(회전+이동)의 역행렬. Scale이 포함된 행렬에는 사용 불가.
    dhMat Inv() const;
    dhQuat GetQuat() const;
    // R = Rz(a) * Ry(b) * Rx(r). b = ±90deg(gimbal lock)에서는 값이 정의되지 않음.
    _RPY RPY() const;
    // Trans(x,y,z) * RotZ(a) * RotY(b) * RotX(r)
    void Hmatrix(float x, float y, float z, float a, float b, float r);
    dhVector O() const;  // 원점(이동 성분)

    dhMat operator*(const dhMat& h) const;
    dhVector operator*(const dhVector& l) const;
};

#endif  // CORE_3D_DHMAT_H
