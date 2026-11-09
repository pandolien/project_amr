#include "3d/dhMat.h"

#include <cmath>
#include <cstring>

dhMat::dhMat() {
    for (int i = 0; i < 16; i++) v[i] = 0;
    v[0] = v[5] = v[10] = v[15] = 1;
}

dhMat::~dhMat() {}

dhMat dhMat::operator*(const dhMat& h) const {
    dhMat out;
    int cnt = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float val = 0;
            for (int k = 0; k < 4; k++) val += v[k * 4 + j] * h.v[i * 4 + k];
            out.v[cnt++] = val;
        }
    }
    return out;
}

dhVector dhMat::operator*(const dhVector& l) const {
    dhVector r;
    r.x = l.x * v[0] + l.y * v[4] + l.z * v[8] + v[12];
    r.y = l.x * v[1] + l.y * v[5] + l.z * v[9] + v[13];
    r.z = l.x * v[2] + l.y * v[6] + l.z * v[10] + v[14];
    return r;
}

dhMat dhMat::Trans(float x, float y, float z) {
    dhMat h;
    h.v[12] = x;
    h.v[13] = y;
    h.v[14] = z;
    return h;
}

// angle is radian
dhMat dhMat::RotX(float angle) {
    dhMat h;
    float s = std::sin(angle);
    float c = std::cos(angle);
    h.v[5] = c;
    h.v[9] = -s;
    h.v[6] = s;
    h.v[10] = c;
    return h;
}

dhMat dhMat::RotY(float angle) {
    dhMat h;
    float s = std::sin(angle);
    float c = std::cos(angle);
    h.v[0] = c;
    h.v[8] = s;
    h.v[2] = -s;
    h.v[10] = c;
    return h;
}

dhMat dhMat::RotZ(float angle) {
    dhMat h;
    float s = std::sin(angle);
    float c = std::cos(angle);
    h.v[0] = c;
    h.v[4] = -s;
    h.v[1] = s;
    h.v[5] = c;
    return h;
}

dhMat dhMat::Scale(float s) {
    dhMat h;
    h.v[0] = s;
    h.v[5] = s;
    return h;
}

dhMat dhMat::Scale(float sx, float sy) {
    dhMat h;
    h.v[0] = sx;
    h.v[5] = sy;
    return h;
}

dhMat dhMat::Scale(float sx, float sy, float sz) {
    dhMat h;
    h.v[0] = sx;
    h.v[5] = sy;
    h.v[10] = sz;
    return h;
}

dhMat dhMat::Scale(const dhVector& s) {
    dhMat h;
    h.v[0] = s.x;
    h.v[5] = s.y;
    h.v[10] = s.z;
    return h;
}

dhMat dhMat::Inv() const {
    dhMat ret;
    // 회전부 전치
    ret.v[0] = v[0];
    ret.v[1] = v[4];
    ret.v[2] = v[8];

    ret.v[4] = v[1];
    ret.v[5] = v[5];
    ret.v[6] = v[9];

    ret.v[8] = v[2];
    ret.v[9] = v[6];
    ret.v[10] = v[10];

    // 이동부: -R^T * t
    dhVector minusT = O() * -1.0f;
    dhVector t = ret * minusT;
    ret.v[12] = t.x;
    ret.v[13] = t.y;
    ret.v[14] = t.z;
    return ret;
}

// axis 축을 기준으로 angle(rad)만큼 회전
dhMat dhMat::Quat(float angle, const dhVector& axis) {
    dhVector u = axis.Unit();
    float half = angle * 0.5f;
    float s = std::sin(half);
    dhQuat q;
    q.w = std::cos(half);
    q.x = u.x * s;
    q.y = u.y * s;
    q.z = u.z * s;
    return Quat(q);
}

// 단위 쿼터니언 -> 회전행렬 (column-major)
dhMat dhMat::Quat(const dhQuat& q) {
    dhMat h;
    float x2 = q.x * q.x;
    float y2 = q.y * q.y;
    float z2 = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zw = q.z * q.w;

    h.v[0] = 1 - 2 * (y2 + z2);
    h.v[5] = 1 - 2 * (x2 + z2);
    h.v[10] = 1 - 2 * (x2 + y2);

    h.v[1] = 2 * (xy + zw);
    h.v[2] = 2 * (xz - yw);
    h.v[4] = 2 * (xy - zw);

    h.v[6] = 2 * (yz + xw);
    h.v[8] = 2 * (xz + yw);
    h.v[9] = 2 * (yz - xw);
    return h;
}

// 회전행렬 -> 쿼터니언. R(i,j) = v[j*4+i]
dhQuat dhMat::GetQuat() const {
    dhQuat q;
    q.w = 1;
    q.x = q.y = q.z = 0;

    const float t = v[0] + v[5] + v[10];
    if (t > 0) {
        float S = 2.0f * std::sqrt(t + 1.0f);  // S = 4w
        q.w = S * 0.25f;
        q.x = (v[6] - v[9]) / S;
        q.y = (v[8] - v[2]) / S;
        q.z = (v[1] - v[4]) / S;
    } else if (v[0] >= v[5] && v[0] >= v[10]) {
        float S = 2.0f * std::sqrt(1.0f + v[0] - v[5] - v[10]);  // S = 4x
        q.x = S * 0.25f;
        q.w = (v[6] - v[9]) / S;
        q.y = (v[4] + v[1]) / S;
        q.z = (v[8] + v[2]) / S;
    } else if (v[5] >= v[10]) {
        float S = 2.0f * std::sqrt(1.0f + v[5] - v[0] - v[10]);  // S = 4y
        q.y = S * 0.25f;
        q.w = (v[8] - v[2]) / S;
        q.x = (v[4] + v[1]) / S;
        q.z = (v[9] + v[6]) / S;
    } else {
        float S = 2.0f * std::sqrt(1.0f + v[10] - v[0] - v[5]);  // S = 4z
        q.z = S * 0.25f;
        q.w = (v[1] - v[4]) / S;
        q.x = (v[8] + v[2]) / S;
        q.y = (v[9] + v[6]) / S;
    }
    return q;
}

_RPY dhMat::RPY() const {
    _RPY abr;
    abr.b = std::atan2(-v[2], std::sqrt(v[0] * v[0] + v[1] * v[1]));
    float cb = std::cos(abr.b);
    abr.a = std::atan2(v[1] / cb, v[0] / cb);
    abr.r = std::atan2(v[6] / cb, v[10] / cb);
    return abr;
}

void dhMat::Hmatrix(float x, float y, float z, float a, float b, float r) {
    dhMat h = Trans(x, y, z) * RotZ(a) * RotY(b) * RotX(r);
    std::memcpy(v, h.v, sizeof(float) * 16);
}

dhVector dhMat::O() const { return dhVector(v[12], v[13], v[14]); }
