#include "CCollision/Obb.h"

#include <math.h>

namespace simul {

namespace {

inline double Dot(const dhVector& a, const dhVector& b) {
    return static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y + static_cast<double>(a.z) * b.z;
}

inline double Comp(const dhVector& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

inline double Abs(double a) { return a < 0 ? -a : a; }

}  // namespace

Obb::Obb() : center(0, 0, 0), half(-1, -1, -1) {
    axis[0] = dhVector(1, 0, 0);
    axis[1] = dhVector(0, 1, 0);
    axis[2] = dhVector(0, 0, 1);
}

Obb::Obb(const dhVector& c, const dhVector& ax0, const dhVector& ax1, const dhVector& ax2, const dhVector& h)
    : center(c), half(h) {
    axis[0] = ax0;
    axis[1] = ax1;
    axis[2] = ax2;
}

Obb Obb::FromAabb(const Aabb& box) {
    Obb o;
    if (box.IsEmpty()) return o;
    o.center = box.Center();
    o.half = box.HalfExtents();
    return o;
}

bool Obb::IsEmpty() const { return !(half.x >= 0 && half.y >= 0 && half.z >= 0); }

float Obb::Volume() const {
    if (IsEmpty()) return 0.0f;
    return 8.0f * half.x * half.y * half.z;
}

void Obb::Corners(dhVector out[8]) const {
    for (int i = 0; i < 8; i++) {
        const float sx = (i & 1) ? half.x : -half.x;
        const float sy = (i & 2) ? half.y : -half.y;
        const float sz = (i & 4) ? half.z : -half.z;
        out[i] = dhVector(center.x + axis[0].x * sx + axis[1].x * sy + axis[2].x * sz,
                          center.y + axis[0].y * sx + axis[1].y * sy + axis[2].y * sz,
                          center.z + axis[0].z * sx + axis[1].z * sy + axis[2].z * sz);
    }
}

Aabb Obb::ToAabb() const {
    if (IsEmpty()) return Aabb();
    // 축 k 방향 반크기 = sum_i |axis[i].k| * half[i]
    const float h[3] = {half.x, half.y, half.z};
    float e[3];
    for (int k = 0; k < 3; k++) {
        float s = 0;
        for (int i = 0; i < 3; i++) s += static_cast<float>(Abs(Comp(axis[i], k))) * h[i];
        e[k] = s;
    }
    return Aabb::FromCenterHalfExtents(center, dhVector(e[0], e[1], e[2]));
}

bool Obb::Contains(const dhVector& p) const {
    if (IsEmpty()) return false;
    const dhVector d = p - center;
    const double h[3] = {half.x, half.y, half.z};
    for (int i = 0; i < 3; i++)
        if (Abs(Dot(d, axis[i])) > h[i]) return false;
    return true;
}

Obb Obb::Inflated(float m) const {
    if (IsEmpty()) return *this;
    Obb o = *this;
    o.half = dhVector(half.x + m, half.y + m, half.z + m);
    if (o.IsEmpty()) return Obb();
    return o;
}

Obb Obb::Transformed(const dhMat& m) const {
    if (IsEmpty()) return Obb();
    Obb o;
    o.center = m * center;
    const float h[3] = {half.x, half.y, half.z};
    float nh[3];
    for (int i = 0; i < 3; i++) {
        // 축을 m 의 3x3 부분(열 우선: R(r, c) = v[c * 4 + r])으로 돌린 벡터
        const float* a = m.v;
        const dhVector r(a[0] * axis[i].x + a[4] * axis[i].y + a[8] * axis[i].z,
                         a[1] * axis[i].x + a[5] * axis[i].y + a[9] * axis[i].z,
                         a[2] * axis[i].x + a[6] * axis[i].y + a[10] * axis[i].z);
        const float len = r.Norm();
        if (len > 0.0f) {
            o.axis[i] = r * (1.0f / len);
            nh[i] = h[i] * len;
        } else {
            o.axis[i] = axis[i];  // 축이 찌그러져 길이가 0 이 된 경우: 크기 0 으로 만든다
            nh[i] = 0.0f;
        }
    }
    o.half = dhVector(nh[0], nh[1], nh[2]);
    return o;
}

bool Obb::Overlaps(const Obb& o) const {
    if (IsEmpty() || o.IsEmpty()) return false;
    // Ericson, Real-Time Collision Detection 4.4.1. 이 상자(A)의 좌표계에서 계산한다.
    const double ea[3] = {half.x, half.y, half.z};
    const double eb[3] = {o.half.x, o.half.y, o.half.z};
    double R[3][3], AbsR[3][3];
    const double kEps = 1e-6;  // 평행에 가까운 축의 외적이 0 이 되는 것과 반올림 오차에 대한 여유
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            R[i][j] = Dot(axis[i], o.axis[j]);
            AbsR[i][j] = Abs(R[i][j]) + kEps;
        }
    const dhVector tw = o.center - center;
    double t[3];
    for (int i = 0; i < 3; i++) t[i] = Dot(tw, axis[i]);
    // NaN 이 섞이면 비교가 모두 거짓이 되어 "겹침" 으로 잘못 나오므로 먼저 걸러낸다
    for (int i = 0; i < 3; i++) {
        if (!(t[i] == t[i])) return false;
        for (int j = 0; j < 3; j++)
            if (!(R[i][j] == R[i][j])) return false;
    }

    // A 의 면 법선 3개
    for (int i = 0; i < 3; i++) {
        const double ra = ea[i];
        const double rb = eb[0] * AbsR[i][0] + eb[1] * AbsR[i][1] + eb[2] * AbsR[i][2];
        if (Abs(t[i]) > ra + rb) return false;
    }
    // B 의 면 법선 3개
    for (int j = 0; j < 3; j++) {
        const double ra = ea[0] * AbsR[0][j] + ea[1] * AbsR[1][j] + ea[2] * AbsR[2][j];
        const double rb = eb[j];
        if (Abs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]) > ra + rb) return false;
    }
    // 모서리 방향의 외적 9개: L = A_i x B_j
    for (int i = 0; i < 3; i++) {
        const int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (int j = 0; j < 3; j++) {
            const int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
            const double ra = ea[i1] * AbsR[i2][j] + ea[i2] * AbsR[i1][j];
            const double rb = eb[j1] * AbsR[i][j2] + eb[j2] * AbsR[i][j1];
            if (Abs(t[i2] * R[i1][j] - t[i1] * R[i2][j]) > ra + rb) return false;
        }
    }
    return true;  // 15개 축 어디에서도 분리되지 않았다
}

}  // namespace simul
