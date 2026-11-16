#ifndef CORE_INTERNAL_RAYTRIANGLE_H
#define CORE_INTERNAL_RAYTRIANGLE_H

#include "CBase/Math.h"

// 광선 o + t*d 와 삼각형(p0, p1, p2)의 교차 (Moller-Trumbore, 양면). 맞으면 true 와 t (0 <= t). 평행/퇴화/NaN 은 false.
// RaycastTriangles(벌크)와 TriangleBvh 가 같은 판정을 쓰도록 한곳에 둔다.
namespace amr {
namespace detail {

inline bool RayTriangle(const double o[3], const double d[3], const float* p0, const float* p1, const float* p2, double* tOut) {
    double a[3], e1[3], e2[3];
    for (int k = 0; k < 3; k++) {
        a[k] = p0[k];
        e1[k] = static_cast<double>(p1[k]) - a[k];
        e2[k] = static_cast<double>(p2[k]) - a[k];
    }
    const double p[3] = {d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]};
    const double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
    if (!(amr::Abs(det) > 1e-18)) return false;  // 광선과 평행하거나 퇴화 (NaN 도 여기서 걸러진다)
    const double inv = 1.0 / det;
    const double s[3] = {o[0] - a[0], o[1] - a[1], o[2] - a[2]};
    const double u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) * inv;
    if (!(u >= 0.0 && u <= 1.0)) return false;
    const double q[3] = {s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0]};
    const double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv;
    if (!(v >= 0.0 && u + v <= 1.0)) return false;
    const double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
    if (!(t >= 0.0)) return false;
    *tOut = t;
    return true;
}

}  // namespace detail
}  // namespace amr

#endif  // CORE_INTERNAL_RAYTRIANGLE_H
