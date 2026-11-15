#include "CCollision/MeshRaycast.h"

#include "CBase/Math.h"

namespace amr {

bool RaycastTriangles(const float* positions, size_t vertexCount, const uint32_t* indices, size_t indexCount,
                      const dhVector& origin, const dhVector& dir, float tMax, float* tHit, size_t* triangle, size_t stride) {
    if (!positions || !indices || stride < 3 || !(tMax >= 0.0f)) return false;
    const double o[3] = {origin.x, origin.y, origin.z}, d[3] = {dir.x, dir.y, dir.z};
    double best = static_cast<double>(tMax);
    bool found = false;
    size_t bestTri = 0;
    for (size_t t = 0; t + 2 < indexCount; t += 3) {
        const uint32_t i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount) continue;
        double a[3], e1[3], e2[3];
        for (int k = 0; k < 3; k++) {
            a[k] = positions[i0 * stride + static_cast<size_t>(k)];
            e1[k] = static_cast<double>(positions[i1 * stride + static_cast<size_t>(k)]) - a[k];
            e2[k] = static_cast<double>(positions[i2 * stride + static_cast<size_t>(k)]) - a[k];
        }
        // Moller-Trumbore
        const double p[3] = {d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]};
        const double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
        if (!(amr::Abs(det) > 1e-18)) continue;  // 광선과 평행하거나 퇴화 (NaN 도 여기서 걸러진다)
        const double inv = 1.0 / det;
        const double s[3] = {o[0] - a[0], o[1] - a[1], o[2] - a[2]};
        const double u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) * inv;
        if (!(u >= 0.0 && u <= 1.0)) continue;
        const double q[3] = {s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0]};
        const double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv;
        if (!(v >= 0.0 && u + v <= 1.0)) continue;
        const double tt = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
        if (tt >= 0.0 && tt <= best) {
            best = tt;
            bestTri = t / 3;
            found = true;
        }
    }
    if (found) {
        if (tHit) *tHit = static_cast<float>(best);
        if (triangle) *triangle = bestTri;
    }
    return found;
}

}  // namespace amr
