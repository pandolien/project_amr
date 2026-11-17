#include "CCollision/MeshRaycast.h"

#include "CBase/Math.h"
#include "RayTriangle.h"

namespace simul {

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
        double tt = 0.0;
        if (!detail::RayTriangle(o, d, positions + i0 * stride, positions + i1 * stride, positions + i2 * stride, &tt)) continue;
        if (tt <= best) {
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

}  // namespace simul
