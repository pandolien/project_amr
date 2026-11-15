#include "CCollision/Aabb.h"

#include <float.h>
#include <math.h>

namespace amr {

namespace {

inline float Min2(float a, float b) { return a < b ? a : b; }
inline float Max2(float a, float b) { return a > b ? a : b; }
inline float Abs(float a) { return a < 0 ? -a : a; }

}  // namespace

Aabb::Aabb() : lo(FLT_MAX, FLT_MAX, FLT_MAX), hi(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}

Aabb::Aabb(const dhVector& l, const dhVector& h) : lo(l), hi(h) {}

Aabb Aabb::FromCenterHalfExtents(const dhVector& c, const dhVector& h) {
    return Aabb(dhVector(c.x - h.x, c.y - h.y, c.z - h.z), dhVector(c.x + h.x, c.y + h.y, c.z + h.z));
}

bool Aabb::IsEmpty() const {
    // 부정 비교로 NaN 도 빈 상자로 취급한다
    return !(lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z);
}

void Aabb::Expand(const dhVector& p) {
    if (!(p.x == p.x && p.y == p.y && p.z == p.z)) return;  // NaN 점은 무시
    lo.x = Min2(lo.x, p.x);
    lo.y = Min2(lo.y, p.y);
    lo.z = Min2(lo.z, p.z);
    hi.x = Max2(hi.x, p.x);
    hi.y = Max2(hi.y, p.y);
    hi.z = Max2(hi.z, p.z);
}

void Aabb::Merge(const Aabb& o) {
    if (o.IsEmpty()) return;
    if (IsEmpty()) {
        *this = o;
        return;
    }
    lo.x = Min2(lo.x, o.lo.x);
    lo.y = Min2(lo.y, o.lo.y);
    lo.z = Min2(lo.z, o.lo.z);
    hi.x = Max2(hi.x, o.hi.x);
    hi.y = Max2(hi.y, o.hi.y);
    hi.z = Max2(hi.z, o.hi.z);
}

Aabb Aabb::Inflated(float m) const {
    if (IsEmpty()) return *this;
    return Aabb(dhVector(lo.x - m, lo.y - m, lo.z - m), dhVector(hi.x + m, hi.y + m, hi.z + m));
}

dhVector Aabb::Center() const {
    if (IsEmpty()) return dhVector(0, 0, 0);
    return dhVector((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f);
}

dhVector Aabb::Size() const {
    if (IsEmpty()) return dhVector(0, 0, 0);
    return dhVector(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
}

dhVector Aabb::HalfExtents() const {
    const dhVector s = Size();
    return dhVector(s.x * 0.5f, s.y * 0.5f, s.z * 0.5f);
}

float Aabb::Volume() const {
    const dhVector s = Size();
    return s.x * s.y * s.z;
}

bool Aabb::Contains(const dhVector& p) const {
    return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z;
}

bool Aabb::Contains(const Aabb& o) const {
    if (IsEmpty() || o.IsEmpty()) return false;
    return o.lo.x >= lo.x && o.hi.x <= hi.x && o.lo.y >= lo.y && o.hi.y <= hi.y && o.lo.z >= lo.z && o.hi.z <= hi.z;
}

bool Aabb::Overlaps(const Aabb& o) const {
    if (IsEmpty() || o.IsEmpty()) return false;
    return lo.x <= o.hi.x && o.lo.x <= hi.x && lo.y <= o.hi.y && o.lo.y <= hi.y && lo.z <= o.hi.z && o.lo.z <= hi.z;
}

float Aabb::Distance(const Aabb& o) const {
    if (IsEmpty() || o.IsEmpty()) return -1.0f;
    const float gx = Max2(0.0f, Max2(o.lo.x - hi.x, lo.x - o.hi.x));
    const float gy = Max2(0.0f, Max2(o.lo.y - hi.y, lo.y - o.hi.y));
    const float gz = Max2(0.0f, Max2(o.lo.z - hi.z, lo.z - o.hi.z));
    return sqrtf(gx * gx + gy * gy + gz * gz);
}

float Aabb::PenetrationDepth(const Aabb& o) const {
    if (!Overlaps(o)) return 0.0f;
    const float dx = Min2(hi.x, o.hi.x) - Max2(lo.x, o.lo.x);
    const float dy = Min2(hi.y, o.hi.y) - Max2(lo.y, o.lo.y);
    const float dz = Min2(hi.z, o.hi.z) - Max2(lo.z, o.lo.z);
    return Min2(dx, Min2(dy, dz));
}

Aabb Aabb::Transformed(const dhMat& m) const {
    if (IsEmpty()) return Aabb();
    // Arvo 의 방법: 새 중심 = m * 중심, 새 반크기 = |R| * 반크기 (R 은 m 의 3x3 부분).
    // dhMat 은 열 우선이므로 R(i, j) = v[j * 4 + i].
    const dhVector c = Center();
    const dhVector h = HalfExtents();
    const dhVector nc = m * c;
    const float* a = m.v;
    const float hx = Abs(a[0]) * h.x + Abs(a[4]) * h.y + Abs(a[8]) * h.z;
    const float hy = Abs(a[1]) * h.x + Abs(a[5]) * h.y + Abs(a[9]) * h.z;
    const float hz = Abs(a[2]) * h.x + Abs(a[6]) * h.y + Abs(a[10]) * h.z;
    return FromCenterHalfExtents(nc, dhVector(hx, hy, hz));
}

bool Aabb::Raycast(const dhVector& origin, const dhVector& dir, float tMax, float* tHit) const {
    if (IsEmpty() || !(tMax >= 0.0f)) return false;
    const float o[3] = {origin.x, origin.y, origin.z};
    const float d[3] = {dir.x, dir.y, dir.z};
    const float l[3] = {lo.x, lo.y, lo.z};
    const float h[3] = {hi.x, hi.y, hi.z};
    if (d[0] == 0.0f && d[1] == 0.0f && d[2] == 0.0f) return false;
    for (int i = 0; i < 3; i++)
        if (!(o[i] == o[i]) || !(d[i] == d[i])) return false;  // NaN

    float t0 = 0.0f, t1 = tMax;
    for (int i = 0; i < 3; i++) {
        if (d[i] == 0.0f) {
            // 이 축과 평행: 시작점이 두 면 사이에 없으면 영원히 만나지 않는다
            if (!(o[i] >= l[i] && o[i] <= h[i])) return false;
            continue;
        }
        float ta = (l[i] - o[i]) / d[i];
        float tb = (h[i] - o[i]) / d[i];
        if (ta > tb) {
            const float tmp = ta;
            ta = tb;
            tb = tmp;
        }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return false;
    }
    if (tHit) *tHit = t0;
    return true;
}

}  // namespace amr
