#include "CCollision/Bvh.h"

#include "CBase/Math.h"
#include "CBase/Util.h"
#include "RayTriangle.h"

namespace amr {

namespace {

const size_t kLeafMax = 4;      // 잎에 들어가는 삼각형 수의 상한 (이보다 많으면 쪼갠다)
const int kBins = 16;           // SAH 구간 수
const size_t kForceMedian = 40; // 이 깊이를 넘으면 SAH 대신 반으로 가른다 (깊이 폭주 방지)

struct Box {
    float lo[3], hi[3];
    Box() {
        for (int k = 0; k < 3; k++) {
            lo[k] = kFloatMax;
            hi[k] = -kFloatMax;
        }
    }
    void Add(const float* lo2, const float* hi2) {
        for (int k = 0; k < 3; k++) {
            if (lo2[k] < lo[k]) lo[k] = lo2[k];
            if (hi2[k] > hi[k]) hi[k] = hi2[k];
        }
    }
    float Area() const {
        const float x = hi[0] - lo[0], y = hi[1] - lo[1], z = hi[2] - lo[2];
        if (x < 0 || y < 0 || z < 0) return 0.0f;
        return 2.0f * (x * y + y * z + z * x);
    }
};

}  // namespace

struct TriangleBvh::BuildCtx {
    Array<float> lo;   // 삼각형 번호별 경계 상자 (3 floats)
    Array<float> hi;
    Array<float> ctr;  // 삼각형 번호별 중심 (3 floats)
};

TriangleBvh::TriangleBvh() : positions_(NULL), vertexCount_(0), indices_(NULL), stride_(3), depth_(0) {}

void TriangleBvh::Build(const float* positions, size_t vertexCount, const uint32_t* indices, size_t indexCount, size_t stride) {
    nodes_.clear();
    tris_.clear();
    depth_ = 0;
    positions_ = positions;
    vertexCount_ = vertexCount;
    indices_ = indices;
    stride_ = stride;
    if (!positions || !indices || stride < 3) return;
    const size_t triTotal = indexCount / 3;
    BuildCtx c;
    c.lo.assign(triTotal * 3, 0.0f);
    c.hi.assign(triTotal * 3, 0.0f);
    c.ctr.assign(triTotal * 3, 0.0f);
    for (size_t t = 0; t < triTotal; t++) {
        const uint32_t i[3] = {indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2]};
        if (i[0] >= vertexCount || i[1] >= vertexCount || i[2] >= vertexCount) continue;  // 범위를 벗어난 삼각형은 뺀다
        bool ok = true;
        float lo[3], hi[3];
        for (int k = 0; k < 3 && ok; k++) {
            const float a = positions[i[0] * stride + static_cast<size_t>(k)], b = positions[i[1] * stride + static_cast<size_t>(k)],
                        d = positions[i[2] * stride + static_cast<size_t>(k)];
            if (!IsFinite(a) || !IsFinite(b) || !IsFinite(d)) {
                ok = false;
                break;
            }
            lo[k] = Min(a, Min(b, d));
            hi[k] = Max(a, Max(b, d));
        }
        if (!ok) continue;  // NaN/무한대 정점의 삼각형은 뺀다
        for (int k = 0; k < 3; k++) {
            c.lo[t * 3 + static_cast<size_t>(k)] = lo[k];
            c.hi[t * 3 + static_cast<size_t>(k)] = hi[k];
            c.ctr[t * 3 + static_cast<size_t>(k)] = 0.5f * (lo[k] + hi[k]);
        }
        tris_.push_back(static_cast<uint32_t>(t));
    }
    if (tris_.empty()) return;
    nodes_.reserve(tris_.size() / 2 + 2);
    nodes_.push_back(Node());
    buildNode(c, 0, 0, tris_.size(), 0);
}

void TriangleBvh::buildNode(BuildCtx& c, uint32_t node, size_t begin, size_t end, size_t depth) {
    if (depth + 1 > depth_) depth_ = depth + 1;
    Box box, cbox;
    for (size_t i = begin; i < end; i++) {
        const size_t t = tris_[i];
        box.Add(&c.lo[t * 3], &c.hi[t * 3]);
        cbox.Add(&c.ctr[t * 3], &c.ctr[t * 3]);
    }
    Node n;
    for (int k = 0; k < 3; k++) {
        n.bmin[k] = box.lo[k];
        n.bmax[k] = box.hi[k];
    }
    const size_t count = end - begin;
    int axis = 0;
    for (int k = 1; k < 3; k++)
        if (cbox.hi[k] - cbox.lo[k] > cbox.hi[axis] - cbox.lo[axis]) axis = k;
    const float extent = cbox.hi[axis] - cbox.lo[axis];
    if (count <= kLeafMax || !(extent > 0.0f)) {  // 작거나, 중심이 전부 한 점이라 더 못 가른다 -> 잎
        n.leftFirst = static_cast<uint32_t>(begin);
        n.count = static_cast<uint32_t>(count);
        nodes_[node] = n;
        return;
    }

    size_t mid = begin;
    bool split = false;
    if (depth < kForceMedian) {
        // binned SAH: 중심을 axis 방향으로 kBins 구간에 넣고, 구간 경계마다 (왼쪽 면적 * 개수 + 오른쪽 면적 * 개수)가 가장 작은 곳에서 가른다
        size_t cnt[kBins];
        Box bb[kBins];
        for (int b = 0; b < kBins; b++) cnt[b] = 0;
        const float scale = static_cast<float>(kBins) / extent;
        for (size_t i = begin; i < end; i++) {
            const size_t t = tris_[i];
            int b = static_cast<int>((c.ctr[t * 3 + static_cast<size_t>(axis)] - cbox.lo[axis]) * scale);
            if (b >= kBins) b = kBins - 1;
            if (b < 0) b = 0;
            cnt[b]++;
            bb[b].Add(&c.lo[t * 3], &c.hi[t * 3]);
        }
        float leftArea[kBins - 1], rightArea[kBins - 1];
        size_t leftCnt[kBins - 1], rightCnt[kBins - 1];
        Box acc;
        size_t sum = 0;
        for (int b = 0; b < kBins - 1; b++) {
            acc.Add(bb[b].lo, bb[b].hi);
            sum += cnt[b];
            leftArea[b] = acc.Area();
            leftCnt[b] = sum;
        }
        acc = Box();
        sum = 0;
        for (int b = kBins - 1; b > 0; b--) {
            acc.Add(bb[b].lo, bb[b].hi);
            sum += cnt[b];
            rightArea[b - 1] = acc.Area();
            rightCnt[b - 1] = sum;
        }
        int bestBin = -1;
        float bestCost = kFloatMax;
        for (int b = 0; b < kBins - 1; b++) {
            if (leftCnt[b] == 0 || rightCnt[b] == 0) continue;
            const float cost = leftArea[b] * static_cast<float>(leftCnt[b]) + rightArea[b] * static_cast<float>(rightCnt[b]);
            if (cost < bestCost) {
                bestCost = cost;
                bestBin = b;
            }
        }
        if (bestBin >= 0) {
            // 구간 번호가 bestBin 이하인 삼각형을 앞으로 모은다
            size_t i = begin, j = end;
            while (i < j) {
                const size_t t = tris_[i];
                int b = static_cast<int>((c.ctr[t * 3 + static_cast<size_t>(axis)] - cbox.lo[axis]) * scale);
                if (b >= kBins) b = kBins - 1;
                if (b < 0) b = 0;
                if (b <= bestBin) {
                    i++;
                } else {
                    j--;
                    Swap(tris_[i], tris_[j]);
                }
            }
            mid = i;
            split = mid > begin && mid < end;
        }
    }
    if (!split) {
        // SAH 로 못 갈랐거나 너무 깊다: 중심의 axis 값 순서로 정확히 반으로 가른다 (정렬)
        struct Key {
            float v;
            uint32_t t;
            bool operator<(const Key& o) const { return v < o.v; }
        };
        Array<Key> keys;
        for (size_t i = begin; i < end; i++) {
            Key k;
            k.t = tris_[i];
            k.v = c.ctr[static_cast<size_t>(k.t) * 3 + static_cast<size_t>(axis)];
            keys.push_back(k);
        }
        Sort(keys.begin(), keys.end());
        for (size_t i = 0; i < keys.size(); i++) tris_[begin + i] = keys[i].t;
        mid = begin + count / 2;
    }

    const uint32_t left = static_cast<uint32_t>(nodes_.size());
    nodes_.push_back(Node());
    nodes_.push_back(Node());
    n.leftFirst = left;
    n.count = 0;
    nodes_[node] = n;
    buildNode(c, left, begin, mid, depth + 1);
    buildNode(c, left + 1, mid, end, depth + 1);
}

namespace {

// 광선(o, d, 역수 inv, 평행 여부 par)과 상자의 slab 교차. 구간 [0, tMax] 안에서 만나면 true, 진입 거리를 tNear 에.
inline bool BoxHit(const float* bmin, const float* bmax, const double o[3], const double d[3], const double inv[3], const bool par[3], double tMax,
                   double* tNear) {
    double t0 = 0.0, t1 = tMax;
    for (int k = 0; k < 3; k++) {
        if (par[k]) {
            if (o[k] < bmin[k] || o[k] > bmax[k]) return false;
        } else {
            double a = (bmin[k] - o[k]) * inv[k], b = (bmax[k] - o[k]) * inv[k];
            if (a > b) Swap(a, b);
            if (a > t0) t0 = a;
            if (b < t1) t1 = b;
            if (t0 > t1) return false;
        }
    }
    (void)d;
    *tNear = t0;
    return true;
}

}  // namespace

bool TriangleBvh::Raycast(const dhVector& origin, const dhVector& dir, float tMax, float* tHit, size_t* triangle) const {
    if (nodes_.empty() || !(tMax >= 0.0f)) return false;
    const double o[3] = {origin.x, origin.y, origin.z}, d[3] = {dir.x, dir.y, dir.z};
    if (!(d[0] == d[0] && d[1] == d[1] && d[2] == d[2]) || (d[0] == 0.0 && d[1] == 0.0 && d[2] == 0.0)) return false;
    double inv[3];
    bool par[3];
    for (int k = 0; k < 3; k++) {
        par[k] = Abs(d[k]) < 1e-30;
        inv[k] = par[k] ? 0.0 : 1.0 / d[k];
    }
    double best = static_cast<double>(tMax);
    bool found = false;
    size_t bestTri = 0;
    uint32_t stack[128];
    int sp = 0;
    double rootNear;
    if (!BoxHit(nodes_[0].bmin, nodes_[0].bmax, o, d, inv, par, best, &rootNear)) return false;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& n = nodes_[stack[--sp]];
        if (n.count > 0) {
            for (uint32_t i = 0; i < n.count; i++) {
                const size_t t = tris_[n.leftFirst + i];
                const uint32_t i0 = indices_[t * 3], i1 = indices_[t * 3 + 1], i2 = indices_[t * 3 + 2];
                double tt;
                if (!detail::RayTriangle(o, d, positions_ + i0 * stride_, positions_ + i1 * stride_, positions_ + i2 * stride_, &tt)) continue;
                if (tt <= best) {
                    best = tt;
                    bestTri = t;
                    found = true;
                }
            }
        } else {
            const Node& l = nodes_[n.leftFirst];
            const Node& r = nodes_[n.leftFirst + 1];
            double tl = 0, tr = 0;
            const bool hl = BoxHit(l.bmin, l.bmax, o, d, inv, par, best, &tl);
            const bool hr = BoxHit(r.bmin, r.bmax, o, d, inv, par, best, &tr);
            if (hl && hr) {  // 가까운 쪽을 먼저 (스택이므로 나중에 넣는다)
                if (tl <= tr) {
                    stack[sp++] = n.leftFirst + 1;
                    stack[sp++] = n.leftFirst;
                } else {
                    stack[sp++] = n.leftFirst;
                    stack[sp++] = n.leftFirst + 1;
                }
            } else if (hl) {
                stack[sp++] = n.leftFirst;
            } else if (hr) {
                stack[sp++] = n.leftFirst + 1;
            }
        }
    }
    if (found) {
        if (tHit) *tHit = static_cast<float>(best);
        if (triangle) *triangle = bestTri;
    }
    return found;
}

}  // namespace amr
