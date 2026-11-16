// TriangleBvh 테스트: 브루트포스(RaycastTriangles)와 같은 답을 내는지, 예외 입력, 대형 메시에서 속도
#include <stdio.h>
#include <time.h>

#include "CBase/Array.h"
#include "CBase/Math.h"
#include "CCollision/Bvh.h"
#include "CCollision/MeshRaycast.h"

using namespace amr;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    double next() {
        s = s * 1664525u + 1013904223u;
        return static_cast<double>((s >> 8) & 0xFFFFFF) / 16777216.0;
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

struct Soup {
    Array<float> pos;  // stride 3
    Array<uint32_t> idx;
};

// 한 변 크기가 size 정도인 작은 삼각형 count 개를 [-extent, extent]^3 에 흩뿌린다
Soup randomSoup(size_t count, double extent, double size, unsigned seed) {
    Rng rng(seed);
    Soup s;
    for (size_t i = 0; i < count; i++) {
        const double cx = rng.range(-extent, extent), cy = rng.range(-extent, extent), cz = rng.range(-extent, extent);
        for (int k = 0; k < 3; k++) {
            s.pos.push_back(static_cast<float>(cx + rng.range(-size, size)));
            s.pos.push_back(static_cast<float>(cy + rng.range(-size, size)));
            s.pos.push_back(static_cast<float>(cz + rng.range(-size, size)));
            s.idx.push_back(static_cast<uint32_t>(i * 3 + static_cast<size_t>(k)));
        }
    }
    return s;
}

// 브루트포스와 BVH 가 같은 답을 내는지 광선 rays 개로 비교한다. 같은 거리에서 다른 삼각형을 고르는 동점은 허용.
int compare(const TriangleBvh& bvh, const float* pos, size_t vertexCount, const uint32_t* idx, size_t indexCount, size_t stride, size_t rays,
            unsigned seed, double extent, float tMax, int* hitCount) {
    Rng rng(seed);
    int bad = 0;
    *hitCount = 0;
    for (size_t r = 0; r < rays; r++) {
        const dhVector o(static_cast<float>(rng.range(-extent, extent)), static_cast<float>(rng.range(-extent, extent)), static_cast<float>(rng.range(-extent, extent)));
        dhVector d(static_cast<float>(rng.range(-1, 1)), static_cast<float>(rng.range(-1, 1)), static_cast<float>(rng.range(-1, 1)));
        // 일부 광선은 축에 나란히 (성분이 0)
        if (r % 7 == 0) d.x = 0.0f;
        if (r % 11 == 0) d.y = 0.0f;
        float tb = 0, tv = 0;
        size_t fb = 0, fv = 0;
        const bool hb = RaycastTriangles(pos, vertexCount, idx, indexCount, o, d, tMax, &tb, &fb, stride);
        const bool hv = bvh.Raycast(o, d, tMax, &tv, &fv);
        if (hb != hv) {
            ++bad;
            continue;
        }
        if (hb) {
            ++*hitCount;
            if (Abs(tb - tv) > 1e-4f * (1.0f + Abs(tb))) ++bad;
        }
    }
    return bad;
}

void testRandomSoup() {
    const Soup s = randomSoup(3000, 5.0, 0.4, 7);
    TriangleBvh bvh;
    bvh.Build(s.pos.data(), s.pos.size() / 3, s.idx.data(), s.idx.size());
    check(bvh.TriangleCount() == 3000, "soup: all triangles are in the tree");
    check(bvh.Depth() > 3 && bvh.Depth() < 40, "soup: reasonable depth");
    int hits = 0;
    const int bad = compare(bvh, s.pos.data(), s.pos.size() / 3, s.idx.data(), s.idx.size(), 3, 3000, 11, 6.0, 100.0f, &hits);
    check(bad == 0, "soup: BVH == brute force (3000 random rays)");
    check(hits > 300, "soup: enough hits to be meaningful");
    // tMax 가 짧아도 같다
    const int bad2 = compare(bvh, s.pos.data(), s.pos.size() / 3, s.idx.data(), s.idx.size(), 3, 3000, 12, 6.0, 1.5f, &hits);
    check(bad2 == 0, "soup: BVH == brute force with a short tMax");
}

void testStride6AndBadInput() {
    // 위치+법선 교차 배열 (stride 6) + 범위 밖 인덱스 + NaN 정점 + 퇴화 삼각형
    Soup s = randomSoup(500, 3.0, 0.5, 21);
    Array<float> inter;
    for (size_t v = 0; v < s.pos.size() / 3; v++) {
        for (int k = 0; k < 3; k++) inter.push_back(s.pos[v * 3 + static_cast<size_t>(k)]);
        inter.push_back(0);
        inter.push_back(0);
        inter.push_back(1);
    }
    Array<uint32_t> idx = s.idx;
    idx[0] = 999999;  // 범위 밖
    inter[6 * 5 + 1] = static_cast<float>(NanFloat());  // 정점 5 의 y 가 NaN
    // 퇴화 (세 점이 같음)
    const uint32_t base = static_cast<uint32_t>(inter.size() / 6);
    for (int k = 0; k < 3; k++) {
        inter.push_back(1.0f);
        inter.push_back(1.0f);
        inter.push_back(1.0f);
        inter.push_back(0);
        inter.push_back(0);
        inter.push_back(1);
        idx.push_back(base + static_cast<uint32_t>(k));
    }
    TriangleBvh bvh;
    bvh.Build(inter.data(), inter.size() / 6, idx.data(), idx.size(), 6);
    int hits = 0;
    const int bad = compare(bvh, inter.data(), inter.size() / 6, idx.data(), idx.size(), 6, 2000, 31, 4.0, 100.0f, &hits);
    check(bad == 0, "stride 6 + bad input: BVH == brute force");
    check(bvh.TriangleCount() < 500 + 1, "bad triangles (range / NaN) are left out of the tree");
}

void testSmallAndEmpty() {
    TriangleBvh bvh;
    float t = 0;
    check(!bvh.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 10, &t), "empty BVH: no hit");
    bvh.Build(NULL, 0, NULL, 0);
    check(bvh.empty(), "Build(NULL): empty");
    const float pos[9] = {1, -1, -1, 1, 1, -1, 1, 0, 1};  // x = 1 평면의 삼각형
    const uint32_t idx[3] = {0, 1, 2};
    bvh.Build(pos, 3, idx, 3);
    check(!bvh.empty() && bvh.TriangleCount() == 1, "single triangle");
    size_t tri = 99;
    check(bvh.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 10, &t, &tri) && Abs(t - 1.0f) < 1e-6f && tri == 0, "single triangle: hit at t = 1");
    check(!bvh.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 0.5f, &t), "tMax before the triangle: miss");
    check(bvh.Raycast(dhVector(0, 0, 0), dhVector(2, 0, 0), 10, &t) && Abs(t - 0.5f) < 1e-6f, "dir length is not normalized: t in dir units");
    check(!bvh.Raycast(dhVector(0, 0, 0), dhVector(-1, 0, 0), 10, &t), "ray pointing away: miss");
    check(!bvh.Raycast(dhVector(0, 0, 0), dhVector(0, 0, 0), 10, &t), "zero direction: miss");
    check(!bvh.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), -1.0f, &t), "negative tMax: miss");
    check(!bvh.Raycast(dhVector(0, 5, 0), dhVector(1, 0, 0), 10, &t), "ray beside the triangle: miss");
    check(bvh.Raycast(dhVector(2, 0, 0), dhVector(-1, 0, 0), 10, &t) && Abs(t - 1.0f) < 1e-6f, "two-sided: hit from behind");
}

void testCoincidentCentroids() {
    // 같은 자리에 겹친 삼각형 200 개 (중심이 전부 같아 가를 수 없다 -> 큰 잎). 앞쪽 하나가 맞아야 한다.
    Array<float> pos;
    Array<uint32_t> idx;
    for (int i = 0; i < 200; i++)
        for (int k = 0; k < 3; k++) {
            const float p[3][3] = {{2, -1, -1}, {2, 1, -1}, {2, 0, 1}};
            for (int a = 0; a < 3; a++) pos.push_back(p[k][a]);
            idx.push_back(static_cast<uint32_t>(i * 3 + k));
        }
    TriangleBvh bvh;
    bvh.Build(pos.data(), pos.size() / 3, idx.data(), idx.size());
    float t = 0;
    check(bvh.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 10, &t) && Abs(t - 2.0f) < 1e-6f, "coincident triangles: hit");
    check(bvh.Depth() < 40, "coincident triangles: no runaway depth");
}

void testGridPerformance() {
    // 격자 메시 (삼각형 약 20만). 정확도와 속도.
    const int n = 320;
    Array<float> pos;
    Array<uint32_t> idx;
    for (int i = 0; i <= n; i++)
        for (int j = 0; j <= n; j++) {
            pos.push_back(static_cast<float>(i) * 0.05f - 8.0f);
            pos.push_back(3.0f + static_cast<float>((i * j) % 5) * 0.002f);
            pos.push_back(static_cast<float>(j) * 0.05f);
        }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            const uint32_t a = static_cast<uint32_t>(i * (n + 1) + j), b = a + 1, c = a + static_cast<uint32_t>(n) + 1, d = c + 1;
            const uint32_t t[6] = {a, b, d, a, d, c};
            for (int k = 0; k < 6; k++) idx.push_back(t[k]);
        }
    TriangleBvh bvh;
    clock_t c0 = clock();
    bvh.Build(pos.data(), pos.size() / 3, idx.data(), idx.size());
    const double buildMs = 1000.0 * static_cast<double>(clock() - c0) / CLOCKS_PER_SEC;
    check(bvh.TriangleCount() == idx.size() / 3, "grid: all triangles in the tree");
    // 정확도: 브루트포스와 같은 답 (광선 300 개만. 브루트포스가 느리다)
    int hits = 0;
    Rng rng(5);
    int bad = 0;
    for (int r = 0; r < 300; r++) {
        const dhVector o(static_cast<float>(rng.range(-6, 6)), 0.0f, static_cast<float>(rng.range(1, 15)));
        const dhVector d(static_cast<float>(rng.range(-0.5, 0.5)), 1.0f, static_cast<float>(rng.range(-0.5, 0.5)));
        float tb = 0, tv = 0;
        const bool hb = RaycastTriangles(pos.data(), pos.size() / 3, idx.data(), idx.size(), o, d, 50.0f, &tb);
        const bool hv = bvh.Raycast(o, d, 50.0f, &tv);
        if (hb != hv || (hb && Abs(tb - tv) > 1e-4f * (1.0f + Abs(tb)))) ++bad;
        hits += hb ? 1 : 0;
    }
    check(bad == 0, "grid: BVH == brute force");
    check(hits > 100, "grid: enough hits");
    // 속도: 광선 20000 개
    c0 = clock();
    int h2 = 0;
    for (int r = 0; r < 20000; r++) {
        const dhVector o(static_cast<float>(rng.range(-6, 6)), 0.0f, static_cast<float>(rng.range(1, 15)));
        const dhVector d(static_cast<float>(rng.range(-0.5, 0.5)), 1.0f, static_cast<float>(rng.range(-0.5, 0.5)));
        float t = 0;
        h2 += bvh.Raycast(o, d, 50.0f, &t) ? 1 : 0;
    }
    const double rayUs = 1e6 * static_cast<double>(clock() - c0) / CLOCKS_PER_SEC / 20000.0;
    printf("grid: %zu triangles, build %.0f ms, depth %zu, %.2f us / ray (%d hits)\n", idx.size() / 3, buildMs, bvh.Depth(), rayUs, h2);
    check(rayUs < 50.0, "grid: a ray takes well under 50 us");
    check(buildMs < 5000.0, "grid: build takes a few seconds at most");
}

}  // namespace

int main() {
    testRandomSoup();
    testStride6AndBadInput();
    testSmallAndEmpty();
    testCoincidentCentroids();
    testGridPerformance();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_bvh: all passed\n");
    return 0;
}
