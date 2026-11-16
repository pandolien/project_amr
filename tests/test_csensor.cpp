// CSensor 테스트: 2D/3D 라이다. 상자/벽까지의 거리를 해석해와 비교하고, 가림·범위·잡음·자세·예외를 확인한다.
#include <math.h>
#include <stdio.h>

#include "CBase/Array.h"
#include "CBase/Math.h"
#include "CCollision/Bvh.h"
#include "CSensor/Lidar.h"

using namespace amr;
using namespace amr::sensor;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol) { return Abs(a - b) <= tol; }

// 축에 나란한 상자 [lo, hi] 의 메시 (정점 8개, 삼각형 12개)
struct BoxMesh {
    Array<float> pos;
    Array<uint32_t> idx;
    BoxMesh(float x0, float y0, float z0, float x1, float y1, float z1) {
        const float p[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        for (int i = 0; i < 8; i++)
            for (int k = 0; k < 3; k++) pos.push_back(p[i][k]);
        const uint32_t f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
        for (int i = 0; i < 12; i++)
            for (int k = 0; k < 3; k++) idx.push_back(f[i][k]);
    }
    LidarTarget target(int id) const {
        LidarTarget t;
        t.positions = pos.data();
        t.vertexCount = pos.size() / 3;
        t.indices = idx.data();
        t.indexCount = idx.size();
        t.id = id;
        t.ComputeBounds();
        return t;
    }
};

void testRayCount() {
    Lidar l;
    check(l.RayCount() == 541, "default: 270 deg / 0.5 deg = 541 rays");
    LidarSettings s;
    s.angleMin = 0;
    s.angleMax = 0;
    check(Lidar(s).RayCount() == 1, "single ray");
    s.angleIncrement = 0;
    check(Lidar(s).RayCount() == 0, "zero increment: no rays");
    s.angleIncrement = 0.1f;
    s.angleMax = -1;
    check(Lidar(s).RayCount() == 0, "max < min: no rays");
}

void testBoxAhead() {
    // 센서는 원점에서 +x 를 본다. 상자 x 4.5..5.5, y -0.5..0.5, z -1..1
    const BoxMesh box(4.5f, -0.5f, -1.0f, 5.5f, 0.5f, 1.0f);
    const LidarTarget t = box.target(7);
    Lidar l;
    LidarScan scan;
    l.Scan(dhMat(), &t, 1, &scan);
    check(scan.size() == 541, "scan size");
    const size_t mid = 270;  // 각도 0
    check(near(scan.Angle(mid), 0.0, 1e-6), "middle ray is angle 0");
    check(scan.IsHit(mid) && near(scan.ranges[mid], 4.5, 1e-4), "straight ahead: 4.5 m");
    check(scan.targetIds[mid] == 7, "hit target id");
    check(near(scan.points[mid].x, 4.5, 1e-4) && near(scan.points[mid].y, 0.0, 1e-4), "hit point in world");
    // 앞면(x=4.5)에 닿는 광선은 거리 4.5/cos(a). 앞면의 y 범위는 |y| <= 0.5, 즉 |tan a| <= 0.5/4.5
    int hits = 0;
    for (size_t i = 0; i < scan.size(); i++) {
        const double a = scan.Angle(i);
        const bool shouldHit = Abs(tan(a)) <= 0.5 / 4.5 - 1e-6 && Abs(a) < 1.0;
        const bool shouldMiss = Abs(tan(a)) >= 0.5 / 4.5 + 1e-6 || Abs(a) >= 1.0;
        if (shouldHit) {
            hits++;
            if (!scan.IsHit(i) || !near(scan.ranges[i], 4.5 / cos(a), 1e-3)) check(false, "front face: range = 4.5 / cos(angle)");
        }
        if (shouldMiss && scan.IsHit(i) && !(Abs(a) < 1.0 && Abs(tan(a)) < 0.5 / 4.5 + 0.02)) check(false, "outside the box: no return");
    }
    check(hits > 10, "enough rays hit the box");
    check(!scan.IsHit(0) && scan.ranges[0] > 1e30f, "miss => infinity");
}

void testPose() {
    // 센서를 -90 도 돌리면(+x 가 월드 -y 를 본다) 월드 (0,-5) 의 상자가 정면에 온다. 센서를 (1,0) 으로 옮기면 거리는 그대로.
    const BoxMesh box(-0.5f, -5.5f, -1.0f, 0.5f, -4.5f, 1.0f);
    const LidarTarget t = box.target(1);
    Lidar l;
    LidarScan scan;
    dhMat pose = dhMat::Trans(0, 0, 0) * dhMat::RotZ(static_cast<float>(-dhPI / 2));
    l.Scan(pose, &t, 1, &scan);
    check(scan.IsHit(270) && near(scan.ranges[270], 4.5, 1e-3), "rotated sensor: box ahead at 4.5");
    check(near(scan.points[270].x, 0.0, 1e-3) && near(scan.points[270].y, -4.5, 1e-3), "rotated sensor: world hit point");
    pose = dhMat::Trans(0, 1, 0) * dhMat::RotZ(static_cast<float>(-dhPI / 2));  // 센서가 y=1 로: 거리 5.5
    l.Scan(pose, &t, 1, &scan);
    check(near(scan.ranges[270], 5.5, 1e-3), "moved sensor: 5.5 m");
    // 메시를 월드에서 옮긴다 (target.world): 메시 좌표의 상자는 원점 근처, world 로 (0,-5)
    const BoxMesh local(-0.5f, -0.5f, -1.0f, 0.5f, 0.5f, 1.0f);
    LidarTarget t2 = local.target(2);
    t2.world = dhMat::Trans(0, -5, 0);
    l.Scan(dhMat::RotZ(static_cast<float>(-dhPI / 2)), &t2, 1, &scan);
    check(scan.IsHit(270) && near(scan.ranges[270], 4.5, 1e-3), "target.world moves the mesh");
    t2.world = dhMat::Trans(0, -5, 0) * dhMat::RotZ(0.7f);  // 상자를 돌려도 중심에서 정면까지는 정사각형 반폭~반대각선 사이
    l.Scan(dhMat::RotZ(static_cast<float>(-dhPI / 2)), &t2, 1, &scan);
    check(scan.IsHit(270) && scan.ranges[270] > 4.5 - 0.71 && scan.ranges[270] < 4.5 + 0.01, "rotated box is still hit near 4.5");
}

void testOcclusionAndRange() {
    const BoxMesh wallNear(2.9f, -5.0f, -1.0f, 3.0f, 5.0f, 1.0f);   // 가까운 벽 (앞면 x=2.9)
    const BoxMesh wallFar(7.0f, -5.0f, -1.0f, 7.1f, 5.0f, 1.0f);
    LidarTarget ts[2] = {wallFar.target(20), wallNear.target(10)};   // 먼 것이 먼저 와도 가까운 것이 이긴다
    Lidar l;
    LidarScan scan;
    l.Scan(dhMat(), ts, 2, &scan);
    check(near(scan.ranges[270], 2.9, 1e-4) && scan.targetIds[270] == 10, "nearer target wins");
    check(near(scan.ranges[270 + 40], 2.9 / cos(scan.Angle(270 + 40)), 1e-3), "oblique ray on the wall");

    LidarSettings s;
    s.rangeMax = 2.0f;
    l.SetSettings(s);
    l.Scan(dhMat(), ts, 2, &scan);
    check(!scan.IsHit(270), "beyond rangeMax: no return");
    s.rangeMax = 30.0f;
    s.rangeMin = 3.5f;  // 가까운 벽이 rangeMin 안쪽: 잡히지 않는다
    l.SetSettings(s);
    l.Scan(dhMat(), ts, 2, &scan);
    check(!scan.IsHit(270), "closer than rangeMin: no return");
}

void testNoise() {
    const BoxMesh wall(9.9f, -50.0f, -1.0f, 10.0f, 50.0f, 1.0f);
    const LidarTarget t = wall.target(0);
    LidarSettings s;
    s.noiseStdDev = 0.02f;
    s.angleMin = s.angleMax = 0.0f;  // 같은 광선을 여러 번
    Lidar l(s);
    LidarScan scan;
    double sum = 0, sum2 = 0;
    const int n = 4000;
    for (int i = 0; i < n; i++) {
        l.Scan(dhMat(), &t, 1, &scan);
        const double r = scan.ranges[0] - 9.9;
        sum += r;
        sum2 += r * r;
    }
    const double mean = sum / n, sd = sqrt(sum2 / n - mean * mean);
    check(Abs(mean) < 0.002, "noise: mean ~ 0");
    check(near(sd, 0.02, 0.002), "noise: std dev ~ 0.02");
    // 같은 시드면 같은 결과
    Lidar a(s), b(s);
    LidarScan sa, sb;
    a.Scan(dhMat(), &t, 1, &sa);
    b.Scan(dhMat(), &t, 1, &sb);
    check(sa.ranges[0] == sb.ranges[0], "same seed: same noise");
}

void testMultiChannel() {
    // 수직 채널 16 개 (-15 ~ +15 도). 벽 x = 5 와 바닥 z = -1.
    const BoxMesh wall(5.0f, -50.0f, -50.0f, 5.1f, 50.0f, 50.0f);
    const BoxMesh floorBox(-50.0f, -50.0f, -1.1f, 50.0f, 50.0f, -1.0f);
    LidarTarget ts[2] = {wall.target(1), floorBox.target(2)};
    LidarSettings s;
    s.angleMin = static_cast<float>(-dhPI / 4);
    s.angleMax = static_cast<float>(dhPI / 4);
    s.angleIncrement = static_cast<float>(dhPI / 180.0 * 5.0);
    s.channels = 16;
    s.elevationMin = static_cast<float>(-15.0 * dhPI / 180.0);
    s.elevationMax = static_cast<float>(15.0 * dhPI / 180.0);
    Lidar l(s);
    LidarScan scan;
    l.Scan(dhMat(), ts, 2, &scan);
    check(l.RayCount() == 19 * 16 && scan.size() == 19 * 16, "3D: rays = azimuth x channels");
    check(scan.azimuthCount == 19 && scan.channelCount == 16, "3D: scan shape");
    check(near(scan.Elevation(0), -15.0 * dhPI / 180, 1e-6) && near(scan.Elevation(15), 15.0 * dhPI / 180, 1e-6), "3D: channel elevations span the range");
    int wallHits = 0, floorHits = 0;
    for (size_t i = 0; i < scan.size(); i++) {
        const double th = scan.Angle(i), ph = scan.Elevation(scan.Channel(i));
        if (!scan.IsHit(i)) {
            check(false, "3D: every ray hits the wall or the floor");
            continue;
        }
        const double dx = cos(ph) * cos(th), dz = sin(ph);
        const double tWall = 5.0 / dx;                               // x = 5 평면까지
        const double tFloor = dz < 0 ? -1.0 / dz : 1e30;             // z = -1 평면까지
        const double expect = tWall < tFloor ? tWall : tFloor;
        if (!near(scan.ranges[i], expect, 2e-3)) check(false, "3D: range = distance to the nearer of wall / floor");
        if (scan.targetIds[i] != (tWall < tFloor ? 1 : 2)) check(false, "3D: hit target id (wall vs floor)");
        if (tWall < tFloor) wallHits++; else floorHits++;
        // 맞은 점이 방향 * 거리와 같다
        const double px = scan.ranges[i] * cos(ph) * cos(th), py = scan.ranges[i] * cos(ph) * sin(th), pz = scan.ranges[i] * sin(ph);
        if (!near(scan.points[i].x, px, 2e-3) || !near(scan.points[i].y, py, 2e-3) || !near(scan.points[i].z, pz, 2e-3))
            check(false, "3D: hit point = range * direction");
    }
    check(wallHits > 0 && floorHits > 0, "3D: both the wall and the floor are seen");

    // 센서를 아래로 10 도 기울이면 (RotY(+10 도): +x 가 아래로) 수평 광선이 바닥에 닿는다: t = 1 / sin(10 도)
    LidarSettings one;
    one.angleMin = one.angleMax = 0.0f;
    Lidar l1(one);
    l1.Scan(dhMat::RotY(static_cast<float>(10.0 * dhPI / 180)), &ts[1], 1, &scan);
    check(scan.IsHit(0) && near(scan.ranges[0], 1.0 / sin(10.0 * dhPI / 180), 1e-3), "tilted sensor: horizontal ray reaches the floor at 1/sin(10 deg)");

    // 채널 1 개라도 고각이 0 이 아니면 (평면 가속을 쓰지 않는 경로) 벽까지 5 / (cos(phi) cos(theta))
    LidarSettings up = one;
    up.elevationMin = 0.2f;
    Lidar l2(up);
    l2.Scan(dhMat(), &ts[0], 1, &scan);
    check(scan.IsHit(0) && near(scan.ranges[0], 5.0 / cos(0.2), 1e-3), "one channel with elevation 0.2 rad");

    // 평면 한쪽에만 있는 벽 (z 0.5 ~ 50: 센서 수평면 z = 0 의 위쪽). 수평 광선은 못 보지만 고각 0.2 rad 광선은 z ~ 1 에서 맞는다.
    // 이 경우 평면 가속을 잘못 쓰면 삼각형이 다 걸러져서 놓친다.
    const BoxMesh upperWall(5.0f, -50.0f, 0.5f, 5.1f, 50.0f, 50.0f);
    const LidarTarget uw = upperWall.target(9);
    l1.Scan(dhMat(), &uw, 1, &scan);
    check(!scan.IsHit(0), "horizontal ray passes under the upper wall");
    l2.Scan(dhMat(), &uw, 1, &scan);
    check(scan.IsHit(0) && near(scan.ranges[0], 5.0 / cos(0.2), 1e-3), "elevated ray hits a wall that lies entirely above the horizontal plane");

    // 잘못된 채널 수
    LidarSettings bad;
    bad.channels = 0;
    check(Lidar(bad).RayCount() == 0, "channels = 0: no rays");
}

void testPlanarShortcutConsistency() {
    // 평면 가속(수평 1채널)을 쓰는 경로와 쓰지 않는 경로(고각 2e-6 rad: 사실상 같은 평면)의 결과가 같다
    const BoxMesh a(2.0f, -1.0f, -1.0f, 3.0f, 1.0f, 1.0f), b(-4.0f, 1.0f, -0.5f, -3.0f, 3.0f, 0.5f), c(1.0f, -6.0f, 0.2f, 4.0f, -5.0f, 0.4f);
    LidarTarget ts[3] = {a.target(1), b.target(2), c.target(3)};
    LidarSettings s;
    Lidar fast(s);
    s.elevationMin = s.elevationMax = 2e-6f;
    Lidar slow(s);
    LidarScan sf, ss;
    const dhMat pose = dhMat::Trans(0.3f, 0.2f, 0.1f) * dhMat::RotZ(0.4f);
    fast.Scan(pose, ts, 3, &sf);
    slow.Scan(pose, ts, 3, &ss);
    bool same = sf.size() == ss.size();
    int hits = 0;
    for (size_t i = 0; same && i < sf.size(); i++) {
        same = sf.targetIds[i] == ss.targetIds[i] && (!sf.IsHit(i) || near(sf.ranges[i], ss.ranges[i], 1e-3));
        hits += sf.IsHit(i) ? 1 : 0;
    }
    check(same, "plane shortcut gives the same result as the general path");
    check(hits > 20, "consistency scene has hits");
}

void testBvhGivesSameScan() {
    // 같은 장면을 BVH 없이/있이 스캔해서 같은 결과가 나오는지 (2D 와 3D 모두)
    const BoxMesh a(2.0f, -1.0f, -1.0f, 3.0f, 1.0f, 1.0f), b(-4.0f, 1.0f, -0.5f, -3.0f, 3.0f, 0.5f), c(1.0f, -6.0f, 0.2f, 4.0f, -5.0f, 0.4f);
    TriangleBvh bvhA, bvhB, bvhC;
    bvhA.Build(a.pos.data(), a.pos.size() / 3, a.idx.data(), a.idx.size());
    bvhB.Build(b.pos.data(), b.pos.size() / 3, b.idx.data(), b.idx.size());
    bvhC.Build(c.pos.data(), c.pos.size() / 3, c.idx.data(), c.idx.size());
    LidarTarget plain[3] = {a.target(1), b.target(2), c.target(3)};
    LidarTarget fast[3] = {plain[0], plain[1], plain[2]};
    fast[0].bvh = &bvhA;
    fast[1].bvh = &bvhB;
    fast[2].bvh = &bvhC;
    for (int ch = 1; ch <= 8; ch += 7) {  // 2D (1 채널)와 3D (8 채널)
        LidarSettings s;
        s.channels = ch;
        s.elevationMin = ch > 1 ? -0.3f : 0.0f;
        s.elevationMax = ch > 1 ? 0.3f : 0.0f;
        Lidar l1(s), l2(s);
        LidarScan sp, sf;
        const dhMat pose = dhMat::Trans(0.3f, 0.2f, 0.1f) * dhMat::RotZ(0.4f);
        l1.Scan(pose, plain, 3, &sp);
        l2.Scan(pose, fast, 3, &sf);
        bool same = sp.size() == sf.size();
        int hits = 0;
        for (size_t i = 0; same && i < sp.size(); i++) {
            same = sp.targetIds[i] == sf.targetIds[i] && (!sp.IsHit(i) || near(sp.ranges[i], sf.ranges[i], 1e-4));
            hits += sp.IsHit(i) ? 1 : 0;
        }
        check(same, ch == 1 ? "BVH: same 2D scan" : "BVH: same 3D scan");
        check(hits > 20, "BVH consistency scene has hits");
    }
}

void testEdgeCases() {
    Lidar l;
    LidarScan scan;
    l.Scan(dhMat(), NULL, 0, &scan);
    check(scan.size() == 541 && !scan.IsHit(0) && !scan.IsHit(270), "no targets: all rays miss");
    LidarTarget bad;  // 비어 있는 대상은 건너뛴다
    l.Scan(dhMat(), &bad, 1, &scan);
    check(!scan.IsHit(270), "empty target is skipped");
    // 인덱스가 정점 범위를 벗어난 삼각형은 건너뛴다 (RaycastTriangles 가 처리)
    const BoxMesh box(4.5f, -0.5f, -1.0f, 5.5f, 0.5f, 1.0f);
    LidarTarget t = box.target(3);
    Array<uint32_t> badIdx = box.idx;
    badIdx[0] = 9999;
    t.indices = badIdx.data();
    l.Scan(dhMat(), &t, 1, &scan);
    check(scan.IsHit(270), "an out-of-range triangle index does not break the scan");
    // 경계 상자가 없어도 같은 결과
    LidarTarget u = box.target(3);
    u.hasBounds = false;
    LidarScan s2, s3;
    l.Scan(dhMat(), &u, 1, &s2);
    const LidarTarget v = box.target(3);
    l.Scan(dhMat(), &v, 1, &s3);
    bool same = s2.size() == s3.size();
    for (size_t i = 0; same && i < s2.size(); i++) same = s2.targetIds[i] == s3.targetIds[i] && (s2.ranges[i] == s3.ranges[i]);
    check(same, "bounds box is only a speed-up (same result)");
}

}  // namespace

int main() {
    testRayCount();
    testBoxAhead();
    testPose();
    testOcclusionAndRange();
    testNoise();
    testMultiChannel();
    testPlanarShortcutConsistency();
    testBvhGivesSameScan();
    testEdgeCases();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_csensor: all passed\n");
    return 0;
}
