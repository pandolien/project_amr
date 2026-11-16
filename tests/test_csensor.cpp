// CSensor 테스트: 2D/3D 라이다. 상자/벽까지의 거리를 해석해와 비교하고, 가림·범위·잡음·자세·예외를 확인한다.
#include <math.h>
#include <stdio.h>

#include "CBase/Array.h"
#include "CBase/Math.h"
#include "CCollision/Bvh.h"
#include "CCollision/RobotCollision.h"
#include "CSensor/Lidar.h"
#include "CURDF/Urdf.h"

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

// 예제 Data/Object/samples/lidar_room.urdf: 방 안의 라이다 기둥. 벽/장애물까지의 거리를 해석해와 비교한다.
struct RoomScene {
    amr::urdf::Robot robot;
    amr::urdf::RobotMeshes meshes;
    Array<TriangleBvh> bvhs;
    Array<LidarTarget> targets;
    int lidarLink;
    bool ok;
    RoomScene() : lidarLink(-1), ok(false) {}
    // joints 자세에서의 라이다 센서 자세와 대상 목록(센서가 붙은 링크는 뺀다)을 만든다
    dhMat build(const amr::urdf::JointValues& q) {
        const Array<dhMat> link = robot.LinkTransforms(q);
        bvhs.clear();
        targets.clear();
        struct Src { const Primitive* prim; dhMat world; int link; };
        Array<Src> srcs;
        for (size_t i = 0; i < meshes.visuals.size(); i++) {
            const amr::urdf::LinkVisual& lv = meshes.visuals[i];
            if (lv.link == lidarLink) continue;
            const Array<dhMat> nodeWorld = lv.scene.WorldTransforms();
            for (size_t n = 0; n < lv.scene.nodes.size(); n++) {
                const SceneNode& node = lv.scene.nodes[n];
                if (node.mesh < 0) continue;
                const Mesh& mesh = lv.scene.meshes[static_cast<size_t>(node.mesh)];
                for (size_t k = 0; k < mesh.primitives.size(); k++) {
                    Src sr;
                    sr.prim = &mesh.primitives[k];
                    sr.world = link[static_cast<size_t>(lv.link)] * lv.local * nodeWorld[n];
                    sr.link = lv.link;
                    srcs.push_back(sr);
                }
            }
        }
        bvhs.resize(srcs.size());
        for (size_t i = 0; i < srcs.size(); i++)
            bvhs[i].Build(srcs[i].prim->positions.data(), srcs[i].prim->VertexCount(), srcs[i].prim->indices.data(), srcs[i].prim->indices.size());
        for (size_t i = 0; i < srcs.size(); i++) {
            LidarTarget t;
            t.positions = srcs[i].prim->positions.data();
            t.vertexCount = srcs[i].prim->VertexCount();
            t.indices = srcs[i].prim->indices.data();
            t.indexCount = srcs[i].prim->indices.size();
            t.world = srcs[i].world;
            t.bvh = &bvhs[i];
            t.id = srcs[i].link;
            targets.push_back(t);
        }
        return link[static_cast<size_t>(lidarLink)];
    }
};

void testLidarRoomExample() {
    static RoomScene room;  // 메시를 가리키는 포인터가 있으므로 이동하지 않는 곳에 둔다
    String err;
    const String path = String(AMR_TEST_DATA_DIR) + "/Object/samples/lidar_room.urdf";
    if (!urdf::LoadUrdf(path, &room.robot, &err)) {
        printf("FAIL: cannot load %s (%s)\n", path.c_str(), err.c_str());
        ++g_fail;
        return;
    }
    urdf::LoadOptions opt;
    check(urdf::LoadRobotMeshes(room.robot, opt, &room.meshes), "room: all visuals build");
    room.lidarLink = room.robot.FindLink("lidar_link");
    check(room.lidarLink >= 0, "room: lidar_link exists");
    if (room.lidarLink < 0) return;
    {   // 충돌: 처음 자세와 방 안 어디서든(장애물 밖) 충돌이 없어야 하고, 장애물 안으로 들어가면 충돌로 나와야 한다
        CollisionModelOptions copt;
        copt.fallbackToVisual = true;  // 뷰어와 같은 설정
        RobotCollisionModel cm;
        check(BuildCollisionModel(room.robot, copt, &cm), "room: collision model builds");
        urdf::JointValues q;
        check(CheckSelfCollisions(room.robot, cm, q).collisions.empty(), "room: no collision at the start pose");
        q["x_slide"] = -1.0f; q["y_slide"] = 0.5f; q["yaw"] = 0.5f;
        check(CheckSelfCollisions(room.robot, cm, q).collisions.empty(), "room: no collision in free space");
        q["x_slide"] = 3.6f; q["y_slide"] = -2.6f;  // 관절 한계의 모서리 (벽 안쪽 여유가 있다)
        check(CheckSelfCollisions(room.robot, cm, q).collisions.empty(), "room: no collision at the joint-limit corner");
        q["x_slide"] = 2.0f; q["y_slide"] = 1.2f;   // 상자 안
        check(!CheckSelfCollisions(room.robot, cm, q).collisions.empty(), "room: pillar inside the box is a collision");
    }
    check(room.robot.lidars.size() == 1 && room.robot.lidars[0].link == "lidar_link", "room: URDF declares one lidar on lidar_link");
    if (room.robot.lidars.size() == 1) {
        const urdf::LidarDef& d = room.robot.lidars[0];
        check(d.hSamples == 541 && d.vSamples == 1 && near(d.hMax, 2.35619449, 1e-4) && d.rangeMax == 30.0f, "room: URDF lidar settings");
    }

    LidarSettings s;
    s.angleMin = static_cast<float>(-dhPI);
    s.angleMax = static_cast<float>(dhPI);
    s.angleIncrement = static_cast<float>(dhPI / 2.0);  // 광선 5 개: -180, -90, 0, 90, 180 도 (방위 번호 0..4)
    Lidar lidar(s);
    LidarScan scan;
    urdf::JointValues q;
    // 방 한가운데 (0, 0), yaw 0: 동서남북 벽까지. (y = 0 선 위에는 장애물이 없다. 낮은 상자는 라이다 평면 아래라 안 보인다.)
    dhMat pose = room.build(q);
    check(near(pose.O().z, 1.05, 1e-4), "room: lidar height is 1.05 m");
    lidar.Scan(pose, room.targets.data(), room.targets.size(), &scan);
    check(scan.size() == 5, "room: 5 rays");
    check(scan.IsHit(2) && near(scan.ranges[2], 4.0, 1e-3), "room: +x wall at 4.0 m");
    check(scan.IsHit(3) && near(scan.ranges[3], 3.0, 1e-3), "room: +y wall at 3.0 m");
    check(scan.IsHit(0) && near(scan.ranges[0], 4.0, 1e-3), "room: -x wall at 4.0 m (-180 deg)");
    check(scan.IsHit(1) && near(scan.ranges[1], 3.0, 1e-3), "room: -y wall at 3.0 m");

    // y 를 1.2 로 올리면 +x 방향에는 상자(x 1.4 ~ 2.6, y 0.8 ~ 1.6)가 있다: 1.4 m
    q["y_slide"] = 1.2f;
    lidar.Scan(room.build(q), room.targets.data(), room.targets.size(), &scan);
    check(near(scan.ranges[2], 1.4, 1e-3), "room: box face at 1.4 m");
    check(scan.targetIds[2] == room.robot.FindLink("room"), "room: the box belongs to the room link");

    // 원기둥(중심 -2.2, -1.3, 반지름 0.35): y = -1.3 에서 -x 방향으로 겨눈다. 면이 N 각형이라 약간의 차이를 허용.
    q["y_slide"] = -1.3f;
    lidar.Scan(room.build(q), room.targets.data(), room.targets.size(), &scan);
    check(near(scan.ranges[0], 1.85, 0.02), "room: column surface at about 1.85 m");

    // 기둥을 돌리면 (yaw 90 도) 같은 위치에서 방위 0 이 +y 벽을 향한다
    q["y_slide"] = 0.0f;
    q["yaw"] = static_cast<float>(dhPI / 2);
    lidar.Scan(room.build(q), room.targets.data(), room.targets.size(), &scan);
    check(near(scan.ranges[2], 3.0, 1e-3), "room: yaw 90 deg: azimuth 0 faces the +y wall");

    // 낮은 상자(중심 1.5, -1.5; 높이 0.4)는 수평 2D 에서는 안 보이지만, 아래를 보는 3D 채널에서는 보인다.
    // 센서 (1.5, 0.5) 에서 -y 방향, 고각 -18 도: 상자 윗면(z 0.4)에 0.65 / sin(18 도) = 2.10 m 거리, y = -1.5 에서 닿는다.
    // (너무 가파르면 센서 바로 아래의 기둥 윗면(반지름 0.12, 센서보다 0.05 아래)에 먼저 맞는다. 실제 3D 라이다의 사각지대와 같다.)
    q["x_slide"] = 1.5f;
    q["y_slide"] = 0.5f;
    q["yaw"] = static_cast<float>(-dhPI / 2);
    LidarSettings down;
    down.angleMin = down.angleMax = 0.0f;
    down.elevationMin = static_cast<float>(-18.0 * dhPI / 180);
    Lidar l3(down);
    l3.Scan(room.build(q), room.targets.data(), room.targets.size(), &scan);
    check(scan.IsHit(0) && near(scan.ranges[0], 0.65 / sin(18.0 * dhPI / 180), 5e-3), "room: the low box is seen by a downward-looking channel");
    check(near(scan.points[0].z, 0.4, 5e-3) && near(scan.points[0].y, -1.5, 5e-3), "room: that hit point is on the box top (z 0.4, y -1.5)");
    // 같은 센서의 수평 2D 스캔은 낮은 상자 위를 지나 -y 벽(y = -3)에 닿는다: 0.5 + 3 = 3.5 m
    LidarSettings fwd = s;
    fwd.angleMin = fwd.angleMax = 0.0f;
    Lidar l2(fwd);
    LidarScan flat;
    l2.Scan(room.build(q), room.targets.data(), room.targets.size(), &flat);
    check(near(flat.ranges[0], 3.5, 1e-3), "room: the horizontal 2D scan passes over the low box (reaches the wall at 3.5 m)");
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
    testLidarRoomExample();
    testEdgeCases();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_csensor: all passed\n");
    return 0;
}
