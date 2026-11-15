// CCollision 테스트: Aabb 연산, 변환, 도형별 AABB, 브로드페이즈(브루트포스 대조), 로봇 자기충돌, OBB(SAT, 기준 구현 대조), 2단계 충돌
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "CCollision/Aabb.h"
#include "CCollision/Broadphase.h"
#include "CCollision/MeshRaycast.h"
#include "CCollision/Obb.h"
#include "CCollision/RobotCollision.h"
#include "CMesh/MeshLoader.h"
#include "CMesh/Primitives.h"
#include "CURDF/Urdf.h"
#include "TestUtil.h"

using namespace amr;

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-5) { return std::fabs(a - b) <= eps; }

bool boxIs(const Aabb& b, double x0, double y0, double z0, double x1, double y1, double z1, double eps = 1e-5) {
    return nearly(b.lo.x, x0, eps) && nearly(b.lo.y, y0, eps) && nearly(b.lo.z, z0, eps) && nearly(b.hi.x, x1, eps) &&
           nearly(b.hi.y, y1, eps) && nearly(b.hi.z, z1, eps);
}

// 재현 가능한 난수 (선형 합동 생성기)
struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    float next() {  // [0, 1)
        s = s * 1664525u + 1013904223u;
        return static_cast<float>((s >> 8) & 0xFFFF) / 65536.0f;
    }
    float range(float a, float b) { return a + (b - a) * next(); }
};

dhMat randomPose(Rng& r) {
    return dhMat::Trans(r.range(-2, 2), r.range(-2, 2), r.range(-2, 2)) * dhMat::RotZ(r.range(-3.14f, 3.14f)) *
           dhMat::RotY(r.range(-3.14f, 3.14f)) * dhMat::RotX(r.range(-3.14f, 3.14f));
}

// ---------------------------------------------------------------- Aabb 기본 연산
void testAabbBasics() {
    Aabb e;
    check(e.IsEmpty(), "default is empty");
    check(!e.Overlaps(e), "empty does not overlap itself");
    check(!e.Contains(dhVector(0, 0, 0)), "empty contains nothing");
    check(e.Size().x == 0 && e.Volume() == 0, "empty size is zero");
    check(e.Distance(Aabb(dhVector(0, 0, 0), dhVector(1, 1, 1))) < 0, "distance to empty is -1");

    Aabb b;
    b.Expand(dhVector(1, 2, 3));
    check(!b.IsEmpty(), "one point is a valid (degenerate) box");
    check(b.Contains(dhVector(1, 2, 3)), "degenerate box contains its point");
    b.Expand(dhVector(-1, 5, 0));
    check(boxIs(b, -1, 2, 0, 1, 5, 3), "expand");
    check(nearly(b.Volume(), 2 * 3 * 3), "volume");
    check(nearly(b.Center().y, 3.5) && nearly(b.HalfExtents().x, 1.0), "center / half extents");

    Aabb nanBox = b;
    nanBox.Expand(dhVector(NAN, 0, 0));
    check(boxIs(nanBox, -1, 2, 0, 1, 5, 3), "NaN point ignored");
    check(Aabb(dhVector(NAN, 0, 0), dhVector(1, 1, 1)).IsEmpty(), "NaN box is empty");

    // 병합: 빈 상자는 무시
    Aabb m;
    m.Merge(b);
    check(boxIs(m, -1, 2, 0, 1, 5, 3), "merge into empty");
    m.Merge(Aabb());
    check(boxIs(m, -1, 2, 0, 1, 5, 3), "merge empty is no-op");
    m.Merge(Aabb(dhVector(0, 0, 0), dhVector(4, 1, 1)));
    check(boxIs(m, -1, 0, 0, 4, 5, 3), "merge");

    // 겹침: 닿는 것도 겹침
    const Aabb a(dhVector(0, 0, 0), dhVector(1, 1, 1));
    check(a.Overlaps(Aabb(dhVector(1, 0, 0), dhVector(2, 1, 1))), "touching faces overlap");
    check(a.Overlaps(Aabb(dhVector(1, 1, 1), dhVector(2, 2, 2))), "touching corners overlap");
    check(!a.Overlaps(Aabb(dhVector(1.001f, 0, 0), dhVector(2, 1, 1))), "separated in x");
    check(!a.Overlaps(Aabb(dhVector(0, 0, 1.001f), dhVector(1, 1, 2))), "separated in z");
    check(a.Overlaps(a), "box overlaps itself");

    // 포함
    check(a.Contains(Aabb(dhVector(0.2f, 0.2f, 0.2f), dhVector(0.8f, 0.8f, 0.8f))), "contains inner box");
    check(!a.Contains(Aabb(dhVector(0.2f, 0.2f, 0.2f), dhVector(1.8f, 0.8f, 0.8f))), "does not contain sticking-out box");
    check(!a.Contains(Aabb()), "does not contain empty");
    check(a.Contains(dhVector(1, 1, 1)) && !a.Contains(dhVector(1.01f, 1, 1)), "contains point (closed interval)");

    // 거리
    check(nearly(a.Distance(Aabb(dhVector(3, 0, 0), dhVector(4, 1, 1))), 2.0), "distance along x");
    check(nearly(a.Distance(Aabb(dhVector(2, 2, 1), dhVector(3, 3, 2))), std::sqrt(2.0)), "distance diagonal");
    check(nearly(a.Distance(Aabb(dhVector(0.5f, 0.5f, 0.5f), dhVector(3, 3, 3))), 0.0), "distance when overlapping is 0");

    // 겹친 정도
    check(nearly(a.PenetrationDepth(Aabb(dhVector(0.7f, 0, 0), dhVector(3, 1, 1))), 0.3), "penetration = smallest axis overlap");
    check(a.PenetrationDepth(Aabb(dhVector(2, 0, 0), dhVector(3, 1, 1))) == 0.0f, "no penetration when apart");

    // 확장/축소
    check(boxIs(a.Inflated(0.5f), -0.5, -0.5, -0.5, 1.5, 1.5, 1.5), "inflate");
    check(a.Inflated(-0.6f).IsEmpty(), "shrinking past zero gives empty");
    check(Aabb().Inflated(1.0f).IsEmpty(), "inflating empty stays empty");
    check(boxIs(Aabb::FromCenterHalfExtents(dhVector(1, 2, 3), dhVector(1, 1, 2)), 0, 1, 1, 2, 3, 5), "from center/half extents");
}

// ---------------------------------------------------------------- 변환
void testTransformed() {
    Rng r(1234);
    // 8 개 꼭짓점을 모두 변환해서 구한 AABB 와 Transformed() 가 같아야 한다 (회전, 이동, 스케일 섞어서)
    for (int iter = 0; iter < 200; iter++) {
        Aabb b;
        b.Expand(dhVector(r.range(-2, 0), r.range(-2, 0), r.range(-2, 0)));
        b.Expand(dhVector(r.range(0, 2), r.range(0, 2), r.range(0, 2)));
        dhMat m = randomPose(r);
        if (iter % 2) m = m * dhMat::Scale(r.range(0.2f, 2), r.range(0.2f, 2), r.range(0.2f, 2));

        Aabb brute;
        for (int c = 0; c < 8; c++)
            brute.Expand(m * dhVector((c & 1) ? b.hi.x : b.lo.x, (c & 2) ? b.hi.y : b.lo.y, (c & 4) ? b.hi.z : b.lo.z));
        const Aabb t = b.Transformed(m);
        check(boxIs(t, brute.lo.x, brute.lo.y, brute.lo.z, brute.hi.x, brute.hi.y, brute.hi.z, 2e-4), "Transformed == brute force over corners");
    }
    // 90 도 회전: 축이 바뀐다
    const Aabb a(dhVector(0, 0, 0), dhVector(1, 2, 3));
    check(boxIs(a.Transformed(dhMat::RotZ(static_cast<float>(dhRad(90)))), -2, 0, 0, 0, 1, 3, 1e-5), "RotZ(90) swaps x,y extents");
    check(boxIs(a.Transformed(dhMat::Trans(1, 1, 1)), 1, 1, 1, 2, 3, 4), "translate");
    check(Aabb().Transformed(dhMat::Trans(1, 1, 1)).IsEmpty(), "transform of empty is empty");
}

// ---------------------------------------------------------------- 광선
void testRaycast() {
    const Aabb a(dhVector(1, -1, -1), dhVector(3, 1, 1));
    float t = -1;
    check(a.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 100, &t) && nearly(t, 1.0), "ray hits box from outside");
    check(a.Raycast(dhVector(0, 0, 0), dhVector(2, 0, 0), 100, &t) && nearly(t, 0.5), "non-unit direction scales t");
    check(!a.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 0.9f, &t), "tMax before the box");
    check(a.Raycast(dhVector(2, 0, 0), dhVector(0, 1, 0), 100, &t) && t == 0.0f, "origin inside gives t = 0");
    check(!a.Raycast(dhVector(0, 0, 0), dhVector(-1, 0, 0), 100, &t), "ray pointing away");
    check(!a.Raycast(dhVector(0, 5, 0), dhVector(1, 0, 0), 100, &t), "parallel ray outside the slab");
    check(a.Raycast(dhVector(0, 0.5f, 0.5f), dhVector(1, 0, 0), 100, &t), "parallel ray inside the slab");
    check(a.Raycast(dhVector(0, 3, 0), dhVector(1, -1, 0), 100, &t) && nearly(t, 2.0), "diagonal ray: hits (1,1,0) edge at t=2");
    check(!a.Raycast(dhVector(0, 0, 0), dhVector(0, 0, 0), 100, &t), "zero direction");
    check(!a.Raycast(dhVector(0, 0, 0), dhVector(NAN, 0, 0), 100, &t), "NaN direction");
    check(!Aabb().Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 100, &t), "empty box");
    check(a.Raycast(dhVector(0, 0, 0), dhVector(1, 0, 0), 100, NULL), "tHit may be NULL");
}

// ---------------------------------------------------------------- 도형별 AABB
void testGeometryAabb() {
    Rng r(99);
    urdf::Geometry box;
    box.type = urdf::kGeomBox;
    box.size[0] = 0.4f;
    box.size[1] = 0.2f;
    box.size[2] = 1.0f;
    check(boxIs(AabbOfGeometry(box, dhMat::Trans(1, 2, 3), NULL), 0.8, 1.9, 2.5, 1.2, 2.1, 3.5), "box at translation");

    urdf::Geometry sph;
    sph.type = urdf::kGeomSphere;
    sph.radius = 0.5f;
    check(boxIs(AabbOfGeometry(sph, dhMat::Trans(1, 0, 0) * dhMat::RotX(1.0f), NULL), 0.5, -0.5, -0.5, 1.5, 0.5, 0.5), "sphere ignores rotation");

    urdf::Geometry cyl;
    cyl.type = urdf::kGeomCylinder;
    cyl.radius = 0.1f;
    cyl.length = 1.0f;
    check(boxIs(AabbOfGeometry(cyl, dhMat(), NULL), -0.1, -0.1, -0.5, 0.1, 0.1, 0.5), "cylinder upright");
    // 90 도 눕히면 축이 X 로 간다
    check(boxIs(AabbOfGeometry(cyl, dhMat::RotY(static_cast<float>(dhRad(90))), NULL), -0.5, -0.1, -0.1, 0.5, 0.1, 0.1, 1e-5), "cylinder lying along x");

    // 해석적 값이 충분히 촘촘한 메시의 AABB 와 맞는지 (임의 자세). 해석값은 메시 값보다 작을 수 없고 아주 조금만 크다.
    const Primitive cylMesh = MakeCylinder(0.1f, 1.0f, 720);
    const Primitive sphMesh = MakeSphere(0.5f, 96, 48);
    for (int iter = 0; iter < 60; iter++) {
        const dhMat w = randomPose(r);
        const Aabb ac = AabbOfGeometry(cyl, w, NULL);
        const Aabb mc = AabbOfPrimitive(cylMesh, w);
        check(ac.Contains(mc.Inflated(-1e-5f)), "cylinder analytic AABB contains mesh AABB");
        check(ac.Inflated(-2e-4f).IsEmpty() || mc.Inflated(2e-4f).Contains(ac), "cylinder analytic AABB is tight");

        const Aabb as = AabbOfGeometry(sph, w, NULL);
        const Aabb ms = AabbOfPrimitive(sphMesh, w);
        check(as.Contains(ms.Inflated(-1e-5f)), "sphere AABB contains mesh AABB");
        check(ms.Inflated(0.01f).Contains(as), "sphere AABB is tight");
    }

    // 메시: 로컬 AABB 를 변환한 값
    urdf::Geometry mesh;
    mesh.type = urdf::kGeomMesh;
    const Aabb local(dhVector(0, 0, 0), dhVector(2, 2, 2));
    check(boxIs(AabbOfGeometry(mesh, dhMat::Trans(1, 0, 0), &local), 1, 0, 0, 3, 2, 2), "mesh: local bounds transformed");
    check(AabbOfGeometry(mesh, dhMat(), NULL).IsEmpty(), "mesh without local bounds is empty");
    urdf::Geometry none;
    check(AabbOfGeometry(none, dhMat(), NULL).IsEmpty(), "no geometry is empty");

    // 장면
    const Scene sc = SceneFromPrimitive(MakeBox(2, 2, 2), "b");
    check(boxIs(AabbOfScene(sc, dhMat::Trans(5, 0, 0)), 4, -1, -1, 6, 1, 1), "scene bounds with transform");
    check(AabbOfScene(Scene(), dhMat()).IsEmpty(), "empty scene");
}

// ---------------------------------------------------------------- 브로드페이즈
std::vector<CollisionPair> bruteCross(const std::vector<Aabb>& a, const std::vector<Aabb>& b) {
    std::vector<CollisionPair> out;
    for (size_t i = 0; i < a.size(); i++)
        for (size_t j = 0; j < b.size(); j++)
            if (a[i].Overlaps(b[j])) out.push_back(CollisionPair(static_cast<int>(i), static_cast<int>(j)));
    return out;
}

Aabb randomBox(Rng& r, float span, float maxSize) {
    const dhVector c(r.range(0, span), r.range(0, span), r.range(0, span));
    const dhVector h(r.range(0, maxSize), r.range(0, maxSize), r.range(0, maxSize));
    return Aabb::FromCenterHalfExtents(c, h);
}

void testBroadphase() {
    check(FindOverlappingPairs(std::vector<Aabb>()).empty(), "no boxes, no pairs");

    for (unsigned seed = 1; seed <= 20; seed++) {
        Rng r(seed);
        std::vector<Aabb> a, b;
        const int na = 1 + static_cast<int>(r.next() * 120), nb = 1 + static_cast<int>(r.next() * 120);
        for (int i = 0; i < na; i++) a.push_back(r.next() < 0.1f ? Aabb() : randomBox(r, 10, 1.5f));
        for (int i = 0; i < nb; i++) b.push_back(r.next() < 0.1f ? Aabb() : randomBox(r, 10, 1.5f));
        if (seed % 3 == 0) {  // 면이 정확히 닿는 경우를 섞는다
            a.push_back(Aabb(dhVector(20, 0, 0), dhVector(21, 1, 1)));
            a.push_back(Aabb(dhVector(21, 0, 0), dhVector(22, 1, 1)));
            a.push_back(a[a.size() - 1]);  // 완전히 같은 상자
        }
        check(FindOverlappingPairs(a) == FindOverlappingPairsBruteForce(a), "sweep and prune == brute force (one set)");
        check(FindOverlappingPairs(a, b) == bruteCross(a, b), "sweep and prune == brute force (two sets)");
    }

    // 정렬 순서와 a < b
    std::vector<Aabb> boxes;
    boxes.push_back(Aabb(dhVector(0, 0, 0), dhVector(2, 2, 2)));
    boxes.push_back(Aabb(dhVector(5, 5, 5), dhVector(6, 6, 6)));
    boxes.push_back(Aabb(dhVector(1, 1, 1), dhVector(3, 3, 3)));
    boxes.push_back(Aabb());
    const std::vector<CollisionPair> p = FindOverlappingPairs(boxes);
    check(p.size() == 1 && p[0] == CollisionPair(0, 2), "pair indices refer to the input order, a < b");

    // x 에서는 겹치지만 y 에서 떨어진 상자는 쌍이 아니다
    boxes.clear();
    boxes.push_back(Aabb(dhVector(0, 0, 0), dhVector(2, 1, 1)));
    boxes.push_back(Aabb(dhVector(1, 2, 0), dhVector(3, 3, 1)));
    check(FindOverlappingPairs(boxes).empty(), "overlap in x only is not a pair");

    // 두 집합: 같은 집합 안의 겹침은 무시, 인덱스는 각자의 집합 기준
    std::vector<Aabb> s1, s2;
    s1.push_back(Aabb(dhVector(0, 0, 0), dhVector(2, 2, 2)));
    s1.push_back(Aabb(dhVector(1, 1, 1), dhVector(3, 3, 3)));  // s1[0] 과 겹치지만 같은 집합
    s2.push_back(Aabb(dhVector(10, 10, 10), dhVector(11, 11, 11)));
    s2.push_back(Aabb(dhVector(2.5f, 2.5f, 2.5f), dhVector(4, 4, 4)));
    const std::vector<CollisionPair> q = FindOverlappingPairs(s1, s2);
    check(q.size() == 1 && q[0] == CollisionPair(1, 1), "two-set pairs use per-set indices");
}

// ---------------------------------------------------------------- 로봇
const char* kChain =
    "<robot name='chain'>"
    " <link name='base'><collision><origin xyz='0 0 0.1'/><geometry><box size='0.2 0.2 0.2'/></geometry></collision></link>"
    " <link name='link1'><collision><origin xyz='0 0 0.2'/><geometry><box size='0.1 0.1 0.4'/></geometry></collision></link>"
    " <link name='link2'><collision><origin xyz='0 0 0.25'/><geometry><cylinder radius='0.05' length='0.6'/></geometry></collision></link>"
    " <joint name='j1' type='revolute'><parent link='base'/><child link='link1'/><origin xyz='0 0 0.2'/><axis xyz='0 1 0'/>"
    "   <limit lower='-3.2' upper='3.2' effort='1' velocity='1'/></joint>"
    " <joint name='j2' type='revolute'><parent link='link1'/><child link='link2'/><origin xyz='0 0 0.4'/><axis xyz='0 1 0'/>"
    "   <limit lower='-3.2' upper='3.2' effort='1' velocity='1'/></joint>"
    "</robot>";

bool hasPair(const std::vector<CollisionPair>& v, int a, int b) {
    for (size_t i = 0; i < v.size(); i++)
        if (v[i] == CollisionPair(a, b)) return true;
    return false;
}

void testRobot() {
    urdf::Robot robot;
    std::string err;
    check(urdf::LoadUrdfFromString(kChain, &robot, &err), "chain urdf loads");
    RobotCollisionModel model;
    check(BuildCollisionModel(robot, CollisionModelOptions(), &model) && model.shapes.size() == 3, "3 collision shapes");

    const int base = robot.FindLink("base"), l1 = robot.FindLink("link1"), l2 = robot.FindLink("link2");

    // 곧게 선 자세
    urdf::JointValues q;
    std::vector<Aabb> boxes = ComputeLinkAabbs(robot, model, q);
    check(boxes.size() == 3, "one AABB per link");
    check(boxIs(boxes[static_cast<size_t>(base)], -0.1, -0.1, 0, 0.1, 0.1, 0.2), "base AABB");
    check(boxIs(boxes[static_cast<size_t>(l1)], -0.05, -0.05, 0.2, 0.05, 0.05, 0.6), "link1 AABB");
    check(boxIs(boxes[static_cast<size_t>(l2)], -0.05, -0.05, 0.55, 0.05, 0.05, 1.15), "link2 AABB");
    check(FindSelfCollisions(robot, boxes).empty(), "straight pose: no self collision (adjacent links skipped)");

    SelfCollisionOptions all;
    all.ignoreAdjacent = false;
    std::vector<CollisionPair> pa = FindSelfCollisions(robot, boxes, all);
    check(hasPair(pa, l1, l2), "adjacent links are reported when not ignored");

    // margin: 곧은 자세에서 base 와 link2 사이 간격은 0.35
    SelfCollisionOptions near;
    near.margin = 0.3f;
    check(FindSelfCollisions(robot, boxes, near).empty(), "margin 0.3 < gap 0.35");
    near.margin = 0.4f;
    check(hasPair(FindSelfCollisions(robot, boxes, near), std::min(base, l2), std::max(base, l2)), "margin 0.4 > gap 0.35");

    // j1 = +90 도: link1 이 +x 로 눕는다 (회전축 y, z -> x)
    q["j1"] = static_cast<float>(dhRad(90));
    boxes = ComputeLinkAabbs(robot, model, q);
    check(boxIs(boxes[static_cast<size_t>(l1)], 0.0, -0.05, 0.15, 0.4, 0.05, 0.25, 1e-4), "link1 AABB after j1 = 90 deg");

    // j2 = 180 도: link2 가 접혀 내려와 base 와 겹친다
    q.clear();
    q["j2"] = static_cast<float>(dhRad(180));
    boxes = ComputeLinkAabbs(robot, model, q);
    std::vector<CollisionPair> col = FindSelfCollisions(robot, boxes);
    check(col.size() == 1 && col[0] == CollisionPair(std::min(base, l2), std::max(base, l2)), "folded pose: base-link2 collide");

    SelfCollisionOptions ign;
    ign.ignoredPairs.push_back(std::make_pair(std::string("link2"), std::string("base")));  // 순서 무관
    check(FindSelfCollisions(robot, boxes, ign).empty(), "ignored pair is skipped");

    // 관절 제한: 허용 범위를 넘는 값은 제한된다
    q.clear();
    q["j2"] = 10.0f;  // 제한 3.2 rad
    boxes = ComputeLinkAabbs(robot, model, q, true);
    const std::vector<Aabb> unclamped = ComputeLinkAabbs(robot, model, q, false);
    check(!(boxIs(boxes[static_cast<size_t>(l2)], unclamped[static_cast<size_t>(l2)].lo.x, unclamped[static_cast<size_t>(l2)].lo.y,
                  unclamped[static_cast<size_t>(l2)].lo.z, unclamped[static_cast<size_t>(l2)].hi.x,
                  unclamped[static_cast<size_t>(l2)].hi.y, unclamped[static_cast<size_t>(l2)].hi.z, 1e-3)),
          "clampToLimits changes the result");

    // 충돌 도형이 없는 링크는 빈 상자
    urdf::Robot r2;
    check(urdf::LoadUrdfFromString(
              "<robot name='r'><link name='a'/><link name='b'><visual><geometry><sphere radius='0.1'/></geometry></visual></link>"
              "<joint name='j' type='fixed'><parent link='a'/><child link='b'/></joint></robot>",
              &r2, &err),
          "visual-only urdf loads");
    RobotCollisionModel m2;
    check(BuildCollisionModel(r2, CollisionModelOptions(), &m2) && m2.shapes.empty(), "no <collision>: no shapes by default");
    std::vector<Aabb> b2 = ComputeLinkAabbs(r2, m2, urdf::JointValues());
    check(b2.size() == 2 && b2[0].IsEmpty() && b2[1].IsEmpty(), "links without shapes have empty AABBs");
    CollisionModelOptions fb;
    fb.fallbackToVisual = true;
    check(BuildCollisionModel(r2, fb, &m2) && m2.shapes.size() == 1 && m2.shapes[0].fromVisual, "fallbackToVisual uses <visual>");
    // <collision> 이 있는 링크는 fallback 을 쓰지 않는다
    check(BuildCollisionModel(robot, fb, &model) && model.shapes.size() == 3 && !model.shapes[0].fromVisual, "fallback only for links without <collision>");
}

// ---------------------------------------------------------------- 메시 충돌 도형
void testMeshCollision() {
    const std::string dir = tu::Tmp("amr_collision_test");
    tu::MkDir(dir);
    const std::string objPath = dir + "/cube.obj";
    {
        std::ofstream f(objPath.c_str());
        f << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n"
          << "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n";
    }
    const char* xml =
        "<robot name='m'><link name='a'><collision><origin xyz='1 0 0'/>"
        "<geometry><mesh filename='cube.obj' scale='2 2 2'/></geometry></collision></link></robot>";
    urdf::Robot robot;
    std::string err;
    check(urdf::LoadUrdfFromString(xml, &robot, &err, dir), "mesh urdf loads");
    RobotCollisionModel model;
    check(BuildCollisionModel(robot, CollisionModelOptions(), &model) && model.shapes.size() == 1, "mesh collision model builds");
    check(boxIs(model.shapes[0].meshBounds, 0, 0, 0, 2, 2, 2), "mesh bounds include URDF scale");
    const std::vector<Aabb> boxes = ComputeLinkAabbs(robot, model, urdf::JointValues());
    check(boxIs(boxes[0], 1, 0, 0, 3, 2, 2), "mesh link AABB includes origin");

    // 파일이 없으면 경고하고 false, 나머지는 계속
    urdf::Robot bad;
    check(urdf::LoadUrdfFromString(
              "<robot name='m'><link name='a'><collision><geometry><mesh filename='nope.obj'/></geometry></collision>"
              "<collision><geometry><box size='1 1 1'/></geometry></collision></link></robot>",
              &bad, &err, dir),
          "urdf with missing mesh loads");
    RobotCollisionModel mb;
    check(!BuildCollisionModel(bad, CollisionModelOptions(), &mb), "missing mesh -> false");
    check(mb.shapes.size() == 1 && mb.warnings.size() == 1, "other shapes are kept, one warning");

    std::remove(objPath.c_str());
    tu::RmDir(dir);
}


// ---------------------------------------------------------------- OBB
Obb randomObb(Rng& r, float span, float maxHalf) {
    const dhVector h(r.range(0.0f, maxHalf), r.range(0.0f, maxHalf), r.range(0.0f, maxHalf));
    const Aabb box = Aabb::FromCenterHalfExtents(dhVector(0, 0, 0), h);
    dhMat m = dhMat::Trans(r.range(0, span), r.range(0, span), r.range(0, span)) * dhMat::RotZ(r.range(-3.14f, 3.14f)) *
              dhMat::RotY(r.range(-3.14f, 3.14f)) * dhMat::RotX(r.range(-3.14f, 3.14f));
    return Obb::FromAabb(box).Transformed(m);
}

// SAT 구현과 독립적인 기준 구현: 꼭짓점을 15개 축에 직접 투영해서 구간이 겹치는지 본다 (double)
bool refProject(const Obb& a, const Obb& b, const double L[3]) {
    dhVector ca[8], cb[8];
    a.Corners(ca);
    b.Corners(cb);
    double amin = 1e300, amax = -1e300, bmin = 1e300, bmax = -1e300;
    for (int i = 0; i < 8; i++) {
        const double pa = ca[i].x * L[0] + ca[i].y * L[1] + ca[i].z * L[2];
        const double pb = cb[i].x * L[0] + cb[i].y * L[1] + cb[i].z * L[2];
        amin = std::min(amin, pa); amax = std::max(amax, pa);
        bmin = std::min(bmin, pb); bmax = std::max(bmax, pb);
    }
    return !(amax < bmin || bmax < amin);
}

bool refOverlap(const Obb& a, const Obb& b) {
    std::vector<double> axes;
    const dhVector* av = a.axis;
    const dhVector* bv = b.axis;
    for (int i = 0; i < 3; i++) {
        axes.push_back(av[i].x); axes.push_back(av[i].y); axes.push_back(av[i].z);
        axes.push_back(bv[i].x); axes.push_back(bv[i].y); axes.push_back(bv[i].z);
    }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            const double c[3] = {static_cast<double>(av[i].y) * bv[j].z - static_cast<double>(av[i].z) * bv[j].y,
                                 static_cast<double>(av[i].z) * bv[j].x - static_cast<double>(av[i].x) * bv[j].z,
                                 static_cast<double>(av[i].x) * bv[j].y - static_cast<double>(av[i].y) * bv[j].x};
            const double n = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
            if (n < 1e-6) continue;  // 평행한 축은 외적이 0 이므로 건너뛴다
            axes.push_back(c[0] / n); axes.push_back(c[1] / n); axes.push_back(c[2] / n);
        }
    for (size_t k = 0; k + 2 < axes.size(); k += 3)
        if (!refProject(a, b, &axes[k])) return false;
    return true;
}

// 표본 점 기준 "확실히 겹침" 판정: A 안의 점 하나가 B 안에 있으면 겹친다
bool sampleOverlap(const Obb& a, const Obb& b) {
    for (int ix = -4; ix <= 4; ix++)
        for (int iy = -4; iy <= 4; iy++)
            for (int iz = -4; iz <= 4; iz++) {
                const float fx = static_cast<float>(ix) / 4.0f, fy = static_cast<float>(iy) / 4.0f, fz = static_cast<float>(iz) / 4.0f;
                const dhVector p = a.center + a.axis[0] * (a.half.x * fx) + a.axis[1] * (a.half.y * fy) + a.axis[2] * (a.half.z * fz);
                if (b.Contains(p)) return true;
            }
    return false;
}

void testObbBasics() {
    check(Obb().IsEmpty(), "default OBB is empty");
    check(!Obb().Overlaps(Obb()), "empty OBB overlaps nothing");
    check(Obb::FromAabb(Aabb()).IsEmpty(), "OBB from empty AABB is empty");

    const Aabb a(dhVector(0, 0, 0), dhVector(2, 4, 6));
    const Obb o = Obb::FromAabb(a);
    check(!o.IsEmpty() && nearly(o.Volume(), 48.0) && nearly(o.center.y, 2.0), "FromAabb: center and volume");
    check(boxIs(o.ToAabb(), 0, 0, 0, 2, 4, 6), "FromAabb -> ToAabb round trip");
    check(o.Contains(dhVector(1, 2, 3)) && o.Contains(dhVector(2, 4, 6)) && !o.Contains(dhVector(2.01f, 2, 3)), "Contains (closed)");
    check(boxIs(o.Inflated(1.0f).ToAabb(), -1, -1, -1, 3, 5, 7), "Inflated");
    check(o.Inflated(-2.0f).IsEmpty(), "shrinking past zero gives empty");

    // 회전하면 상자가 돈다: ToAabb 는 커지고 부피는 그대로
    const Obb rot = o.Transformed(dhMat::RotZ(static_cast<float>(dhRad(45))));
    check(nearly(rot.Volume(), 48.0, 1e-3), "rotation keeps the OBB volume");
    check(rot.ToAabb().Volume() > 48.0f * 1.2f, "rotated OBB's AABB is larger");
    check(nearly(rot.axis[0].x, std::cos(dhRad(45)), 1e-5) && nearly(rot.axis[0].y, std::sin(dhRad(45)), 1e-5), "axis follows the rotation");

    // 균일 스케일
    const Obb sc = o.Transformed(dhMat::Scale(2.0f));
    check(nearly(sc.half.x, 2.0) && nearly(sc.axis[0].x, 1.0), "uniform scale doubles the half extents");
    // 이동
    check(boxIs(o.Transformed(dhMat::Trans(1, 1, 1)).ToAabb(), 1, 1, 1, 3, 5, 7), "translation");
    check(Obb().Transformed(dhMat::Trans(1, 1, 1)).IsEmpty(), "transform of empty is empty");

    // 꼭짓점 8개가 모두 상자에 포함되고, AABB 는 꼭짓점의 AABB 와 같다
    Rng r(5);
    for (int iter = 0; iter < 100; iter++) {
        const Obb b = randomObb(r, 5, 2);
        dhVector c[8];
        b.Corners(c);
        Aabb brute;
        bool allIn = true;
        for (int i = 0; i < 8; i++) {
            brute.Expand(c[i]);
            allIn = allIn && b.Inflated(1e-4f).Contains(c[i]);
        }
        check(allIn, "corners are inside the OBB");
        const Aabb t = b.ToAabb();
        check(boxIs(t, brute.lo.x, brute.lo.y, brute.lo.z, brute.hi.x, brute.hi.y, brute.hi.z, 1e-4), "ToAabb == AABB of corners");
        // Transformed 는 축을 직교로 유지한다
        check(nearly(b.axis[0].Dot(b.axis[1]), 0.0, 1e-4) && nearly(b.axis[0].Dot(b.axis[2]), 0.0, 1e-4) &&
                  nearly(b.axis[1].Dot(b.axis[2]), 0.0, 1e-4) && nearly(b.axis[0].Norm(), 1.0, 1e-4),
              "axes stay orthonormal");
    }
}

void testObbOverlap() {
    // 기본 사례
    const Obb a = Obb::FromAabb(Aabb(dhVector(0, 0, 0), dhVector(1, 1, 1)));
    check(a.Overlaps(a), "box overlaps itself");
    check(a.Overlaps(Obb::FromAabb(Aabb(dhVector(1, 0, 0), dhVector(2, 1, 1)))), "touching faces overlap");
    check(!a.Overlaps(Obb::FromAabb(Aabb(dhVector(1.01f, 0, 0), dhVector(2, 1, 1)))), "separated along x");
    check(a.Overlaps(Obb::FromAabb(Aabb(dhVector(0.2f, 0.2f, 0.2f), dhVector(0.4f, 0.4f, 0.4f)))), "contained box overlaps");
    check(!a.Overlaps(Obb()), "empty never overlaps");

    // 평평한(두께 0) 상자
    const Obb flat = Obb::FromAabb(Aabb(dhVector(-1, -1, 0.5f), dhVector(2, 2, 0.5f)));
    check(a.Overlaps(flat), "zero-thickness box crossing the cube");
    check(!a.Overlaps(flat.Transformed(dhMat::Trans(0, 0, 1))), "zero-thickness box above the cube");

    // AABB 는 겹치지만 OBB 는 겹치지 않는 경우: 나란한 대각선 막대 두 개 (OBB 를 쓰는 이유)
    const Obb bar = Obb::FromAabb(Aabb(dhVector(-2, -0.05f, -0.05f), dhVector(2, 0.05f, 0.05f)));
    const dhMat diag = dhMat::RotZ(static_cast<float>(dhRad(45)));
    const Obb b1 = bar.Transformed(diag);
    const Obb b2 = bar.Transformed(dhMat::Trans(-0.2121f, 0.2121f, 0) * diag);  // 대각선에 수직으로 0.3 떨어뜨림
    check(b1.ToAabb().Overlaps(b2.ToAabb()), "diagonal bars: AABBs overlap");
    check(!b1.Overlaps(b2), "diagonal bars: OBBs do not overlap");
    const Obb b3 = bar.Transformed(dhMat::Trans(-0.035f, 0.035f, 0) * diag);  // 0.05 떨어뜨림 (두께 0.1 보다 작음)
    check(b1.Overlaps(b3), "diagonal bars: close enough to overlap");

    // 모서리-모서리로만 분리되는 경우 (면 법선 6개로는 분리를 못 찾고 외적 축이 필요하다)
    const Obb ex = Obb::FromAabb(Aabb(dhVector(-2, -0.1f, -0.1f), dhVector(2, 0.1f, 0.1f)));
    const Obb ey = Obb::FromAabb(Aabb(dhVector(-0.1f, -2, -0.1f), dhVector(0.1f, 2, 0.1f)));
    const dhMat tilt = dhMat::RotX(static_cast<float>(dhRad(45)));
    // ex 는 x 방향 막대, ey 를 x 축으로 45도 돌려서 위로 띄운다. 외적 축 x 방향... 아래에서 기준 구현과 일치하는지 본다.
    for (int k = 0; k < 12; k++) {
        const Obb e2 = ey.Transformed(dhMat::Trans(0, 0, 0.05f * static_cast<float>(k)) * tilt);
        check(ex.Overlaps(e2) == refOverlap(ex, e2), "crossing bars match the reference");
    }

    // 임의 상자 쌍: 기준 구현(꼭짓점 투영)과 표본 검사에 대조
    int overlaps = 0, separated = 0, crossOnly = 0;
    for (unsigned seed = 1; seed <= 40; seed++) {
        Rng r(seed * 7919u);
        for (int iter = 0; iter < 150; iter++) {
            const Obb p = randomObb(r, 4, 1.2f);
            const Obb q = randomObb(r, 4, 1.2f);
            const bool sat = p.Overlaps(q);
            const bool ref = refOverlap(p, q);
            // 정확히 닿는 경계 근처만 아니면 두 구현은 일치해야 한다 (SAT 는 반올림 오차 쪽으로 여유를 둔다)
            check(!ref || sat, "SAT never misses an overlap the reference sees");
            check(!sat || refOverlap(p.Inflated(1e-3f), q), "SAT overlap implies reference overlap (with tiny tolerance)");
            if (sampleOverlap(p, q) || sampleOverlap(q, p)) check(sat, "a shared sample point implies SAT overlap");
            check(p.Overlaps(q) == q.Overlaps(p), "overlap is symmetric");
            sat ? overlaps++ : separated++;
            // 면 법선 6개만으로는 겹친다고 나오는데 15축 전체로는 분리되는 경우를 센다
            if (!sat) {
                bool faceSeparated = false;
                const Obb* ob[2] = {&p, &q};
                for (int w = 0; w < 2 && !faceSeparated; w++)
                    for (int i = 0; i < 3 && !faceSeparated; i++) {
                        const double L[3] = {ob[w]->axis[i].x, ob[w]->axis[i].y, ob[w]->axis[i].z};
                        if (!refProject(p, q, L)) faceSeparated = true;
                    }
                if (!faceSeparated) crossOnly++;
            }
        }
    }
    check(overlaps > 500 && separated > 500, "random test covers both outcomes");
    check(crossOnly > 5, "random test includes pairs separated only by an edge-cross axis");

    // 같은 방향의 축(평행)에서도 분리를 놓치거나 거짓 분리를 만들지 않는다
    Rng r(42);
    for (int iter = 0; iter < 200; iter++) {
        const Obb p = randomObb(r, 3, 1.0f);
        const dhMat shift = dhMat::Trans(r.range(-2, 2), r.range(-2, 2), r.range(-2, 2));
        const Obb q = p.Transformed(shift);  // 같은 자세, 위치만 다름 (모든 외적 축이 0 에 가까움)
        check(p.Overlaps(q) == refOverlap(p, q) || p.Overlaps(q), "parallel boxes: no false separation");
        check(!p.Overlaps(q) || refOverlap(p.Inflated(1e-3f), q), "parallel boxes: no false overlap");
    }

    // NaN
    Obb nanBox = a;
    nanBox.center.x = NAN;
    check(!a.Overlaps(nanBox) && !nanBox.Overlaps(a), "NaN center never overlaps");
}

void testObbGeometry() {
    urdf::Geometry box;
    box.type = urdf::kGeomBox;
    box.size[0] = 0.4f;
    box.size[1] = 0.2f;
    box.size[2] = 1.0f;
    const dhMat w = dhMat::Trans(1, 2, 3) * dhMat::RotZ(static_cast<float>(dhRad(90)));
    const Obb o = ObbOfGeometry(box, w, NULL);
    check(nearly(o.center.x, 1.0) && nearly(o.center.y, 2.0) && nearly(o.half.x, 0.2) && nearly(o.half.z, 0.5), "box OBB");
    check(nearly(o.axis[0].y, 1.0, 1e-5), "box OBB axis follows RotZ(90)");
    // 같은 도형의 AABB 는 OBB 의 AABB 와 같다 (box 는 정확)
    const Aabb ab = AabbOfGeometry(box, w, NULL);
    const Aabb ob = o.ToAabb();
    check(boxIs(ob, ab.lo.x, ab.lo.y, ab.lo.z, ab.hi.x, ab.hi.y, ab.hi.z, 1e-5), "box: OBB's AABB == exact AABB");

    urdf::Geometry cyl;
    cyl.type = urdf::kGeomCylinder;
    cyl.radius = 0.1f;
    cyl.length = 1.0f;
    const Obb oc = ObbOfGeometry(cyl, dhMat::RotY(static_cast<float>(dhRad(90))), NULL);
    check(nearly(oc.half.z, 0.5) && nearly(oc.axis[2].x, 1.0, 1e-5), "cylinder OBB lies along x");
    // 원기둥 OBB 의 AABB 는 해석적 AABB 를 포함한다 (모서리 때문에 조금 더 크다)
    Rng r(3);
    for (int iter = 0; iter < 40; iter++) {
        const dhMat pose = randomPose(r);
        check(ObbOfGeometry(cyl, pose, NULL).ToAabb().Inflated(1e-4f).Contains(AabbOfGeometry(cyl, pose, NULL)), "cylinder OBB contains the exact AABB");
    }

    urdf::Geometry mesh;
    mesh.type = urdf::kGeomMesh;
    const Aabb local(dhVector(0, 0, 0), dhVector(2, 2, 2));
    check(nearly(ObbOfGeometry(mesh, dhMat::Trans(1, 0, 0), &local).center.x, 2.0), "mesh OBB from local bounds");
    check(ObbOfGeometry(mesh, dhMat(), NULL).IsEmpty(), "mesh without bounds is empty");
    check(ObbOfGeometry(urdf::Geometry(), dhMat(), NULL).IsEmpty(), "no geometry is empty");
}

// AABB 로 거른 뒤 OBB 로 다시 검사하는 로봇 자기충돌
const char* kBars =
    "<robot name='bars'>"
    " <link name='root'/>"
    " <link name='a'><collision><geometry><box size='4 0.1 0.1'/></geometry></collision></link>"
    " <link name='b'><collision><geometry><box size='4 0.1 0.1'/></geometry></collision></link>"
    " <joint name='ja' type='fixed'><parent link='root'/><child link='a'/><origin rpy='0 0 0.7853982'/></joint>"
    " <joint name='jb' type='prismatic'><parent link='root'/><child link='b'/><origin rpy='0 0 0.7853982'/><axis xyz='0 1 0'/>"
    "   <limit lower='-1' upper='1' effort='1' velocity='1'/></joint>"
    "</robot>";

void testTwoPhaseSelfCollision() {
    urdf::Robot robot;
    std::string err;
    check(urdf::LoadUrdfFromString(kBars, &robot, &err), "bars urdf loads");
    RobotCollisionModel model;
    check(BuildCollisionModel(robot, CollisionModelOptions(), &model) && model.shapes.size() == 2, "bars model");
    const int a = robot.FindLink("a"), b = robot.FindLink("b");
    const CollisionPair ab(std::min(a, b), std::max(a, b));

    // b 를 막대에 수직으로 0.3 띄움: AABB 는 겹치지만 OBB 는 떨어져 있다
    urdf::JointValues q;
    q["jb"] = 0.3f;
    SelfCollisionReport rep = CheckSelfCollisions(robot, model, q);
    check(rep.aabbCandidates.size() == 1 && rep.aabbCandidates[0] == ab, "AABB phase reports the pair");
    check(rep.collisions.empty() && rep.shapePairs.empty(), "OBB phase rejects it");
    check(rep.linkAabbs.size() == robot.links.size() && rep.shapeObbs.size() == model.shapes.size(), "report sizes");

    // 0.05 로 가까이: 진짜 겹침
    q["jb"] = 0.05f;
    rep = CheckSelfCollisions(robot, model, q);
    check(rep.collisions.size() == 1 && rep.collisions[0] == ab, "overlapping bars collide");
    check(rep.shapePairs.size() == 1, "one shape pair is the cause");

    // margin: 간격 = 0.3 - 0.1 = 0.2. margin 0.3 이면 충돌로 본다, 0.1 이면 아니다
    q["jb"] = 0.3f;
    SelfCollisionOptions opt;
    opt.margin = 0.1f;
    check(CheckSelfCollisions(robot, model, q, opt).collisions.empty(), "margin 0.1 < gap 0.2");
    opt.margin = 0.3f;
    check(CheckSelfCollisions(robot, model, q, opt).collisions.size() == 1, "margin 0.3 > gap 0.2");

    // 1.0 떨어져도 대각선 막대의 AABB 는 여전히 겹친다 (AABB 가 헐렁하다는 증거). OBB 는 정확히 걸러낸다.
    q["jb"] = 1.0f;
    rep = CheckSelfCollisions(robot, model, q);
    check(rep.aabbCandidates.size() == 1 && rep.collisions.empty(), "gap 0.9: AABB candidate, but no collision");

    // ignoredPairs 는 두 단계 모두에서 적용된다
    q["jb"] = 0.05f;
    SelfCollisionOptions ign;
    ign.ignoredPairs.push_back(std::make_pair(std::string("b"), std::string("a")));
    rep = CheckSelfCollisions(robot, model, q, ign);
    check(rep.aabbCandidates.empty() && rep.collisions.empty(), "ignored pair");

    // 링크에 도형이 여러 개이면 그중 하나만 겹쳐도 충돌: 막대 a 옆에 작은 상자를 단 링크
    urdf::Robot multi;
    check(urdf::LoadUrdfFromString(
              "<robot name='m'><link name='root'/>"
              " <link name='a'><collision><geometry><box size='4 0.1 0.1'/></geometry></collision></link>"
              " <link name='c'>"
              "  <collision><origin xyz='0 3 0'/><geometry><box size='0.2 0.2 0.2'/></geometry></collision>"
              "  <collision><origin xyz='0 0.05 0'/><geometry><box size='0.2 0.2 0.2'/></geometry></collision></link>"
              " <joint name='ja' type='fixed'><parent link='root'/><child link='a'/></joint>"
              " <joint name='jc' type='fixed'><parent link='root'/><child link='c'/></joint></robot>",
              &multi, &err),
          "multi-shape urdf loads");
    RobotCollisionModel mm;
    check(BuildCollisionModel(multi, CollisionModelOptions(), &mm) && mm.shapes.size() == 3, "multi-shape model");
    rep = CheckSelfCollisions(multi, mm, urdf::JointValues());
    check(rep.collisions.size() == 1 && rep.shapePairs.size() == 1, "one of two shapes is enough");
    // 닿은 도형은 c 의 두 번째 도형 (model.shapes 인덱스 2)
    check(rep.shapePairs.size() == 1 && rep.shapePairs[0].b == 2, "the reported shape is the one that overlaps");
}


// ---- 광선-삼각형 (화면 클릭 선택에 쓴다) 과 좌표계 OBJ
void testMeshRaycast() {
    // 한 변 2 의 정육면체 (정점 8개, 면 12개)
    const float cube[24] = {-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1};
    const uint32_t idx[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
    float t = -1;
    size_t tri = 99;
    check(RaycastTriangles(cube, 8, idx, 36, dhVector(5, 0.1f, 0.2f), dhVector(-1, 0, 0), 100, &t, &tri) && std::fabs(t - 4.0f) < 1e-5f && tri < 12,
          "ray hits the near face of the cube");
    check(RaycastTriangles(cube, 8, idx, 36, dhVector(0, 0, 0), dhVector(0, 0, 1), 100, &t) && std::fabs(t - 1.0f) < 1e-5f, "ray from inside hits the wall (both sides count)");
    check(!RaycastTriangles(cube, 8, idx, 36, dhVector(5, 3, 0), dhVector(-1, 0, 0), 100, &t), "ray beside the cube misses");
    check(!RaycastTriangles(cube, 8, idx, 36, dhVector(5, 0, 0), dhVector(1, 0, 0), 100, &t), "ray pointing away misses");
    check(!RaycastTriangles(cube, 8, idx, 36, dhVector(5, 0, 0), dhVector(-1, 0, 0), 3.5f, &t), "tMax shorter than the distance misses");
    check(RaycastTriangles(cube, 8, idx, 36, dhVector(5, 0, 0), dhVector(-2, 0, 0), 100, &t) && std::fabs(t - 2.0f) < 1e-5f, "t is in units of the direction length");
    // 위치+법선이 섞인 배열 (stride 6): 법선 자리에 엉뚱한 값을 넣어도 위치만 쓴다
    float mixed[48];
    for (int v = 0; v < 8; v++) {
        for (int k = 0; k < 3; k++) {
            mixed[v * 6 + k] = cube[v * 3 + k];
            mixed[v * 6 + 3 + k] = 777.0f;
        }
    }
    check(RaycastTriangles(mixed, 8, idx, 36, dhVector(5, 0.1f, 0.2f), dhVector(-1, 0, 0), 100, &t, nullptr, 6) && std::fabs(t - 4.0f) < 1e-5f, "stride 6 reads positions only");
    // 안전: 범위를 벗어난 인덱스, 퇴화 삼각형, NaN 은 건너뛴다
    const uint32_t bad[3] = {0, 1, 99};
    check(!RaycastTriangles(cube, 8, bad, 3, dhVector(0, 0, 5), dhVector(0, 0, -1), 100, &t), "out-of-range index is skipped");
    const float flat[9] = {0, 0, 0, 1, 0, 0, 2, 0, 0};
    const uint32_t one[3] = {0, 1, 2};
    check(!RaycastTriangles(flat, 3, one, 3, dhVector(1, 0, 5), dhVector(0, 0, -1), 100, &t), "degenerate triangle is skipped");
    check(!RaycastTriangles(cube, 8, idx, 36, dhVector(NAN, 0, 0), dhVector(-1, 0, 0), 100, &t), "NaN origin does not hit");
    check(!RaycastTriangles(nullptr, 0, nullptr, 0, dhVector(), dhVector(0, 0, 1), 1, &t), "null arrays");
    // 가장 가까운 것: 같은 광선 위의 두 겹
    const float two[18] = {-1, -1, 1, 1, -1, 1, 0, 1, 1, -1, -1, 3, 1, -1, 3, 0, 1, 3};
    const uint32_t i2[6] = {0, 1, 2, 3, 4, 5};
    size_t which = 99;
    check(RaycastTriangles(two, 6, i2, 6, dhVector(0, 0, 10), dhVector(0, 0, -1), 100, &t, &which) && std::fabs(t - 7.0f) < 1e-5f && which == 1, "nearest of two layers wins");
    // 무작위 광선: 같은 정육면체와 슬랩 방식(Aabb::Raycast)이 같은 결과를 내야 한다
    Aabb box;
    box.lo = dhVector(-1, -1, -1);
    box.hi = dhVector(1, 1, 1);
    unsigned rng = 12345;
    auto rnd = [&rng]() {
        rng = rng * 1664525u + 1013904223u;
        return static_cast<float>((rng >> 8) & 0xFFFF) / 65535.0f * 6.0f - 3.0f;
    };
    int agree = 0, total = 0, hits = 0;
    for (int i = 0; i < 2000; i++) {
        const dhVector o(rnd(), rnd(), rnd()), d(rnd(), rnd(), rnd());
        if (std::fabs(d.x) + std::fabs(d.y) + std::fabs(d.z) < 0.05f) continue;
        float ta = 0, tb = 0;
        const bool a = RaycastTriangles(cube, 8, idx, 36, o, d, 1e6f, &ta);
        const bool b = box.Raycast(o, d, 1e6f, &tb);
        // 상자 안에서 시작하면 슬랩은 t = 0 으로 맞고 삼각형은 벽에서 맞는다. 그 경우는 비교하지 않는다
        if (box.Contains(o)) continue;
        total++;
        hits += a ? 1 : 0;
        if (a == b && (!a || std::fabs(ta - tb) < 1e-3f * std::max(1.0f, tb))) agree++;
    }
    check(total > 1000 && hits > 100 && agree == total, "random rays agree with the slab test");
}

void testGizmoObj() {
    Scene g;
    std::string err;
    const std::string path = std::string(AMR_TEST_DATA_DIR) + "/Object/gizmo/axes.obj";
    check(LoadObj(path, &g, &err), "gizmo OBJ loads");
    check(g.warnings.empty(), "gizmo OBJ has no warnings (mtl found)");
    check(g.materials.size() == 4, "gizmo has center + x + y + z materials");
    // 전체 크기: 화살표 끝 = 1, 구 반지름 0.1
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    size_t tris = 0;
    for (size_t m = 0; m < g.meshes.size(); m++)
        for (size_t p = 0; p < g.meshes[m].primitives.size(); p++) {
            const Primitive& pr = g.meshes[m].primitives[p];
            check(pr.IsValid() && pr.normals.size() == pr.positions.size(), "gizmo primitive is valid and has normals");
            tris += pr.TriangleCount();
            for (size_t v = 0; v < pr.VertexCount(); v++)
                for (int k = 0; k < 3; k++) {
                    lo[k] = std::min(lo[k], pr.positions[v * 3 + static_cast<size_t>(k)]);
                    hi[k] = std::max(hi[k], pr.positions[v * 3 + static_cast<size_t>(k)]);
                }
        }
    check(tris > 500, "gizmo has real geometry");
    for (int k = 0; k < 3; k++) check(std::fabs(hi[k] - 1.0f) < 1e-4f && std::fabs(lo[k] + 0.1f) < 1e-4f, "gizmo spans -0.1 .. 1.0 on every axis");
    // 각 축 방향 광선이 그 축의 색 재질을 맞춘다 (X 빨강, Y 초록, Z 파랑), 가운데는 회색
    struct Probe { float o[3], d[3]; int dom; };  // dom: 가장 큰 색 성분 (0 r, 1 g, 2 b), -1 이면 회색
    const Probe probes[4] = {{{3, 0.01f, 0}, {-1, 0, 0}, 0}, {{0.01f, 3, 0}, {0, -1, 0}, 1}, {{0.01f, 0, 3}, {0, 0, -1}, 2}, {{0.3f, 0.01f, 0.4f}, {-0.6f, 0, -0.8f}, -1}};
    for (int pi = 0; pi < 4; pi++) {
        float best = 1e9f;
        int bestMat = -2;
        for (size_t m = 0; m < g.meshes.size(); m++)
            for (size_t p = 0; p < g.meshes[m].primitives.size(); p++) {
                const Primitive& pr = g.meshes[m].primitives[p];
                float t = 0;
                if (RaycastTriangles(pr.positions.data(), pr.VertexCount(), pr.indices.data(), pr.indices.size(), dhVector(probes[pi].o[0], probes[pi].o[1], probes[pi].o[2]),
                                     dhVector(probes[pi].d[0], probes[pi].d[1], probes[pi].d[2]), 100, &t) && t < best) {
                    best = t;
                    bestMat = pr.material;
                }
            }
        check(bestMat >= 0, "probe ray hits the gizmo");
        if (bestMat < 0) continue;
        const Color& c = g.materials[static_cast<size_t>(bestMat)].baseColor;
        int dom = (c.r > c.g && c.r > c.b) ? 0 : (c.g > c.r && c.g > c.b) ? 1 : (c.b > c.r && c.b > c.g) ? 2 : -1;
        if (std::fabs(c.r - c.g) < 0.1f && std::fabs(c.g - c.b) < 0.1f) dom = -1;
        check(dom == probes[pi].dom, "gizmo part has the expected color");
        if (probes[pi].dom >= 0) check(std::fabs(best - 2.0f) < 0.1f, "the arrow tip is at distance 1 from the origin");
    }
}

}  // namespace

int main() {
    testAabbBasics();
    testTransformed();
    testRaycast();
    testGeometryAabb();
    testBroadphase();
    testRobot();
    testMeshCollision();
    testObbBasics();
    testObbOverlap();
    testObbGeometry();
    testTwoPhaseSelfCollision();
    testMeshRaycast();
    testGizmoObj();
    if (g_fail == 0) std::printf("test_ccollision: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
