// CCollision 테스트: Aabb 연산, 변환, 도형별 AABB, 브로드페이즈(브루트포스 대조), 로봇 자기충돌
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "CCollision/Aabb.h"
#include "CCollision/Broadphase.h"
#include "CCollision/RobotCollision.h"
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

}  // namespace

int main() {
    testAabbBasics();
    testTransformed();
    testRaycast();
    testGeometryAabb();
    testBroadphase();
    testRobot();
    testMeshCollision();
    if (g_fail == 0) std::printf("test_ccollision: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
