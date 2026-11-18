// Robot -> URDF 쓰기: 쓴 파일을 다시 읽으면 같은 구조가 나오는지(왕복), 여러 모델을 합친 장면도 하나의 URDF 로 저장되는지 확인
#include <math.h>
#include <stdio.h>

#include "CURDF/UrdfWriter.h"
#include "CURDF/Primitives.h"
#include "CURDF/World.h"
#include "TestUtil.h"

using namespace simul;
using namespace simul::urdf;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol = 1e-5) { return fabs(a - b) <= tol; }
bool nearMat(const dhMat& a, const dhMat& b, double tol = 1e-5) {
    for (int i = 0; i < 16; i++)
        if (!near(a.v[i], b.v[i], tol)) return false;
    return true;
}

const String kData = String(SIMUL_TEST_DATA_DIR) + "/Object/samples/";

void geomSame(const Geometry& a, const Geometry& b, const String& what) {
    check(a.type == b.type, (what + ": geometry type").c_str());
    check(near(a.size[0], b.size[0]) && near(a.size[1], b.size[1]) && near(a.size[2], b.size[2]), (what + ": box size").c_str());
    check(near(a.radius, b.radius) && near(a.length, b.length), (what + ": radius/length").c_str());
}

// 두 Robot 의 구조(링크, 관절, 질량/관성, 형상, 색)가 같은가
void sameRobot(const Robot& a, const Robot& b, const String& tag) {
    check(a.links.size() == b.links.size() && a.joints.size() == b.joints.size(), (tag + ": counts").c_str());
    if (a.links.size() != b.links.size() || a.joints.size() != b.joints.size()) return;
    for (size_t i = 0; i < a.links.size(); i++) {
        const Link& x = a.links[i];
        const int bi = b.FindLink(x.name);
        check(bi >= 0, (tag + ": link " + x.name).c_str());
        if (bi < 0) continue;
        const Link& y = b.links[static_cast<size_t>(bi)];
        check(x.hasInertial == y.hasInertial, (tag + ": hasInertial " + x.name).c_str());
        if (x.hasInertial && y.hasInertial) {
            check(near(x.inertial.mass, y.inertial.mass) && near(x.inertial.ixx, y.inertial.ixx) && near(x.inertial.iyy, y.inertial.iyy) && near(x.inertial.izz, y.inertial.izz) &&
                      near(x.inertial.ixy, y.inertial.ixy) && near(x.inertial.ixz, y.inertial.ixz) && near(x.inertial.iyz, y.inertial.iyz),
                  (tag + ": inertial " + x.name).c_str());
            check(nearMat(x.inertial.origin, y.inertial.origin), (tag + ": inertial origin " + x.name).c_str());
        }
        check(x.visuals.size() == y.visuals.size() && x.collisions.size() == y.collisions.size(), (tag + ": shapes " + x.name).c_str());
        for (size_t k = 0; k < x.visuals.size() && k < y.visuals.size(); k++) {
            geomSame(x.visuals[k].geometry, y.visuals[k].geometry, tag + " visual " + x.name);
            check(nearMat(x.visuals[k].origin, y.visuals[k].origin), (tag + ": visual origin " + x.name).c_str());
            check(x.visuals[k].hasMaterial == y.visuals[k].hasMaterial, (tag + ": has material " + x.name).c_str());
            if (x.visuals[k].hasMaterial && x.visuals[k].material.hasColor)
                check(y.visuals[k].material.hasColor && near(x.visuals[k].material.color.r, y.visuals[k].material.color.r) && near(x.visuals[k].material.color.b, y.visuals[k].material.color.b), (tag + ": color " + x.name).c_str());
        }
        for (size_t k = 0; k < x.collisions.size() && k < y.collisions.size(); k++) geomSame(x.collisions[k].geometry, y.collisions[k].geometry, tag + " collision " + x.name);
    }
    for (size_t i = 0; i < a.joints.size(); i++) {
        const Joint& x = a.joints[i];
        const int bi = b.FindJoint(x.name);
        check(bi >= 0, (tag + ": joint " + x.name).c_str());
        if (bi < 0) continue;
        const Joint& y = b.joints[static_cast<size_t>(bi)];
        check(x.type == y.type && x.parent == y.parent && x.child == y.child, (tag + ": joint wiring " + x.name).c_str());
        check(nearMat(x.origin, y.origin), (tag + ": joint origin " + x.name).c_str());
        if (x.type != kJointFixed) check(near(x.axis.x, y.axis.x) && near(x.axis.y, y.axis.y) && near(x.axis.z, y.axis.z), (tag + ": axis " + x.name).c_str());
        check(x.hasLimit == y.hasLimit, (tag + ": hasLimit " + x.name).c_str());
        if (x.hasLimit) check(near(x.lower, y.lower) && near(x.upper, y.upper) && near(x.effort, y.effort) && near(x.velocity, y.velocity), (tag + ": limit " + x.name).c_str());
        check(near(x.damping, y.damping) && near(x.friction, y.friction), (tag + ": dynamics " + x.name).c_str());
        check(x.hasMimic == y.hasMimic && (!x.hasMimic || (x.mimicJoint == y.mimicJoint && near(x.mimicMultiplier, y.mimicMultiplier) && near(x.mimicOffset, y.mimicOffset))), (tag + ": mimic " + x.name).c_str());
    }
    check(a.lidars.size() == b.lidars.size(), (tag + ": lidar count").c_str());
    check(b.links[static_cast<size_t>(b.root)].name == a.links[static_cast<size_t>(a.root)].name, (tag + ": root").c_str());
}

void roundTrip(const String& file) {
    Robot a, b;
    String err;
    check(LoadUrdf(file, &a, &err), ("load " + file).c_str());
    const String out = tu::Tmp("simul_write_rt.urdf");
    check(SaveUrdf(a, out, &err), ("save " + file).c_str());
    check(LoadUrdf(out, &b, &err), ("reload " + file + ": " + err).c_str());
    sameRobot(a, b, file);
}

// 임의 회전 -> rpy -> 회전: 어떤 자세도 그대로 돌아와야 한다 (짐벌 락 근처 포함)
void testRpy() {
    const float angles[] = {0.0f, 0.3f, -1.2f, 1.5707963f, -1.5707963f, 2.8f, 3.14f};
    for (size_t a = 0; a < 7; a++)
        for (size_t b = 0; b < 7; b++)
            for (size_t c = 0; c < 7; c++) {
                dhMat m = dhEuler(angles[c], angles[b], angles[a]).GetMat();   // yaw, pitch, roll
                double rpy[3];
                RpyFromMat(m, rpy);
                const dhMat back = dhEuler(static_cast<float>(rpy[2]), static_cast<float>(rpy[1]), static_cast<float>(rpy[0])).GetMat();
                if (!nearMat(m, back, 2e-3)) {
                    printf("FAIL: rpy roundtrip roll=%f pitch=%f yaw=%f\n", angles[a], angles[b], angles[c]);
                    ++g_fail;
                    return;
                }
            }
}

// 여러 모델 + 회전된 모델을 하나의 URDF 로 저장하고 다시 읽는다
void testScene() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err) && w.Add(kData + "arm_gripper.urdf", &err) && w.Add(kData + "lidar_room.urdf", &err), "add models");
    w.SetPlacement(1, dhMat::Quat(0.7f, dhVector(0.3f, 0.4f, 0.8f)));   // 비스듬히 돌린 두 번째 팔
    dhMat p = dhMat::Quat(0.4f, dhVector(0, 0, 1));
    p.v[12] = 2.0f; p.v[13] = -1.0f;
    w.SetPlacement(2, p);
    Robot scene, again;
    check(w.Build(LoadOptions(), &scene, &err), "build scene");
    const String out = tu::Tmp("simul_write_scene.urdf");
    check(SaveUrdf(scene, out, &err), "save scene");
    check(LoadUrdf(out, &again, &err), ("reload scene: " + err).c_str());
    sameRobot(scene, again, "scene");
    check(again.links[static_cast<size_t>(again.root)].name == "world", "scene root is world");
}

// 기본 도형(박스, 원통, 구): 관성이 공식대로 계산되고, 장면에 올려 URDF 로 저장했다 다시 읽어도 같다
void testPrimitives() {
    PrimitiveSpec box;
    box.type = kGeomBox;
    box.size[0] = 0.4f; box.size[1] = 0.6f; box.size[2] = 0.8f;
    box.mass = 12.0f;
    Robot b;
    String err;
    check(MakePrimitiveRobot("crate", box, &b, &err), "make box");
    check(near(b.links[0].inertial.ixx, 12.0 / 12.0 * (0.36 + 0.64)) && near(b.links[0].inertial.izz, 12.0 / 12.0 * (0.16 + 0.36)), "box inertia");
    PrimitiveSpec cyl;
    cyl.type = kGeomCylinder; cyl.radius = 0.1f; cyl.length = 1.0f; cyl.mass = 6.0f;
    Robot c;
    check(MakePrimitiveRobot("pipe", cyl, &c, &err), "make cylinder");
    check(near(c.links[0].inertial.izz, 0.5 * 6.0 * 0.01) && near(c.links[0].inertial.ixx, 6.0 / 12.0 * (3 * 0.01 + 1.0)), "cylinder inertia");
    PrimitiveSpec sph;
    sph.type = kGeomSphere; sph.radius = 0.5f; sph.mass = 5.0f;
    Robot sp;
    check(MakePrimitiveRobot("ball", sph, &sp, &err), "make sphere");
    check(near(sp.links[0].inertial.ixx, 0.4 * 5.0 * 0.25), "sphere inertia");
    PrimitiveSpec bad = box;
    bad.size[1] = 0.0f;
    Robot x;
    check(!MakePrimitiveRobot("bad", bad, &x, &err) && !err.empty(), "zero size is rejected");

    World w;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    check(w.AddRobot("crate", b, &err) && w.AddRobot("crate", b, &err) && w.AddRobot("pipe", c, &err) && w.AddRobot("ball", sp, &err), "add shapes to the world");
    check(w.entry(2).name == "crate_2", "duplicate shape name gets a number");
    dhMat p;
    p.v[12] = 1.0f; p.v[13] = 2.0f; p.v[14] = 0.4f;
    w.SetPlacement(1, p);
    Robot scene, again;
    check(w.Build(LoadOptions(), &scene, &err), "build scene with shapes");
    const String out = tu::Tmp("simul_write_shapes.urdf");
    check(SaveUrdf(scene, out, &err), "save scene with shapes");
    check(LoadUrdf(out, &again, &err), ("reload scene with shapes: " + err).c_str());
    sameRobot(scene, again, "shapes");
}

}  // namespace

int main() {
    testPrimitives();
    roundTrip(kData + "arm.urdf");
    roundTrip(kData + "arm_gripper.urdf");
    roundTrip(kData + "lidar_room.urdf");
    roundTrip(kData + "lidar_room_3d.urdf");
    testRpy();
    testScene();
    if (g_fail == 0) printf("test_curdf_write: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
