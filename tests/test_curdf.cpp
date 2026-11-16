// CURDF 테스트: 파싱, 검증, 순기구학(rpy 규약, 제한, mimic), 메시 경로 해석과 로딩
#include "CBase/File.h"
#include "CBase/Math.h"
#include <stdio.h>
#include <stdlib.h>

#include "CURDF/Urdf.h"
#include "TestGlb.h"
#include "TestUtil.h"

using namespace amr;
using namespace amr::urdf;

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-4) { return amr::Abs(a - b) <= eps; }

bool posIs(const dhMat& m, double x, double y, double z) {
    return nearly(m.v[12], x) && nearly(m.v[13], y) && nearly(m.v[14], z);
}

// 변환된 점
void apply(const dhMat& m, double x, double y, double z, double* o) {
    o[0] = m.v[0] * x + m.v[4] * y + m.v[8] * z + m.v[12];
    o[1] = m.v[1] * x + m.v[5] * y + m.v[9] * z + m.v[13];
    o[2] = m.v[2] * x + m.v[6] * y + m.v[10] * z + m.v[14];
}

const char* kArm =
    "<?xml version=\"1.0\"?>\n"
    "<robot name=\"arm\">\n"
    "  <material name=\"blue\"><color rgba=\"0 0 1 1\"/></material>\n"
    "  <link name=\"base\"><visual><geometry><box size=\"1 1 0.2\"/></geometry><material name=\"blue\"/></visual>"
    "<inertial><mass value=\"2.5\"/><inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"2\" iyz=\"0\" izz=\"3\"/></inertial></link>\n"
    "  <link name=\"l1\"/><link name=\"l2\"/><link name=\"l3\"/><link name=\"tool\"/><link name=\"finger\"/>\n"
    "  <joint name=\"j1\" type=\"revolute\"><parent link=\"base\"/><child link=\"l1\"/><origin xyz=\"0 0 0.5\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-1.6\" upper=\"1.6\" effort=\"10\" velocity=\"1\"/></joint>\n"
    "  <joint name=\"j2\" type=\"revolute\"><parent link=\"l1\"/><child link=\"l2\"/><origin xyz=\"1 0 0\"/>"
    "<axis xyz=\"0 2 0\"/><limit lower=\"-3\" upper=\"3\" effort=\"10\" velocity=\"1\"/></joint>\n"
    "  <joint name=\"j3\" type=\"prismatic\"><parent link=\"l2\"/><child link=\"l3\"/><origin xyz=\"1 0 0\"/>"
    "<axis xyz=\"1 0 0\"/><limit lower=\"0\" upper=\"0.8\" effort=\"10\" velocity=\"1\"/></joint>\n"
    "  <joint name=\"jt\" type=\"fixed\"><parent link=\"l3\"/><child link=\"tool\"/><origin xyz=\"0 0 0.1\" rpy=\"0 0 1.5707963\"/></joint>\n"
    "  <joint name=\"jf\" type=\"continuous\"><parent link=\"tool\"/><child link=\"finger\"/><axis xyz=\"0 0 1\"/>"
    "<mimic joint=\"j1\" multiplier=\"2\" offset=\"0.1\"/></joint>\n"
    "</robot>\n";

void testParse() {
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(kArm, &r, &err), "arm loads");
    check(r.name == "arm" && r.links.size() == 6 && r.joints.size() == 5, "counts");
    check(r.root >= 0 && r.links[static_cast<size_t>(r.root)].name == "base", "root is base");
    check(r.order.size() == 5, "order size");
    check(r.links[0].hasInertial && nearly(r.links[0].inertial.mass, 2.5) && nearly(r.links[0].inertial.izz, 3), "inertial");
    check(r.links[0].visuals.size() == 1 && r.links[0].visuals[0].geometry.type == kGeomBox, "box visual");
    check(r.links[0].visuals[0].material.hasColor && nearly(r.links[0].visuals[0].material.color.b, 1), "named material color resolved");
    const Joint j2 = r.joints[static_cast<size_t>(r.FindJoint("j2"))];
    check(nearly(j2.axis.y, 1) && nearly(j2.axis.x, 0), "axis normalized");
    check(r.FindLink("nope") == -1 && r.FindJoint("nope") == -1, "find missing");
    const amr::Array<int> act = r.ActuatedJoints();
    check(act.size() == 3, "actuated = j1 j2 j3 (not fixed, not mimic)");
    // order: 부모 joint 가 자식 joint 보다 먼저 와야 한다
    amr::Array<bool> seen(r.links.size(), false);
    seen[static_cast<size_t>(r.root)] = true;
    bool ordered = true;
    for (size_t k = 0; k < r.order.size(); k++) {
        const Joint& j = r.joints[static_cast<size_t>(r.order[k])];
        if (!seen[static_cast<size_t>(r.FindLink(j.parent))]) ordered = false;
        seen[static_cast<size_t>(r.FindLink(j.child))] = true;
    }
    check(ordered, "topological order");
}

void testKinematics() {
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(kArm, &r, &err), "arm loads (fk)");
    const int l1 = r.FindLink("l1"), l2 = r.FindLink("l2"), l3 = r.FindLink("l3"), tool = r.FindLink("tool"), fin = r.FindLink("finger");

    amr::Array<dhMat> T = r.LinkTransforms(JointValues());
    check(posIs(T[static_cast<size_t>(l1)], 0, 0, 0.5), "zero pose l1");
    check(posIs(T[static_cast<size_t>(l3)], 2, 0, 0.5), "zero pose l3");
    check(posIs(T[static_cast<size_t>(tool)], 2, 0, 0.6), "zero pose tool");

    JointValues q;
    q["j1"] = static_cast<float>(tu::kPi / 2);
    q["j2"] = static_cast<float>(tu::kPi / 2);
    q["j3"] = 0.5f;
    T = r.LinkTransforms(q);
    check(posIs(T[static_cast<size_t>(l2)], 0, 1, 0.5), "l2 after j1=90");
    check(posIs(T[static_cast<size_t>(l3)], 0, 1, -1.0), "l3 after j1=90, j2=90, j3=0.5 (hand-computed)");
    // l3 = Rz(90) Ry(90) 이므로 l3 의 z 축은 월드 +Y. 도구는 그 방향으로 0.1 이동 -> (0, 1.1, -1)
    check(posIs(T[static_cast<size_t>(tool)], 0, 1.1, -1.0), "tool offset follows l3 orientation");

    // 제한: j1 의 상한 1.6 으로 잘림, 끄면 그대로
    JointValues big;
    big["j1"] = 3.0f;
    check(nearly(r.EffectiveJointValue(big, r.FindJoint("j1")), 1.6), "clamped to upper limit");
    check(nearly(r.EffectiveJointValue(big, r.FindJoint("j1"), false), 3.0), "clamp disabled");
    big["j3"] = -5.0f;
    check(nearly(r.EffectiveJointValue(big, r.FindJoint("j3")), 0.0), "prismatic lower clamp");

    // mimic: finger = 2 * j1 + 0.1 (j1 이 제한으로 잘린 값을 따른다), continuous 는 제한 없음
    JointValues m;
    m["j1"] = 1.0f;
    check(nearly(r.EffectiveJointValue(m, r.FindJoint("jf")), 2.1), "mimic value");
    check(nearly(r.EffectiveJointValue(big, r.FindJoint("jf")), 2 * 1.6 + 0.1), "mimic follows clamped target");
    // mimic 관절을 직접 지정해도 무시된다
    m["jf"] = 100.0f;
    check(nearly(r.EffectiveJointValue(m, r.FindJoint("jf")), 2.1), "explicit value ignored for mimic joint");
    // 고정 관절은 항상 0
    m["jt"] = 5.0f;
    check(nearly(r.EffectiveJointValue(m, r.FindJoint("jt")), 0.0), "fixed joint value is 0");
    T = r.LinkTransforms(m);
    double o[3];
    apply(T[static_cast<size_t>(fin)], 1, 0, 0, o);  // 손가락은 도구 z축 기준 2.1 rad 회전
    check(T.size() == r.links.size(), "transform per link");
    (void)o;
}

void testRpyConvention() {
    // rpy=(r=90, p=0, y=180): R = Rz(180) Rx(90). z 축 -> +Y, y 축 -> +Z, x 축 -> -X (roll/yaw 를 바꾸면 결과가 달라진다)
    const amr::String u =
        "<robot name=\"r\"><link name=\"a\"/><link name=\"b\"/>"
        "<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/>"
        "<origin xyz=\"1 2 3\" rpy=\"1.5707963 0 3.1415926\"/></joint></robot>";
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(u, &r, &err), "rpy robot");
    const amr::Array<dhMat> T = r.LinkTransforms(JointValues());
    double o[3];
    apply(T[1], 0, 0, 1, o);
    check(nearly(o[0], 1) && nearly(o[1], 3) && nearly(o[2], 3), "z axis -> +Y (plus translation)");
    apply(T[1], 0, 1, 0, o);
    check(nearly(o[0], 1) && nearly(o[1], 2) && nearly(o[2], 4), "y axis -> +Z");
    apply(T[1], 1, 0, 0, o);
    check(nearly(o[0], 0) && nearly(o[1], 2) && nearly(o[2], 3), "x axis -> -X");
}

amr::String Wrap(const amr::String& body) { return "<robot name=\"t\">" + body + "</robot>"; }

void expectFail(const amr::String& xml, const char* what) {
    Robot r;
    amr::String err;
    const bool ok = LoadUrdfFromString(xml, &r, &err);
    if (ok || err.empty() || !r.links.empty()) {
        printf("FAIL: %s (ok=%d err='%s')\n", what, ok ? 1 : 0, err.c_str());
        ++g_fail;
    }
}

void testErrors() {
    expectFail("", "empty");
    expectFail("<robot", "broken xml");
    expectFail("<notrobot/>", "wrong root");
    expectFail(Wrap(""), "no links");
    expectFail("<robot name=\"x\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\"><xacro:macro name=\"m\"/><link name=\"a\"/></robot>", "xacro rejected");
    expectFail(Wrap("<link name=\"a\"/><link name=\"a\"/>"), "duplicate link");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/>"), "two roots");
    expectFail(Wrap("<link name=\"a\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"zz\"/></joint>"), "missing child link");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"warp\"><parent link=\"a\"/><child link=\"b\"/></joint>"), "unknown joint type");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"
                    "<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"), "duplicate joint");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>"
                    "<joint name=\"j1\" type=\"fixed\"><parent link=\"a\"/><child link=\"c\"/></joint>"
                    "<joint name=\"j2\" type=\"fixed\"><parent link=\"b\"/><child link=\"c\"/></joint>"), "two parents");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/>"
                    "<joint name=\"j1\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"
                    "<joint name=\"j2\" type=\"fixed\"><parent link=\"b\"/><child link=\"a\"/></joint>"), "cycle without root");
    // 루트 하나 + 분리된 순환 (c <-> d)
    expectFail(Wrap("<link name=\"a\"/><link name=\"c\"/><link name=\"d\"/>"
                    "<joint name=\"j1\" type=\"fixed\"><parent link=\"c\"/><child link=\"d\"/></joint>"
                    "<joint name=\"j2\" type=\"fixed\"><parent link=\"d\"/><child link=\"c\"/></joint>"), "detached cycle");
    expectFail(Wrap("<link name=\"a\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"a\"/></joint>"), "self loop");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/>"
                    "<origin xyz=\"1 2\"/></joint>"), "xyz needs 3 numbers");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/>"
                    "<origin xyz=\"1 nan 3\"/></joint>"), "nan rejected");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/>"
                    "<limit lower=\"2\" upper=\"1\"/></joint>"), "lower > upper");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/>"
                    "<axis xyz=\"0 0 0\"/></joint>"), "zero axis on movable joint");
    expectFail(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/>"
                    "<mimic joint=\"ghost\"/></joint>"), "mimic target missing");
    expectFail(Wrap("<link name=\"a\"><visual><geometry><cylinder radius=\"-1\" length=\"1\"/></geometry></visual></link>"), "negative radius");
    expectFail(Wrap("<link name=\"a\"><visual><geometry><mesh/></geometry></visual></link>"), "mesh without filename");
    expectFail(Wrap("<link name=\"a\"><visual><geometry><capsule/></geometry></visual></link>"), "unknown geometry");
    expectFail(Wrap("<link name=\"a\"><visual/></link>"), "visual without geometry");
    expectFail(Wrap("<link/>"), "link without name");

    // 경고만 나는 경우: revolute 에 limit 이 없음
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/></joint>"), &r, &err),
          "missing limit is lenient");
    check(!r.warnings.empty(), "missing limit warns");
    // 기본값: axis = Z (프로젝트 규약. URDF 규격은 X 이므로 경고를 남긴다)
    check(nearly(r.joints[0].axis.z, 1) && nearly(r.joints[0].axis.x, 0), "default axis is Z");
    bool axisWarn = false;
    for (size_t i = 0; i < r.warnings.size(); i++) axisWarn = axisWarn || r.warnings[i].find("axis") != amr::String::npos;
    check(axisWarn, "missing axis on a movable joint warns");
    check(LoadUrdfFromString(Wrap("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>"), &r, &err) && r.warnings.empty(),
          "fixed joint without axis does not warn");
    // mesh scale 한 개만 있어도 허용
    check(LoadUrdfFromString(Wrap("<link name=\"a\"><visual><geometry><mesh filename=\"x.stl\" scale=\"0.001\"/></geometry></visual></link>"), &r, &err) &&
              nearly(r.links[0].visuals[0].geometry.scale[2], 0.001),
          "single-number mesh scale");

    check(!LoadUrdf(tu::Tmp("amr_no_such.urdf"), &r, &err) && !err.empty(), "missing file");

    // 모든 접두 길이로 잘라도 죽지 않고, 성공하면 구조가 일관된다
    const amr::String good = kArm;
    for (size_t n = 0; n < good.size(); n += 2) {
        Robot t;
        if (LoadUrdfFromString(good.substr(0, n), &t, &err) && t.order.size() != t.joints.size()) check(false, "prefix produced inconsistent robot");
    }
    for (size_t i = 0; i < good.size(); i += 3) {
        amr::String c = good;
        c[i] = static_cast<char>(c[i] ^ 0x21);
        Robot t;
        if (LoadUrdfFromString(c, &t, &err) && t.order.size() != t.joints.size()) check(false, "corruption produced inconsistent robot");
    }
}

amr::Array<amr::String> g_files;  // 만든 파일 (마지막에 지운다)

void writeFile(const amr::String& path, const amr::String& s) {
    g_files.push_back(path);
    amr::WriteFileBytes(path, s.data(), s.size());
}

void testMeshes() {
    const amr::String root = tu::Tmp("amr_urdf_test");
    tu::MkDir(root);
    tu::MkDir(root + "/mypkg");
    tu::MkDir(root + "/mypkg/urdf");
    tu::MkDir(root + "/mypkg/meshes");
    tu::MkDir(root + "/other");

    writeFile(root + "/mypkg/meshes/tri.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    // GLB: 삼각형 (0,0,0),(1,0,0),(0,1,0) -> Y-up 변환 후 y 가 z 로
    {
        amr::Array<char> b;
        const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);
        const amr::String js =
            "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
        const amr::Array<char> glb = testglb::MakeGlb(js, b);
        writeFile(root + "/mypkg/meshes/tri.glb", amr::String(glb.begin(), glb.end()));
    }
    writeFile(root + "/mypkg/urdf/robot.urdf",
        "<robot name=\"m\"><link name=\"base\">"
        "<visual><origin xyz=\"1 0 0\"/><geometry><mesh filename=\"package://mypkg/meshes/tri.obj\" scale=\"2 2 2\"/></geometry>"
        "<material name=\"c\"><color rgba=\"1 0 0 1\"/></material></visual>"
        "<visual><geometry><mesh filename=\"../meshes/tri.glb\"/></geometry></visual>"
        "<visual><geometry><box size=\"1 2 3\"/></geometry></visual>"
        "<visual><geometry><cylinder radius=\"1\" length=\"2\"/></geometry></visual>"
        "<visual><geometry><sphere radius=\"1\"/></geometry></visual>"
        "</link></robot>");

    Robot r;
    amr::String err;
    check(LoadUrdf(root + "/mypkg/urdf/robot.urdf", &r, &err), "file urdf loads");
    check(r.baseDir == root + "/mypkg/urdf", "baseDir recorded");

    LoadOptions opt;
    RobotMeshes rm;
    check(LoadRobotMeshes(r, opt, &rm), "all meshes load (package:// found by walking up)");
    check(rm.visuals.size() == 5 && rm.warnings.empty(), "5 visuals");
    dhVector mn, mx;
    if (rm.visuals.size() == 5) {
        check(rm.visuals[0].scene.ComputeBounds(&mn, &mx) && nearly(mx.x, 2) && nearly(mx.y, 2), "obj scale 2 applied");
        check(nearly(rm.visuals[0].local.v[12], 1.0), "visual origin kept in local");
        check(rm.visuals[0].hasColor && nearly(rm.visuals[0].color.r, 1), "color override carried");
        check(rm.visuals[1].scene.ComputeBounds(&mn, &mx) && nearly(mx.z, 1) && nearly(mx.y, 0), "glb Y-up -> Z-up in URDF");
        check(rm.visuals[2].scene.TriangleCount() == 12, "box primitive");
        check(rm.visuals[3].scene.ComputeBounds(&mn, &mx) && nearly(mx.z, 1) && nearly(mn.z, -1), "cylinder along Z, length 2");
        check(rm.visuals[4].scene.IsValid(), "sphere primitive");
    }
    LoadOptions raw;
    raw.glbYUpToZUp = false;
    RobotMeshes rm2;
    check(LoadRobotMeshes(r, raw, &rm2) && rm2.visuals.size() == 5, "loads with axis conversion off");
    if (rm2.visuals.size() == 5)
        check(rm2.visuals[1].scene.ComputeBounds(&mn, &mx) && nearly(mx.y, 1) && nearly(mx.z, 0), "glb untouched when conversion off");

    // 경로 해석
    amr::String path;
    check(ResolveMeshPath("package://mypkg/meshes/tri.obj", tu::Tmp("zzz_no_such_dir/a/b"), opt, &path) == false, "not found from unrelated dir");
    check(ResolveMeshPath("package://mypkg/meshes/tri.obj", root + "/other", opt, &path), "found via ancestor/pkg/sub");
    LoadOptions withPkg;
    withPkg.packagePaths.push_back(amr::MakePair(amr::String("mypkg"), root + "/mypkg"));
    check(ResolveMeshPath("package://mypkg/meshes/tri.obj", root + "/other", withPkg, &path) && path == root + "/mypkg/meshes/tri.obj",
          "packagePaths mapping");
    LoadOptions withDir;
    withDir.searchDirs.push_back(root);
    check(ResolveMeshPath("package://mypkg/meshes/tri.obj", "", withDir, &path), "searchDirs/pkg/sub");
    check(ResolveMeshPath("file://" + root + "/mypkg/meshes/tri.obj", "", opt, &path) && path == root + "/mypkg/meshes/tri.obj", "file://");
    check(ResolveMeshPath(root + "/mypkg/meshes/tri.obj", "", opt, &path), "absolute path");
    check(ResolveMeshPath("meshes/tri.obj", root + "/mypkg", opt, &path), "relative to baseDir");
    check(!ResolveMeshPath("", "", opt, &path), "empty filename");

    // 없는 메시: 나머지는 채워지고 false + 경고
    writeFile(root + "/mypkg/urdf/broken.urdf",
        "<robot name=\"b\"><link name=\"base\">"
        "<visual><geometry><mesh filename=\"package://mypkg/meshes/ghost.dae\"/></geometry></visual>"
        "<visual><geometry><mesh filename=\"package://mypkg/meshes/tri.obj\"/></geometry></visual>"
        "<visual><geometry><mesh filename=\"package://mypkg/meshes/tri.obj.txt\"/></geometry></visual>"
        "</link></robot>");
    writeFile(root + "/mypkg/meshes/tri.obj.txt", "not a mesh");
    check(LoadUrdf(root + "/mypkg/urdf/broken.urdf", &r, &err), "broken urdf parses");
    RobotMeshes rm3;
    check(!LoadRobotMeshes(r, opt, &rm3), "missing/unsupported mesh -> false");
    check(rm3.visuals.size() == 1 && rm3.warnings.size() == 2, "good mesh kept, 2 warnings");

    for (size_t i = 0; i < g_files.size(); i++) remove(g_files[i].c_str());
    tu::RmDir(root + "/other");
    tu::RmDir(root + "/mypkg/urdf");
    tu::RmDir(root + "/mypkg/meshes");
    tu::RmDir(root + "/mypkg");
    tu::RmDir(root);
}

}  // namespace

void testDynamicsAndUnits() {
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(
              "<robot name='d'><link name='a'/><link name='b'/><link name='c'/>"
              "<joint name='j1' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 0 1'/>"
              "<limit lower='-1' upper='1' effort='5' velocity='1'/><dynamics damping='0.25' friction='0.5'/></joint>"
              "<joint name='j2' type='prismatic'><parent link='b'/><child link='c'/><axis xyz='1 0 0'/>"
              "<limit lower='0' upper='1' effort='5' velocity='1'/></joint></robot>",
              &r, &err),
          "dynamics urdf loads");
    const Joint j1 = r.joints[static_cast<size_t>(r.FindJoint("j1"))];  // 복사 (아래에서 r 을 다시 쓴다)
    const Joint j2 = r.joints[static_cast<size_t>(r.FindJoint("j2"))];
    check(j1.hasDynamics && nearly(j1.damping, 0.25) && nearly(j1.friction, 0.5), "dynamics damping/friction parsed");
    check(!j2.hasDynamics && j2.damping == 0 && j2.friction == 0, "no <dynamics>: zero");
    check(!LoadUrdfFromString(
              "<robot name='d'><link name='a'/><link name='b'/>"
              "<joint name='j' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 0 1'/>"
              "<limit lower='-1' upper='1' effort='5' velocity='1'/><dynamics damping='-1'/></joint></robot>",
              &r, &err),
          "negative damping is rejected");
    // 단위 변환: 각도 관절은 degree, 이동 관절은 m 그대로
    check(j1.IsAngular() && !j2.IsAngular(), "IsAngular");
    check(nearly(j1.ToDisplay(3.14159265358979 / 2), 90.0, 1e-9) && nearly(j1.FromDisplay(180.0), 3.14159265358979, 1e-9), "angular joint: rad <-> deg");
    check(nearly(j2.ToDisplay(0.25), 0.25) && nearly(j2.FromDisplay(0.25), 0.25), "prismatic joint: m unchanged");
    check(amr::String(j1.DisplayUnit()) == "deg" && amr::String(j2.DisplayUnit()) == "m", "display units");
    check(amr::String(j1.TorqueUnit()) == "N*m" && amr::String(j2.TorqueUnit()) == "N", "torque units");
}

void testLidarSensor() {
    const char* base = "<robot name='s'><link name='a'/><link name='b'/>"
                       "<joint name='j' type='fixed'><parent link='a'/><child link='b'/></joint>";
    Robot r;
    amr::String err;
    check(LoadUrdfFromString(amr::String(base) +
              "<gazebo reference='b'><sensor type='gpu_ray' name='scan'><pose>0.1 0.2 0.3 0 0 1.5707963</pose><ray><scan>"
              "<horizontal><samples>181</samples><min_angle>-1.5707963</min_angle><max_angle>1.5707963</max_angle></horizontal>"
              "<vertical><samples>8</samples><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan>"
              "<range><min>0.1</min><max>12</max></range><noise><type>gaussian</type><stddev>0.02</stddev></noise></ray></sensor>"
              "<sensor type='camera' name='cam'/></gazebo></robot>", &r, &err), "lidar sensor loads");
    check(r.lidars.size() == 1, "camera sensor is skipped, lidar kept");
    if (r.lidars.size() == 1) {
        const LidarDef& d = r.lidars[0];
        check(d.name == "scan" && d.link == "b", "lidar name/link");
        check(d.hSamples == 181 && nearly(d.hMin, -1.5707963) && nearly(d.hMax, 1.5707963), "horizontal scan");
        check(d.vSamples == 8 && nearly(d.vMin, -0.2) && nearly(d.vMax, 0.2), "vertical scan");
        check(nearly(d.rangeMin, 0.1) && nearly(d.rangeMax, 12.0) && nearly(d.noiseStdDev, 0.02), "range and noise");
        check(posIs(d.pose, 0.1, 0.2, 0.3), "sensor pose position");
        const dhVector ex = d.pose * dhVector(1, 0, 0);  // yaw 90 도: 센서 +x 가 링크 +y
        check(nearly(ex.x, 0.1, 1e-3) && nearly(ex.y, 1.2, 1e-3), "sensor pose yaw");
    }
    // 기본값: 센서에 scan 이 없어도 읽힌다
    check(LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray/></sensor></gazebo></robot>", &r, &err) && r.lidars.size() == 1 &&
              r.lidars[0].name == "a" && r.lidars[0].vSamples == 1, "defaults");
    // 라이다가 없는 로봇
    check(LoadUrdfFromString(amr::String(base) + "</robot>", &r, &err) && r.lidars.empty(), "no lidar");
    // 잘못된 값은 거부
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='zz'><sensor type='ray'><ray/></sensor></gazebo></robot>", &r, &err), "unknown link rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo><sensor type='ray'><ray/></sensor></gazebo></robot>", &r, &err), "missing reference rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'/></gazebo></robot>", &r, &err), "missing <ray> rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray><scan><horizontal><samples>0</samples></horizontal></scan></ray></sensor></gazebo></robot>", &r, &err), "samples 0 rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray><scan><horizontal><samples>2.5</samples></horizontal></scan></ray></sensor></gazebo></robot>", &r, &err), "fractional samples rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray><scan><horizontal><min_angle>1</min_angle><max_angle>0</max_angle></horizontal></scan></ray></sensor></gazebo></robot>", &r, &err), "max < min rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray><range><min>5</min><max>1</max></range></ray></sensor></gazebo></robot>", &r, &err), "bad range rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><ray><noise><stddev>-1</stddev></noise></ray></sensor></gazebo></robot>", &r, &err), "negative noise rejected");
    check(!LoadUrdfFromString(amr::String(base) + "<gazebo reference='a'><sensor type='ray'><pose>1 2 3</pose><ray/></sensor></gazebo></robot>", &r, &err), "short pose rejected");
}

int main() {
    testLidarSensor();
    testDynamicsAndUnits();
    testParse();
    testKinematics();
    testRpyConvention();
    testErrors();
    testMeshes();
    if (g_fail == 0) printf("test_curdf: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
