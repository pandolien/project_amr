// 여러 모델을 한 장면에 올리는 World 테스트
#include <math.h>
#include <stdio.h>

#include "CBase/File.h"
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
bool near(double a, double b, double tol = 1e-4) { return fabs(a - b) <= tol; }

const String kData = String(SIMUL_TEST_DATA_DIR) + "/Object/samples/";

void testSingleModelIsUnchanged() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    Robot arm, built;
    check(LoadUrdf(kData + "arm.urdf", &arm, &err), "load arm directly");
    check(w.Build(LoadOptions(), &built, &err), "build one model");
    check(built.links.size() == arm.links.size() && built.joints.size() == arm.joints.size(), "one model: same structure");
    check(built.links[static_cast<size_t>(built.root)].name == arm.links[static_cast<size_t>(arm.root)].name, "one model: same root");
    check(w.EntryOfRoot(built, built.root) == 0, "one model: root belongs to entry 0");
}

// 모델 하나도 world 아래에 매달 수 있다 (회전을 attach 관절로 표현하려고)
void testSingleModelForcedComposite() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    check(!w.Composite(), "one model is not composite");
    w.SetForceComposite(true);
    check(w.Composite(), "forced composite");
    const dhMat pose = dhMat::Trans(1, 2, 3) * dhMat::RotZ(1.5707963f);
    w.SetPlacement(0, pose);
    Robot built;
    check(w.Build(LoadOptions(), &built, &err), "build forced");
    check(built.links[static_cast<size_t>(built.root)].name == "world", "forced: world root");
    check(w.EntryOfRoot(built, built.FindLink(w.entry(0).rootName)) == 0, "forced: model root found");
    const int j = built.FindJoint(w.AttachJointName(0));
    check(j >= 0 && near(built.joints[static_cast<size_t>(j)].origin.v[12], 1) && near(built.joints[static_cast<size_t>(j)].origin.v[1], 1), "forced: attach joint carries the pose");
    w.Clear();
    check(!w.Composite(), "Clear resets forced composite");
}

void testTwoModels() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    check(w.Add(kData + "lidar_room.urdf", &err), "add room");
    Robot arm, room, built;
    LoadUrdf(kData + "arm.urdf", &arm, &err);
    LoadUrdf(kData + "lidar_room.urdf", &room, &err);
    check(w.Build(LoadOptions(), &built, &err), "build two models");
    check(built.links.size() == 1 + arm.links.size() + room.links.size(), "world link + all links");
    check(built.links[static_cast<size_t>(built.root)].name == "world", "root is the virtual world link");
    check(built.FindJoint("arm_attach") >= 0 && built.FindJoint("lidar_room_attach") >= 0, "attach joints exist");
    check(built.joints[static_cast<size_t>(built.FindJoint("arm_attach"))].type == kJointFixed, "attach joint is fixed");
    check(w.EntryOfRoot(built, built.FindLink(arm.links[static_cast<size_t>(arm.root)].name)) == 0, "arm root -> entry 0");
    check(w.EntryOfLink(built, built.FindLink("pillar")) == 1, "room's pillar -> entry 1");
    check(w.EntryOfLink(built, built.root) == -1, "world link belongs to no entry");
    check(built.lidars.size() == 1 && built.FindLink(built.lidars[0].link) >= 0, "the room's lidar follows into the merged robot");
    // 움직이는 관절은 이름이 그대로 남아 있어야 한다 (--joint, 관절 패널)
    check(built.FindJoint("x_slide") >= 0, "joint names are kept when they do not collide");

    // 배치: 방을 x = 5 로 옮기면 방의 링크가 5 만큼 간다
    w.SetPlacement(1, dhMat::Trans(5, 0, 0));
    check(w.Build(LoadOptions(), &built, &err), "rebuild with placement");
    const Array<dhMat> T = built.LinkTransforms(JointValues());
    check(near(T[static_cast<size_t>(built.FindLink("room"))].v[12], 5.0), "placement moves the model");
    check(near(T[static_cast<size_t>(built.FindLink(arm.links[static_cast<size_t>(arm.root)].name))].v[12], 0.0), "other model stays");
}

void testSameFileTwice() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err) && w.Add(kData + "arm.urdf", &err), "same file twice");
    check(w.entry(0).name == "arm" && w.entry(1).name == "arm_2", "unique entry names");
    Robot arm, built;
    LoadUrdf(kData + "arm.urdf", &arm, &err);
    check(w.Build(LoadOptions(), &built, &err), "build duplicates");
    check(built.links.size() == 1 + 2 * arm.links.size(), "both copies are present");
    // 두 번째 복사본의 이름은 prefix 가 붙는다
    const String firstJoint = arm.joints[0].name;
    check(built.FindJoint(firstJoint) >= 0 && built.FindJoint("arm_2/" + firstJoint) >= 0, "second copy's joints are prefixed");
    // 관절 값이 서로 독립이어야 한다
    JointValues q;
    q["arm_2/" + firstJoint] = 0.5f;
    const Array<dhMat> a = built.LinkTransforms(JointValues());
    const Array<dhMat> b = built.LinkTransforms(q);
    bool firstSame = true, secondMoved = false;
    for (size_t i = 0; i < arm.links.size(); i++) {
        const size_t i1 = static_cast<size_t>(built.FindLink(arm.links[i].name));
        const size_t i2 = static_cast<size_t>(built.FindLink("arm_2/" + arm.links[i].name));
        for (int k = 0; k < 16; k++) {
            if (!near(a[i1].v[k], b[i1].v[k])) firstSame = false;
            if (i2 < built.links.size() && !near(a[i2].v[k], b[i2].v[k])) secondMoved = true;
        }
    }
    check(firstSame, "moving the second copy leaves the first unchanged");
    check(secondMoved, "moving the second copy moves it");

    // 제거
    w.Remove(0);
    check(w.size() == 1 && w.entry(0).name == "arm_2", "remove first entry");
    check(w.Build(LoadOptions(), &built, &err) && built.links.size() == arm.links.size(), "build after remove (single model)");
}

void testMeshFileAsModel() {
    const String stl = tu::Tmp("simul_world_t.stl");
    const String txt = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
    WriteFileBytes(stl, txt.data(), txt.size());
    World w;
    String err;
    check(w.Add(stl, &err), "add stl");
    check(w.Add(kData + "arm.urdf", &err), "add arm next to it");
    Robot built;
    check(w.Build(LoadOptions(), &built, &err), "build stl + arm");
    RobotMeshes meshes;
    check(LoadRobotMeshes(built, LoadOptions(), &meshes), "meshes of the merged robot load");
    bool hasStl = false;
    for (size_t i = 0; i < meshes.visuals.size(); i++) if (built.links[static_cast<size_t>(meshes.visuals[i].link)].name == "simul_world_t") hasStl = true;
    check(hasStl, "the stl is a visual of its own link");
    check(!w.Add("/no/such/file.urdf", &err) && !err.empty(), "missing file is rejected");
    check(!w.Add("/no/such/file.stl", &err), "missing mesh is rejected");
    remove(stl.c_str());
}

void testBuildEmpty() {
    World w;
    Robot r;
    String err;
    check(!w.Build(LoadOptions(), &r, &err) && !err.empty(), "empty world cannot build");
}
}  // namespace


// 모델을 다른 모델의 링크에 관절로 연결한다
void testAttachToLink() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    PrimitiveSpec box;
    box.type = kGeomBox;
    box.size[0] = box.size[1] = box.size[2] = 0.1f;
    box.mass = 1.0f;
    Robot b;
    check(MakePrimitiveRobot("crate", box, &b, &err), "make crate");
    check(w.AddRobot("crate", b, &err), "add crate");
    const String armName = w.entry(0).name;

    AttachSpec a;
    a.parentModel = armName;
    a.parentLink = "tool";
    a.type = kJointRevolute;
    a.axis = dhVector(0, 0, 2);   // 정규화된다
    a.hasLimit = true; a.lower = -1.0f; a.upper = 1.0f; a.effort = 5.0f; a.velocity = 2.0f;
    check(w.SetAttach(1, a, &err), ("connect crate to tool: " + err).c_str());
    check(w.IsConnected(1) && w.ParentOf(1) == 0 && w.RootOf(1) == 0 && w.RootOf(0) == 0 && w.IsAncestor(0, 1) && !w.IsAncestor(1, 0), "parent/root queries");
    w.SetPlacement(1, dhMat::Trans(0.0f, 0.0f, 0.25f));
    Robot built;
    check(w.Build(LoadOptions(), &built, &err), ("build connected scene: " + err).c_str());
    const int jc = built.FindJoint("crate_attach");
    check(jc >= 0, "attach joint exists");
    if (jc >= 0) {
        const Joint& j = built.joints[static_cast<size_t>(jc)];
        check(j.parent == "tool" && j.child == w.entry(1).rootName && j.type == kJointRevolute, "joint hangs from the tool link");
        check(near(j.axis.z, 1.0) && near(j.axis.x, 0.0) && j.hasLimit && near(j.upper, 1.0) && near(j.effort, 5.0), "axis normalized, limits kept");
        check(near(j.origin.v[14], 0.25), "origin = placement relative to the parent link");
    }
    check(built.links[static_cast<size_t>(built.root)].name == "world", "root is world");
    // 기구학: 연결한 모델은 tool 링크를 따라가고, attach 관절 값만큼 돈다
    JointValues q;
    Array<dhMat> t0 = built.LinkTransforms(q);
    const int tool = built.FindLink("tool"), crate = built.FindLink(w.entry(1).rootName);
    check(tool >= 0 && crate >= 0, "links found");
    if (tool >= 0 && crate >= 0) {
        const dhMat rel = t0[static_cast<size_t>(tool)].Inv() * t0[static_cast<size_t>(crate)];
        check(near(rel.v[14], 0.25) && near(rel.v[12], 0.0) && near(rel.v[0], 1.0), "crate rides on the tool at the joint origin");
        q["crate_attach"] = 0.5;
        t0 = built.LinkTransforms(q);
        const dhMat rel2 = t0[static_cast<size_t>(tool)].Inv() * t0[static_cast<size_t>(crate)];
        check(near(rel2.v[0], cos(0.5)) && near(rel2.v[1], sin(0.5)) && near(rel2.v[14], 0.25), "attach joint value rotates the crate about its axis");
        q["shoulder_pitch"] = 0.4;   // 팔이 움직이면 같이 간다
        const Array<dhMat> t1 = built.LinkTransforms(q);
        check(fabs(t1[static_cast<size_t>(crate)].v[14] - t0[static_cast<size_t>(crate)].v[14]) > 1e-3, "crate follows the arm");
    }
    check(w.EntryOfLink(built, crate) == 1 && w.EntryOfRoot(built, crate) == 1, "crate root still belongs to the crate model");

    // 거부되는 연결
    AttachSpec bad = a;
    bad.parentModel = "crate";
    check(!w.SetAttach(1, bad, &err) && !err.empty(), "cannot attach to itself");
    bad = a; bad.parentModel = "nobody";
    check(!w.SetAttach(1, bad, &err), "unknown parent model");
    bad = a; bad.parentLink = "nolink";
    check(!w.SetAttach(1, bad, &err), "unknown parent link");
    bad = a; bad.parentModel = "crate"; bad.parentLink = w.entry(1).robot.links[0].name;
    check(!w.SetAttach(0, bad, &err) && err.find("순환") != String::npos, "cycle is rejected (arm onto its own child)");
    bad = a; bad.hasLimit = false;
    check(!w.SetAttach(1, bad, &err), "revolute needs limits");
    bad = a; bad.axis = dhVector(0, 0, 0);
    check(!w.SetAttach(1, bad, &err), "zero axis is rejected");
    bad = a; bad.type = kJointFloating;
    check(!w.SetAttach(1, bad, &err), "unsupported type is rejected");
    check(w.IsConnected(1) && w.entry(1).attach.type == kJointRevolute, "a rejected connect leaves the old one in place");

    // 고정으로 바꾸기, 연결 풀기
    AttachSpec fixed = a;
    fixed.type = kJointFixed;
    check(w.SetAttach(1, fixed, &err), "re-connect as fixed");
    check(w.Build(LoadOptions(), &built, &err) && built.joints[static_cast<size_t>(built.FindJoint("crate_attach"))].type == kJointFixed, "fixed attach joint");
    w.ClearAttach(1);
    check(!w.IsConnected(1) && w.Build(LoadOptions(), &built, &err), "disconnect, rebuild");
    check(built.joints[static_cast<size_t>(built.FindJoint("crate_attach"))].parent == "world", "after disconnect it hangs from world again");

    // 부모를 지우면 자식의 연결이 풀린다
    check(w.SetAttach(1, a, &err), "connect again");
    w.Remove(0);
    check(w.size() == 1 && !w.IsConnected(0), "removing the parent disconnects the child");

    // 부모가 뒤에 있어도 된다 (두 단계 합치기)
    World w2;
    check(w2.AddRobot("crate", b, &err) && w2.Add(kData + "arm.urdf", &err), "add crate then arm");
    AttachSpec c = a;
    c.parentModel = w2.entry(1).name;
    check(w2.SetAttach(0, c, &err), "child before parent");
    check(w2.Build(LoadOptions(), &built, &err), ("build with the parent after the child: " + err).c_str());
}

int main() {
    testSingleModelIsUnchanged();
    testSingleModelForcedComposite();
    testTwoModels();
    testSameFileTwice();
    testMeshFileAsModel();
    testBuildEmpty();
    testAttachToLink();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_cworld: all passed\n");
    return 0;
}
