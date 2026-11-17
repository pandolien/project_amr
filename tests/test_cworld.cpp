// 여러 모델을 한 장면에 올리는 World 테스트
#include <math.h>
#include <stdio.h>

#include "CBase/File.h"
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

int main() {
    testSingleModelIsUnchanged();
    testTwoModels();
    testSameFileTwice();
    testMeshFileAsModel();
    testBuildEmpty();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_cworld: all passed\n");
    return 0;
}
