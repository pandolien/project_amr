// 모델 크기 배율(ScaleRobot, World::SetModelScale) 테스트
#include <math.h>
#include <stdio.h>

#include "CURDF/Scale.h"
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
const String kData = String(SIMUL_TEST_DATA_DIR) + "/Object/samples/";
}  // namespace

// 모델 크기 배율: 길이는 s 배, 질량 s^3, 관성 s^5, 순기구학의 끝점도 s 배. 원래 크기 기준의 절대 배율이다.
void testScale() {
    Robot arm;
    String err;
    check(LoadUrdf(kData + "arm.urdf", &arm, &err), "load arm for scale");
    Robot half;
    check(ScaleRobot(arm, 0.5, true, &half), "scale arm by 0.5");
    check(!ScaleRobot(arm, 0.0, true, &half) && !ScaleRobot(arm, -1.0, true, &half), "bad scale rejected");
    check(ScaleRobot(arm, 0.5, true, &half), "scale arm by 0.5 again");
    const int tool = arm.FindLink("tool");
    JointValues q;
    q["shoulder_pitch"] = 0.4f;
    q["elbow_pitch"] = -0.7f;
    const dhMat a = arm.LinkTransforms(q)[static_cast<size_t>(tool)];
    const dhMat b = half.LinkTransforms(q)[static_cast<size_t>(tool)];
    check(near(b.v[12], a.v[12] * 0.5, 1e-5) && near(b.v[13], a.v[13] * 0.5, 1e-5) && near(b.v[14], a.v[14] * 0.5, 1e-5), "end point moves s times as far");
    check(near(b.v[0], a.v[0], 1e-6) && near(b.v[10], a.v[10], 1e-6), "orientation is unchanged by scaling");
    const int ua = arm.FindLink("upper_arm");
    check(near(half.links[static_cast<size_t>(ua)].visuals[0].geometry.size[0], arm.links[static_cast<size_t>(ua)].visuals[0].geometry.size[0] * 0.5), "box size scaled");
    check(near(half.links[static_cast<size_t>(ua)].inertial.mass, arm.links[static_cast<size_t>(ua)].inertial.mass / 8.0), "mass scales with s^3");
    check(near(half.links[static_cast<size_t>(ua)].inertial.ixx, arm.links[static_cast<size_t>(ua)].inertial.ixx / 32.0, 1e-7), "inertia scales with s^5");
    Robot same;
    check(ScaleRobot(arm, 0.5, false, &same) && near(same.links[static_cast<size_t>(ua)].inertial.mass, arm.links[static_cast<size_t>(ua)].inertial.mass), "massScales=false keeps the mass");
    // 이동 관절의 한계와 SetModelScale 의 절대 배율
    Robot r;
    Link l0, l1;
    l0.name = "a"; l1.name = "b";
    r.links.push_back(l0); r.links.push_back(l1);
    Joint j;
    j.name = "slide"; j.type = kJointPrismatic; j.parent = "a"; j.child = "b"; j.lower = -1; j.upper = 2; j.hasLimit = true;
    j.origin.v[14] = 1.0f;
    r.joints.push_back(j);
    check(FinalizeRobot(&r, &err), "finalize prismatic robot");
    Robot big;
    check(ScaleRobot(r, 3.0, true, &big) && near(big.joints[0].upper, 6.0) && near(big.joints[0].lower, -3.0) && near(big.joints[0].origin.v[14], 3.0), "prismatic limits and origin scale");
    World w;
    check(w.Add(kData + "arm.urdf", &err), "add arm to world");
    check(w.SetModelScale(0, 2.0) && w.SetModelScale(0, 2.0) && near(w.ModelScale(0), 2.0), "world model scale is absolute");
    Robot built;
    check(w.Build(LoadOptions(), &built, &err), "build scaled world");
    check(near(built.links[static_cast<size_t>(built.FindLink("upper_arm"))].visuals[0].geometry.size[0], 1.2), "scaled geometry reaches the built robot");
    check(w.SetModelScale(0, 1.0) && w.Build(LoadOptions(), &built, &err) && near(built.links[static_cast<size_t>(built.FindLink("upper_arm"))].visuals[0].geometry.size[0], 0.6), "scale back to 1 restores the original");
    check(!w.SetModelScale(5, 2.0) && !w.SetModelScale(0, 0.0), "bad index or scale rejected by world");
}

int main() {
    testScale();
    if (g_fail == 0) printf("test_curdf_scale: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
