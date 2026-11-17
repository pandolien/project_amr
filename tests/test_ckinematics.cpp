// CKinematics 테스트: 수치 IK. 2링크 해석해, 도달 불가 목표, 관절 한계, 이동 관절/mimic, 6축·7축의 임의 도달 가능 목표, 자세 목표
#include <stdio.h>

#include "CBase/Math.h"
#include "CBase/String.h"
#include "CKinematics/IK.h"
#include "CURDF/Urdf.h"

using namespace simul;
using namespace simul::kinematics;
typedef simul::urdf::JointValues JointValues;

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
        return static_cast<double>((s >> 8) & 0xFFFF) / 65536.0;
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

urdf::Robot load(const String& xml) {
    urdf::Robot r;
    String err;
    if (!urdf::LoadUrdfFromString(xml, &r, &err)) {
        printf("load failed: %s\n", err.c_str());
        ++g_fail;
    }
    return r;
}

// 회전 관절 n 개가 이어진 팔. 축은 z, y, x 를 돌아가며 쓰고 링크마다 x 로 0.3 m 뻗는다.
String chainUrdf(int n, double limit) {
    String x = "<robot name='chain'><link name='l0'/>";
    static const char* axes[3] = {"0 0 1", "0 1 0", "1 0 0"};
    for (int i = 1; i <= n; i++) {
        x += "<link name='l" + ToString(i) + "'/>";
        x += "<joint name='j" + ToString(i) + "' type='revolute'><parent link='l" + ToString(i - 1) + "'/><child link='l" + ToString(i) +
             "'/><origin xyz='" + (i == 1 ? "0 0 0.2" : "0.3 0 0") + "'/><axis xyz='" + axes[(i - 1) % 3] + "'/><limit lower='" +
             ToString(-limit) + "' upper='" + ToString(limit) + "' effort='10' velocity='1'/></joint>";
    }
    return x + "</robot>";
}

JointValues randomQ(const urdf::Robot& r, Rng& rng, double span) {
    JointValues q;
    const Array<int> a = r.ActuatedJoints();
    for (size_t i = 0; i < a.size(); i++) q[r.joints[static_cast<size_t>(a[i])].name] = static_cast<float>(rng.range(-span, span));
    return q;
}

JointValues perturbed(const urdf::Robot& r, const JointValues& q, Rng& rng, double d) {
    JointValues o = q;
    const Array<int> a = r.ActuatedJoints();
    for (size_t i = 0; i < a.size(); i++) {
        const String& n = r.joints[static_cast<size_t>(a[i])].name;
        o[n] = o[n] + static_cast<float>(rng.range(-d, d));
    }
    return o;
}

void testPlanarTwoLink() {
    const urdf::Robot r = load(
        "<robot name='p'><link name='b'/><link name='u'/><link name='f'/><link name='t'/>"
        "<joint name='s' type='revolute'><parent link='b'/><child link='u'/><axis xyz='0 0 1'/>"
        "<limit lower='-3.14' upper='3.14' effort='1' velocity='1'/></joint>"
        "<joint name='e' type='revolute'><parent link='u'/><child link='f'/><origin xyz='1 0 0'/><axis xyz='0 0 1'/>"
        "<limit lower='-3.14' upper='3.14' effort='1' velocity='1'/></joint>"
        "<joint name='w' type='fixed'><parent link='f'/><child link='t'/><origin xyz='1 0 0'/></joint></robot>");
    const int tool = r.FindLink("t");
    dhMat target;
    target.Hmatrix(1.0f, 1.0f, 0.0f, 0, 0, 0);
    JointValues start;
    start["s"] = 0.3f;
    start["e"] = 0.5f;
    const IkResult res = SolveIk(r, start, tool, target);
    check(res.converged, "planar: converged");
    check(res.positionError < 1e-4, "planar: position error");
    // 해석해: 팔꿈치 각도는 +-90 도, 어깨는 0 또는 90 도 (시작 값에 가까운 해로 수렴)
    const double e = res.q.at("e");
    check(Abs(Abs(e) - 1.5707963) < 1e-3, "planar: elbow is +-90 deg");

    // 팔 길이(2) 를 넘는 목표: 수렴하지 않고, 팔을 목표 방향으로 쭉 편다 (남은 거리 1)
    dhMat far3;
    far3.Hmatrix(3.0f, 0.0f, 0.0f, 0, 0, 0);
    const IkResult far = SolveIk(r, start, tool, far3);
    check(!far.converged, "unreachable: not converged");
    check(Abs(far.positionError - 1.0) < 1e-3, "unreachable: closest is fully stretched (error 1)");
    check(Abs(far.q.at("s")) < 1e-2 && Abs(far.q.at("e")) < 1e-2, "unreachable: stretched toward target");
}

void testLimits() {
    // 어깨 0..0.5 rad, 팔꿈치 -3..3. 목표는 어깨를 90 도 돌려야만 닿는 위치.
    const urdf::Robot r = load(
        "<robot name='l'><link name='b'/><link name='u'/><link name='f'/>"
        "<joint name='s' type='revolute'><parent link='b'/><child link='u'/><axis xyz='0 0 1'/>"
        "<limit lower='0' upper='0.5' effort='1' velocity='1'/></joint>"
        "<joint name='e' type='revolute'><parent link='u'/><child link='f'/><origin xyz='1 0 0'/><axis xyz='0 0 1'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint></robot>");
    dhMat target;
    target.Hmatrix(0.0f, 1.0f, 0.0f, 0, 0, 0);  // 링크 f 의 원점 = (cos s, sin s, 0): 어깨 90 도 필요
    JointValues start;
    const IkResult res = SolveIk(r, start, r.FindLink("f"), target);
    check(!res.converged, "limits: cannot reach");
    check(res.q.at("s") >= -1e-6f && res.q.at("s") <= 0.5f + 1e-6f, "limits: shoulder stays in [0,0.5]");
    check(Abs(res.q.at("s") - 0.5) < 1e-3, "limits: shoulder at its upper limit (closest)");
    IkOptions free;
    free.respectLimits = false;
    const IkResult res2 = SolveIk(r, start, r.FindLink("f"), target, free);
    check(res2.converged, "limits off: converges");
}

void testPrismaticAndMimic() {
    // 수직 슬라이더(z) + 회전 + 손가락 둘 (mimic, 부호 반대): 손가락 끝을 (0.3, 0.2, 0.5) 로
    const urdf::Robot r = load(
        "<robot name='pm'><link name='b'/><link name='lift'/><link name='arm'/><link name='fa'/><link name='fb'/>"
        "<joint name='z' type='prismatic'><parent link='b'/><child link='lift'/><axis xyz='0 0 1'/>"
        "<limit lower='0' upper='1' effort='1' velocity='1'/></joint>"
        "<joint name='yaw' type='revolute'><parent link='lift'/><child link='arm'/><axis xyz='0 0 1'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "<joint name='ga' type='prismatic'><parent link='arm'/><child link='fa'/><origin xyz='0.4 0 0'/><axis xyz='0 1 0'/>"
        "<limit lower='0' upper='0.1' effort='1' velocity='1'/></joint>"
        "<joint name='gb' type='prismatic'><parent link='arm'/><child link='fb'/><origin xyz='0.4 0 0'/><axis xyz='0 1 0'/>"
        "<limit lower='-0.1' upper='0' effort='1' velocity='1'/><mimic joint='ga' multiplier='-1'/></joint></robot>");
    dhMat target;
    // 손가락 a 의 끝: z=0.5, yaw 로 (0.4, ga) 방향. 도달 가능한 점 = yaw=0.5, ga=0.05, z=0.5
    JointValues truth;
    truth["z"] = 0.5f;
    truth["yaw"] = 0.5f;
    truth["ga"] = 0.05f;
    const Array<dhMat> T = r.LinkTransforms(truth);
    target = T[static_cast<size_t>(r.FindLink("fa"))];
    JointValues start;
    start["z"] = 0.1f;
    start["yaw"] = -0.5f;
    IkOptions o;
    o.useOrientation = false;
    const IkResult res = SolveIk(r, start, r.FindLink("fa"), target, o);
    check(res.converged, "prismatic: converged");
    check(res.positionError < 1e-4, "prismatic: position error");
    check(res.q.at("ga") >= -1e-6f && res.q.at("ga") <= 0.1f + 1e-6f, "prismatic: finger within limits");
    // 두 번째 손가락(mimic)을 목표로 하면 마스터가 반대로 움직여서 같은 점에 닿는다
    const Array<dhMat> Tb = r.LinkTransforms(truth);
    const IkResult resB = SolveIk(r, start, r.FindLink("fb"), Tb[static_cast<size_t>(r.FindLink("fb"))], o);
    check(resB.converged, "mimic: target on mimic link converges");
    double pe = 0;
    PoseError(r, resB.q, r.FindLink("fb"), Tb[static_cast<size_t>(r.FindLink("fb"))], &pe, NULL);
    check(pe < 1e-4, "mimic: PoseError agrees with result");
}

void testRandomReachable(int n, bool withOrientation, unsigned seed) {
    const urdf::Robot r = load(chainUrdf(n, 2.6));
    const int tip = r.FindLink("l" + ToString(n));
    Rng rng(seed);
    int nearOk = 0, farOk = 0;
    const int trials = 60;
    for (int t = 0; t < trials; t++) {
        const JointValues truth = randomQ(r, rng, 2.0);
        const dhMat target = r.LinkTransforms(truth)[static_cast<size_t>(tip)];
        IkOptions o;
        o.useOrientation = withOrientation;
        o.orientationTolerance = 1e-3;
        // (1) 진짜 해 근처에서 시작: 반드시 수렴
        const IkResult a = SolveIk(r, perturbed(r, truth, rng, 0.4), tip, target, o);
        if (a.converged) nearOk++;
        // (2) 임의의 시작: 대부분 수렴 (국소해에 빠질 수 있다)
        o.restarts = 40;
        const IkResult b = SolveIk(r, randomQ(r, rng, 2.0), tip, target, o);
        if (b.converged) farOk++;
        if (a.converged) {
            double pe, oe;
            PoseError(r, a.q, tip, target, &pe, &oe);
            if (pe > 2e-4 || (withOrientation && oe > 2e-3)) check(false, "random: reported converged but FK disagrees");
        }
    }
    printf("n=%d orient=%d near %d/%d far %d/%d\n", n, withOrientation ? 1 : 0, nearOk, trials, farOk, trials);
    check(nearOk >= trials - 1, "random: near start converges (>= 98%)");
    check(farOk >= trials * 95 / 100, "random: random start converges (>= 95%, restarts)");
}

void testRedundantStaysClose() {
    // 7축 팔, 위치만: 최소 변화 해이므로 시작 값에서 크게 벗어나지 않는다
    const urdf::Robot r = load(chainUrdf(7, 2.6));
    const int tip = r.FindLink("l7");
    Rng rng(7);
    const JointValues truth = randomQ(r, rng, 1.0);
    const dhMat target = r.LinkTransforms(truth)[static_cast<size_t>(tip)];
    JointValues start = perturbed(r, truth, rng, 0.1);
    const IkResult res = SolveIk(r, start, tip, target);
    check(res.converged, "7dof: converged");
    double maxd = 0;
    for (int i = 1; i <= 7; i++) {
        const String n = "j" + ToString(i);
        maxd = Max(maxd, static_cast<double>(Abs(res.q.at(n) - start.at(n))));
    }
    check(maxd < 0.3, "7dof: minimal motion from the start (no wandering)");
}

void testEdgeCases() {
    const urdf::Robot r = load(chainUrdf(3, 2.0));
    dhMat t;
    JointValues q;
    check(!SolveIk(r, q, -1, t).converged, "bad link index");
    check(!SolveIk(r, q, 999, t).converged, "link index out of range");
    check(!SolveIk(r, q, r.root, t).converged || true, "root link (no joints) does not crash");
    // 이미 목표에 있음: 0 번 반복
    const dhMat here = r.LinkTransforms(q)[static_cast<size_t>(r.FindLink("l3"))];
    const IkResult res = SolveIk(r, q, r.FindLink("l3"), here);
    check(res.converged && res.iterations == 0, "already at the target: 0 iterations");
    // 자세 오차: 180 도 차이에서도 NaN 없이 동작
    dhMat flip = here;
    flip.Hmatrix(here.v[12], here.v[13], here.v[14], 3.14159265f, 0.0f, 0.0f);
    IkOptions o;
    o.useOrientation = true;
    const IkResult rf = SolveIk(r, q, r.FindLink("l3"), flip, o);
    check(rf.positionError == rf.positionError && rf.orientationError == rf.orientationError, "180 deg error: no NaN");
}

}  // namespace

int main() {
    testPlanarTwoLink();
    testLimits();
    testPrismaticAndMimic();
    testRandomReachable(6, false, 11);
    testRandomReachable(6, true, 12);
    testRandomReachable(7, true, 13);
    testRedundantStaysClose();
    testEdgeCases();
    if (g_fail) {
        printf("%d failed\n", g_fail);
        return 1;
    }
    printf("test_ckinematics: all passed\n");
    return 0;
}
