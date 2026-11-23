// CDynamics 테스트: 중력 토크. 해석해(진자, 슬라이더), 위치에너지 기울기(독립 구현인 float FK 로 수치 미분)와의 대조,
// 중력 방향 변화, mimic 합산, 관절 제한, 질량중심
#include "CBase/Math.h"
#include <stdio.h>
#include <stdlib.h>
#include "CBase/String.h"

#include "CDynamics/Dynamics.h"
#include "CURDF/World.h"
#include "3d/dhMat.h"
#include "CDynamics/FreeFall.h"
#include "CDynamics/RigidFall.h"
#include "CDynamics/Gravity.h"
#include "CURDF/Urdf.h"

using namespace simul;
using namespace simul::dynamics;

namespace {

int g_fail = 0;
const double G = kStandardGravity;

void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-5) { return simul::Abs(a - b) <= eps; }

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    double next() {
        s = s * 1664525u + 1013904223u;
        return static_cast<double>((s >> 8) & 0xFFFF) / 65536.0;
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

urdf::Robot load(const char* xml) {
    urdf::Robot r;
    simul::String err;
    if (!urdf::LoadUrdfFromString(xml, &r, &err)) {
        printf("URDF load failed: %s\n", err.c_str());
        exit(2);
    }
    return r;
}

const char* kPendulum =
    "<robot name='p'><link name='base'/>"
    " <link name='arm'><inertial><origin xyz='0.5 0 0'/><mass value='2'/>"
    "  <inertia ixx='0.1' ixy='0' ixz='0' iyy='0.1' iyz='0' izz='0.1'/></inertial></link>"
    " <joint name='j' type='revolute'><parent link='base'/><child link='arm'/><axis xyz='0 1 0'/>"
    "  <limit lower='-3.2' upper='3.2' effort='50' velocity='1'/></joint></robot>";

void testPendulum() {
    const urdf::Robot r = load(kPendulum);
    urdf::JointValues q;
    JointTorques t = GravityTorques(r, q);
    check(t.size() == 1 && t.count("j") == 1, "one actuated joint");
    check(nearly(t["j"], -2 * G * 0.5), "horizontal arm: tau = -m g L");

    // tau(theta) = -m g L cos(theta): 위치에너지 U = -m g L sin(theta) 의 기울기
    for (int deg = -170; deg <= 170; deg += 20) {
        const double th = deg * 3.14159265358979 / 180.0;
        q["j"] = static_cast<float>(th);
        check(nearly(GravityTorques(r, q)["j"], -2 * G * 0.5 * simul::Cos(th), 2e-5), "pendulum tau = -m g L cos(theta)");
    }
    q["j"] = static_cast<float>(3.14159265358979 / 2);
    check(nearly(GravityTorques(r, q)["j"], 0.0, 1e-5), "arm hanging straight down needs no torque");

    // 중력 방향: 반대로 하면 부호가 바뀌고, 0 이면 토크가 없다
    q["j"] = 0.0f;
    check(nearly(GravityTorques(r, q, Gravity(0, 0, G))["j"], 2 * G * 0.5), "reversed gravity flips the sign");
    check(nearly(GravityTorques(r, q, Gravity(0, 0, 0))["j"], 0.0), "no gravity, no torque");
    check(nearly(GravityTorques(r, q, Gravity(0, G, 0))["j"], 0.0, 1e-9), "gravity along the joint axis gives no torque");
    // 기울어진 중력(AMR 이 경사에 있을 때): g = (-G sin f, 0, -G cos f) 이면 tau = -m g L cos(theta - f)  (f = 경사 각)
    const double f = 0.4;
    for (int deg = -90; deg <= 90; deg += 30) {
        const double th = deg * 3.14159265358979 / 180.0;
        q["j"] = static_cast<float>(th);
        // 점 질량 위치 p = L (cos th, 0, -sin th),  U = -m g.p = m G L (sin f cos th - cos f sin th),  dU/dth = -m G L cos(th - f)
        const double expected = -2 * 0.5 * G * simul::Cos(th - f);
        check(nearly(GravityTorques(r, q, Gravity(-G * simul::Sin(f), 0, -G * simul::Cos(f)))["j"], expected, 3e-5), "tilted gravity vector");
    }

    // 관절 제한: 범위를 넘는 값은 제한값과 같다
    q["j"] = 10.0f;
    const double clamped = GravityTorques(r, q)["j"];
    q["j"] = 3.2f;
    check(nearly(clamped, GravityTorques(r, q)["j"]), "values past the limit are clamped");
    q["j"] = 10.0f;
    check(!nearly(GravityTorques(r, q, Gravity(), false)["j"], clamped, 1e-3), "unclamped uses the raw value");
}

void testPrismatic() {
    const urdf::Robot up = load(
        "<robot name='s'><link name='base'/><link name='slider'><inertial><mass value='3'/></inertial></link>"
        " <joint name='z' type='prismatic'><parent link='base'/><child link='slider'/><axis xyz='0 0 1'/>"
        "  <limit lower='0' upper='1' effort='100' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(up, urdf::JointValues())["z"], 3 * G), "vertical slider: holding force = +m g (against the weight)");
    const urdf::Robot down = load(
        "<robot name='s'><link name='base'/><link name='slider'><inertial><mass value='3'/></inertial></link>"
        " <joint name='z' type='prismatic'><parent link='base'/><child link='slider'/><axis xyz='0 0 -1'/>"
        "  <limit lower='0' upper='1' effort='100' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(down, urdf::JointValues())["z"], -3 * G), "slider along -z: force sign follows the axis");
    const urdf::Robot side = load(
        "<robot name='s'><link name='base'/><link name='slider'><inertial><mass value='3'/></inertial></link>"
        " <joint name='x' type='prismatic'><parent link='base'/><child link='slider'/><axis xyz='1 0 0'/>"
        "  <limit lower='0' upper='1' effort='100' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(side, urdf::JointValues())["x"], 0.0, 1e-9), "horizontal slider carries no gravity load");
}

// 가지가 있는 트리: 회전 + 이동 + fixed + mimic + continuous, 회전된 관절/관성 원점, 정규화되지 않은 축
const char* kTree =
    "<robot name='tree'>"
    " <link name='base'><inertial><origin xyz='0.1 0 0.05'/><mass value='5'/></inertial></link>"
    " <link name='l1'><inertial><origin xyz='0.2 0.05 0.1' rpy='0.3 0.2 0.1'/><mass value='2'/></inertial></link>"
    " <link name='l2'><inertial><origin xyz='0.1 0 0'/><mass value='1.5'/></inertial></link>"
    " <link name='l3'><inertial><origin xyz='0 0.1 0.2'/><mass value='0.8'/></inertial></link>"
    " <link name='l4'><inertial><origin xyz='0.3 0 0'/><mass value='1.2'/></inertial></link>"
    " <link name='l5'><inertial><origin xyz='0 0 0.1'/><mass value='0.5'/></inertial></link>"
    " <link name='l6'><inertial><origin xyz='0.1 0 0'/><mass value='0.7'/></inertial></link>"
    " <link name='l7'><inertial><origin xyz='0 0 0.1'/><mass value='0.4'/></inertial></link>"
    " <link name='l8'/>"
    " <joint name='j1' type='revolute'><parent link='base'/><child link='l1'/><origin xyz='0.1 0.2 0.3' rpy='0.2 -0.4 0.9'/>"
    "   <axis xyz='0.3 0.5 0.8'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
    " <joint name='j2' type='prismatic'><parent link='l1'/><child link='l2'/><origin xyz='0.3 0 0.1' rpy='0 0.5 0'/>"
    "   <axis xyz='1 0.2 0'/><limit lower='-0.2' upper='0.5' effort='1' velocity='1'/></joint>"
    " <joint name='j3' type='fixed'><parent link='l2'/><child link='l3'/><origin xyz='0.2 0.1 0' rpy='1.0 0 0.3'/></joint>"
    " <joint name='j4' type='revolute'><parent link='l3'/><child link='l4'/><origin xyz='0 0.1 0.2'/>"
    "   <axis xyz='0 1 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
    " <joint name='j5' type='revolute'><parent link='l1'/><child link='l5'/><origin xyz='0 0.3 0' rpy='0 0 -0.7'/>"
    "   <axis xyz='1 0 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
    " <joint name='j6' type='revolute'><parent link='l2'/><child link='l6'/><origin xyz='0.1 0.2 0' rpy='0 0.3 0'/>"
    "   <axis xyz='0 0 1'/><limit lower='-3' upper='3' effort='1' velocity='1'/><mimic joint='j4' multiplier='-0.5' offset='0.1'/></joint>"
    " <joint name='j7' type='continuous'><parent link='l4'/><child link='l7'/><origin xyz='0.2 0 0'/><axis xyz='0 1 1'/></joint>"
    " <joint name='j8' type='fixed'><parent link='l7'/><child link='l8'/></joint>"
    "</robot>";

// 독립 구현: float FK(Robot::LinkTransforms) 로 중력 위치에너지 U = -sum m g.p 를 구한다
double potential(const urdf::Robot& r, const urdf::JointValues& q, const Gravity& g) {
    const simul::Array<dhMat> T = r.LinkTransforms(q);
    double u = 0;
    for (size_t i = 0; i < r.links.size(); i++) {
        if (!r.links[i].hasInertial) continue;
        const dhVector p = (T[i] * r.links[i].inertial.origin).O();
        u -= static_cast<double>(r.links[i].inertial.mass) * (g.x * p.x + g.y * p.y + g.z * p.z);
    }
    return u;
}

void testGradientOfPotential() {
    const urdf::Robot r = load(kTree);
    const simul::Array<int> act = r.ActuatedJoints();
    check(act.size() == 5, "actuated joints: j1 j2 j4 j5 j7 (j3, j8 fixed, j6 mimic)");

    Rng rng(7);
    int compared = 0;
    for (int iter = 0; iter < 60; iter++) {
        urdf::JointValues q;
        q["j1"] = static_cast<float>(rng.range(-2, 2));
        q["j2"] = static_cast<float>(rng.range(-0.15, 0.45));
        q["j4"] = static_cast<float>(rng.range(-2, 2));
        q["j5"] = static_cast<float>(rng.range(-2, 2));
        q["j7"] = static_cast<float>(rng.range(-6, 6));
        // 임의 방향의 중력
        Gravity g(rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1));
        const double n = simul::Sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
        g = Gravity(g.x / n * G, g.y / n * G, g.z / n * G);

        const JointTorques t = GravityTorques(r, q, g);
        check(t.size() == 5, "torques for the 5 actuated joints only");
        for (size_t a = 0; a < act.size(); a++) {
            const simul::String name = r.joints[static_cast<size_t>(act[a])].name;
            const double h = 0.01;
            urdf::JointValues qp = q, qm = q;
            qp[name] = q[name] + static_cast<float>(h);
            qm[name] = q[name] - static_cast<float>(h);
            const double fd = (potential(r, qp, g) - potential(r, qm, g)) / (2 * h);
            const double got = t.find(name)->second;
            const bool ok = simul::Abs(got - fd) <= 0.01 + 0.004 * simul::Abs(fd);
            if (!ok) printf("  joint %s: analytic %.5f vs gradient %.5f\n", name.c_str(), got, fd);
            check(ok, "tau == dU/dq (float-FK finite difference)");
            compared++;
        }
    }
    check(compared == 300, "compared every joint in every sample");
}

void testStructure() {
    // <inertial> 이 없는 로봇은 토크 0
    const urdf::Robot r = load(
        "<robot name='n'><link name='a'/><link name='b'/>"
        "<joint name='j' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(r, urdf::JointValues())["j"], 0.0, 1e-12), "no mass, no torque");

    // 질량 0 은 무시된다
    const urdf::Robot z = load(
        "<robot name='n'><link name='a'/><link name='b'><inertial><mass value='0'/></inertial></link>"
        "<joint name='j' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(z, urdf::JointValues())["j"], 0.0, 1e-12), "zero mass is ignored");

    // 루트 링크의 질량은 어떤 관절에도 걸리지 않는다
    const urdf::Robot heavyBase = load(
        "<robot name='n'><link name='a'><inertial><origin xyz='5 5 5'/><mass value='100'/></inertial></link><link name='b'/>"
        "<joint name='j' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>");
    check(nearly(GravityTorques(heavyBase, urdf::JointValues())["j"], 0.0, 1e-12), "root link mass loads no joint");

    // 아래 링크의 질량은 윗 관절에 모두 걸린다: 2 단 팔의 어깨 토크 = 팔꿈치 이후 질량 모두 포함
    const urdf::Robot two = load(
        "<robot name='n'><link name='a'/>"
        "<link name='b'><inertial><origin xyz='0.5 0 0'/><mass value='1'/></inertial></link>"
        "<link name='c'><inertial><origin xyz='0.5 0 0'/><mass value='1'/></inertial></link>"
        "<joint name='s' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "<joint name='e' type='revolute'><parent link='b'/><child link='c'/><origin xyz='1 0 0'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint></robot>");
    const JointTorques t = GravityTorques(two, urdf::JointValues());
    check(nearly(t.find("e")->second, -G * 0.5), "elbow carries only the forearm: -m g 0.5");
    check(nearly(t.find("s")->second, -G * (0.5 + 1.5)), "shoulder carries both: -g (1*0.5 + 1*1.5)");

    // mimic: 원본 관절(g)에 multiplier 배로 합쳐진다
    const urdf::Robot mm = load(
        "<robot name='n'><link name='a'/>"
        "<link name='b'><inertial><origin xyz='1 0 0'/><mass value='1'/></inertial></link>"
        "<link name='c'><inertial><origin xyz='1 0 0'/><mass value='2'/></inertial></link>"
        "<joint name='m' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "<joint name='f' type='revolute'><parent link='a'/><child link='c'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/><mimic joint='m' multiplier='3' offset='0'/></joint></robot>");
    const JointTorques tm = GravityTorques(mm, urdf::JointValues());
    check(tm.size() == 1 && tm.count("m") == 1, "mimic joint is not listed");
    // m 자체: -1*G*1 (수평), f 단독: -2*G*1, q_f = 3 q_m 이므로 m 에는 3 * (-2G) 가 더해진다. (q=0 에서 cos=1)
    check(nearly(tm.find("m")->second, -G * 1 + 3 * (-G * 2)), "mimic torque folded into the master with its multiplier");
}

void testCenterOfMass() {
    const urdf::Robot r = load(
        "<robot name='n'><link name='a'><inertial><origin xyz='0 0 1'/><mass value='3'/></inertial></link>"
        "<link name='b'><inertial><origin xyz='1 0 0'/><mass value='1'/></inertial></link>"
        "<joint name='j' type='prismatic'><parent link='a'/><child link='b'/><origin xyz='1 0 0'/><axis xyz='0 0 1'/>"
        "<limit lower='0' upper='2' effort='1' velocity='1'/></joint></robot>");
    double c[3], m = 0;
    urdf::JointValues q;
    q["j"] = 2.0f;
    check(CenterOfMass(r, q, c, &m), "center of mass exists");
    // a: 3 kg @ (0,0,1),  b: 1 kg @ (1+1, 0, 2)
    check(nearly(m, 4.0) && nearly(c[0], 2.0 / 4) && nearly(c[1], 0.0) && nearly(c[2], (3.0 + 2.0) / 4), "center of mass and total mass");
    const urdf::Robot none = load("<robot name='n'><link name='a'/></robot>");
    check(!CenterOfMass(none, urdf::JointValues(), c, &m), "no mass: no center of mass");
}


const JointLoad* findLoad(const simul::Array<JointLoad>& v, const char* name) {
    for (size_t i = 0; i < v.size(); i++)
        if (v[i].name == name) return &v[i];
    return NULL;
}

double sumOf(const simul::Array<LinkContribution>& v) {
    double s = 0;
    for (size_t i = 0; i < v.size(); i++) s += v[i].value;
    return s;
}

void testPerJointLoads() {
    // 진자: 힘은 위로 m g, 모멘트는 -y 방향으로 m g L, torque 는 그 모멘트의 y 성분
    const urdf::Robot p = load(kPendulum);
    simul::Array<JointLoad> l = GravityLoads(p, urdf::JointValues());
    check(l.size() == 1 && l[0].name == "j" && l[0].movable && !l[0].mimic, "pendulum load entry");
    check(nearly(l[0].force[2], 2 * G) && nearly(l[0].force[0], 0) && nearly(l[0].force[1], 0), "force = -M g (upward)");
    check(nearly(l[0].moment[1], -2 * G * 0.5) && nearly(l[0].moment[0], 0) && nearly(l[0].moment[2], 0), "moment about the joint");
    check(nearly(l[0].torque, l[0].moment[1]), "torque == axis . moment");

    // fixed 관절도 하중을 가진다 (체결부가 받는 힘/모멘트)
    const urdf::Robot f = load(
        "<robot name='f'><link name='base'/><link name='l'><inertial><origin xyz='0.5 0 0'/><mass value='2'/></inertial></link>"
        "<joint name='weld' type='fixed'><parent link='base'/><child link='l'/></joint></robot>");
    l = GravityLoads(f, urdf::JointValues());
    check(l.size() == 1 && !l[0].movable && l[0].torque == 0.0, "fixed joint: listed, not movable, no joint torque");
    check(nearly(l[0].force[2], 2 * G) && nearly(l[0].moment[1], -2 * 0.5 * G) && nearly(l[0].moment[0], 0), "fixed joint carries the weld load");
    check(GravityTorques(f, urdf::JointValues()).empty(), "fixed joints are not in GravityTorques");

    // 가지가 있는 트리: 모든 관절이 나오고, mimic 은 자기 몫이 따로 있으며 원본과 합치면 GravityTorques 와 같다
    const urdf::Robot t = load(kTree);
    urdf::JointValues q;
    q["j1"] = 0.4f;
    q["j2"] = 0.1f;
    q["j4"] = -0.8f;
    q["j5"] = 0.6f;
    q["j7"] = 1.0f;
    l = GravityLoads(t, q);
    check(l.size() == t.joints.size(), "one entry per joint (including fixed and mimic)");
    const JointLoad* j3 = findLoad(l, "j3");
    const JointLoad* j6 = findLoad(l, "j6");
    const JointLoad* j4 = findLoad(l, "j4");
    check(j3 && !j3->movable && j3->torque == 0.0, "j3 fixed");
    check(j6 && j6->movable && j6->mimic && simul::Abs(j6->torque) > 1e-6, "j6 mimic has its own share");
    const JointTorques tm = GravityTorques(t, q);
    check(j4 && nearly(tm.find("j4")->second, j4->torque + (-0.5) * j6->torque, 1e-9), "master torque = own + multiplier * mimic's own");
    for (size_t i = 0; i < l.size(); i++) {
        if (!l[i].movable || l[i].mimic) continue;
        // mimic 이 달리지 않은 구동 관절은 GravityTorques 와 똑같다
        if (l[i].name != "j4") check(nearly(tm.find(l[i].name)->second, l[i].torque, 1e-9), "non-master actuated joint matches GravityTorques");
    }
    // 하중의 힘은 -M g: j1 아래쪽 전체 질량 (l1..l8 = 2+1.5+0.8+1.2+0.5+0.7+0.4 = 7.1 kg)
    const JointLoad* j1 = findLoad(l, "j1");
    check(j1 && nearly(simul::Sqrt(j1->force[0] * j1->force[0] + j1->force[1] * j1->force[1] + j1->force[2] * j1->force[2]), 7.1 * G, 1e-5), "|force| = M g");
}

void testContributions() {
    const urdf::Robot two = load(
        "<robot name='n'><link name='a'/>"
        "<link name='b'><inertial><origin xyz='0.5 0 0'/><mass value='1'/></inertial></link>"
        "<link name='c'><inertial><origin xyz='0.5 0 0'/><mass value='1'/></inertial></link>"
        "<link name='d'/>"
        "<joint name='s' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "<joint name='e' type='revolute'><parent link='b'/><child link='c'/><origin xyz='1 0 0'/><axis xyz='0 1 0'/>"
        "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "<joint name='w' type='fixed'><parent link='c'/><child link='d'/></joint></robot>");
    simul::Array<LinkContribution> s = GravityContributions(two, urdf::JointValues(), "s");
    check(s.size() == 2 && s[0].link == "b" && s[1].link == "c", "shoulder: contributions from b and c (d has no mass)");
    check(s.size() == 2 && nearly(s[0].value, -G * 0.5) && nearly(s[1].value, -G * 1.5), "shoulder: b = -g*0.5, c = -g*1.5");
    const simul::Array<LinkContribution> e = GravityContributions(two, urdf::JointValues(), "e");
    check(e.size() == 1 && e[0].link == "c" && nearly(e[0].value, -G * 0.5), "elbow: only c");
    check(GravityContributions(two, urdf::JointValues(), "w").empty(), "fixed joint: no contributions");
    check(GravityContributions(two, urdf::JointValues(), "nope").empty(), "unknown joint: no contributions");

    // 합은 항상 그 관절의 torque 와 같다 (가지가 있는 트리, 임의 자세와 중력)
    const urdf::Robot t = load(kTree);
    Rng rng(11);
    for (int iter = 0; iter < 30; iter++) {
        urdf::JointValues q;
        q["j1"] = static_cast<float>(rng.range(-2, 2));
        q["j2"] = static_cast<float>(rng.range(-0.15, 0.45));
        q["j4"] = static_cast<float>(rng.range(-2, 2));
        q["j5"] = static_cast<float>(rng.range(-2, 2));
        q["j7"] = static_cast<float>(rng.range(-6, 6));
        const Gravity g(rng.range(-9, 9), rng.range(-9, 9), rng.range(-9, 9));
        const simul::Array<JointLoad> loads = GravityLoads(t, q, g);
        for (size_t i = 0; i < loads.size(); i++) {
            if (!loads[i].movable) continue;
            check(nearly(sumOf(GravityContributions(t, q, loads[i].name, g)), loads[i].torque, 1e-9), "sum of link contributions == joint torque");
        }
    }
}


// ---- 바닥 위 강체 낙하 (회전)
namespace {
// 1 m 정육면체의 꼭짓점 (몸체 원점 = 중심)
simul::Array<double> unitCube(double sx = 1.0, double sy = 1.0, double sz = 1.0) {
    simul::Array<double> p;
    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2) {
                p.push_back(0.5 * sx * x);
                p.push_back(0.5 * sy * y);
                p.push_back(0.5 * sz * z);
            }
    return p;
}
void rotX(double a, double r[9]) {
    const double c = simul::Cos(a), s = simul::Sin(a);
    const double m[9] = {1, 0, 0, 0, c, -s, 0, s, c};
    for (int i = 0; i < 9; i++) r[i] = m[i];
}
void rotXY(double ax, double ay, double r[9]) {
    double a[9], b[9];
    rotX(ax, a);
    const double c = simul::Cos(ay), s = simul::Sin(ay);
    const double m[9] = {c, 0, s, 0, 1, 0, -s, 0, c};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) b[i * 3 + j] = m[i * 3] * a[j] + m[i * 3 + 1] * a[3 + j] + m[i * 3 + 2] * a[6 + j];
    for (int i = 0; i < 9; i++) r[i] = b[i];
}
bool faceDown(const simul::dynamics::RigidFall& b, double tol = 1e-3) {   // 몸체의 한 축이 수직이다 (면이 바닥에 놓였다)
    return simul::Max(simul::Abs(b.rot[6]), simul::Max(simul::Abs(b.rot[7]), simul::Abs(b.rot[8]))) > 1.0 - tol;   // 월드 z 가 몸체 한 축과 나란하다
}
double run(simul::dynamics::RigidFall& b, double maxSeconds) {
    double t = 0;
    const double dt = 1.0 / 240.0;
    while (t < maxSeconds && b.Step(dt, -9.80665)) t += dt;
    return t;
}
}  // namespace

void testRigidFall() {
    using simul::dynamics::RigidFall;
    // 바닥에 반듯하게 떨어뜨린 상자: 회전 없이 면으로 선다 (x, y 도 그대로)
    {
        RigidFall b;
        b.SetShape(unitCube());
        const double p[3] = {0.3, -0.2, 1.5};
        double r[9];
        rotX(0.0, r);
        b.SetPose(p, r);
        const double t = run(b, 20.0);
        check(b.AtRest() && t < 19.0, "flat box comes to rest");
        check(nearly(b.pos[2], 0.5, 2e-3), "flat box rests on its face (center 0.5 m up)");
        check(nearly(b.pos[0], 0.3, 1e-3) && nearly(b.pos[1], -0.2, 1e-3), "flat drop does not slide sideways");
        check(faceDown(b, 1e-4), "flat drop does not rotate");
    }
    // 기울여 떨어뜨리면 모서리로 닿아 구르다 한 면으로 선다
    {
        const double tilts[4][2] = {{0.5, 0.0}, {0.6, 0.4}, {0.78, 0.6}, {1.2, -0.7}};
        for (int k = 0; k < 4; k++) {
            RigidFall b;
            b.SetShape(unitCube());
            const double p[3] = {0.0, 0.0, 1.5};
            double r[9];
            rotXY(tilts[k][0], tilts[k][1], r);
            b.SetPose(p, r);
            const double t = run(b, 30.0);
            check(b.AtRest() && t < 29.0, "tilted box comes to rest");
            check(faceDown(b, 2e-3), "tilted box ends up on a face");
            check(nearly(b.LowestPointZ(), 0.0, 1e-3), "resting box touches the floor, not above it or inside");
            check(b.ContactCount(1e-3) >= 4, "resting on a face touches with four corners");
        }
    }
    // 모서리로 선 상자는 그대로 서 있지 못하고 쓰러진다 (옛 낙하 계산은 기운 채 멈췄다)
    {
        RigidFall b;
        b.SetShape(unitCube());
        double r[9];
        rotXY(0.7853981634, 0.6154797087, r);   // 한 꼭짓점이 바로 아래 (대각선이 수직)
        r[0] = r[0];
        const double p[3] = {0.0, 0.0, 0.9};
        b.SetPose(p, r);
        run(b, 30.0);
        check(b.AtRest() && faceDown(b, 2e-3), "a box balanced near a corner topples onto a face");
    }
    // 공중의 회전: 각속도가 일정하면 회전각이 w t 이고 (감쇠 0) 에너지가 같다
    {
        RigidFall b;
        b.SetShape(unitCube(1.0, 1.0, 1.0));
        b.angularDamping = 0.0;
        const double p[3] = {0.0, 0.0, 100.0};
        double r[9];
        rotX(0.0, r);
        b.SetPose(p, r);
        const double v[3] = {0.0, 0.0, 0.0}, w[3] = {0.0, 0.0, 2.0};
        b.SetVelocity(v, w);
        for (int i = 0; i < 240; i++) b.Step(1.0 / 240.0, -9.80665);   // 1 s
        check(nearly(b.rot[0], simul::Cos(2.0), 1e-4) && nearly(b.rot[3], simul::Sin(2.0), 1e-4), "free spin about z turns by w t");
        check(nearly(b.omega[2], 2.0, 1e-9), "spin about a symmetry axis is constant");
        check(nearly(b.vel[2], -9.80665, 1e-6), "falling speed is g t while spinning");
    }
    // 튕김: 완전 탄성, 마찰 없음, 평평한 낙하는 거의 같은 높이까지 올라온다 (에너지가 늘지 않는다)
    {
        RigidFall b;
        b.SetShape(unitCube());
        b.restitution = 1.0;
        b.friction = 0.0;
        b.angularDamping = 0.0;
        const double p[3] = {0.0, 0.0, 2.0};
        double r[9];
        rotX(0.0, r);
        b.SetPose(p, r);
        double peak = 0.0;
        bool bounced = false;
        for (int i = 0; i < 240 * 3; i++) {
            b.Step(1.0 / 240.0, -9.80665);
            if (b.vel[2] > 0.1) bounced = true;
            if (bounced) peak = simul::Max(peak, b.pos[2]);
        }
        check(bounced && peak > 1.8 && peak < 2.05, "elastic flat drop returns to about the starting height and not above");
    }
    // 던진 상자(옆 속도 + 회전)도 결국 멈추고, 멈춘 뒤에는 움직이지 않는다
    {
        RigidFall b;
        b.SetShape(unitCube(0.4, 0.2, 0.1));
        const double p[3] = {0.0, 0.0, 1.0};
        double r[9];
        rotX(0.3, r);
        b.SetPose(p, r);
        const double v[3] = {1.0, 0.5, 0.0}, w[3] = {3.0, -2.0, 4.0};
        b.SetVelocity(v, w);
        const double t = run(b, 60.0);
        check(b.AtRest() && t < 59.0, "a thrown, spinning plate comes to rest");
        check(faceDown(b, 2e-3) && nearly(b.LowestPointZ(), 0.0, 1e-3), "the plate lies flat on the floor");
        const double z = b.pos[2];
        b.Step(0.5, -9.80665);
        check(b.pos[2] == z, "a resting body stays put");
    }
    // 접촉 후보 점 고르기: 많은 꼭짓점에서도 바운딩 상자의 꼭짓점은 남는다
    {
        simul::Array<double> pts;
        for (int i = 0; i < 2000; i++) {
            const double a = 0.0031 * i, c = 1.0 + 0.0007 * i;
            pts.push_back(c * simul::Cos(a));
            pts.push_back(c * simul::Sin(a));
            pts.push_back(((i % 7) - 3) * 0.1);
        }
        for (int s = 0; s < 8; s++) {   // 상자 꼭짓점
            pts.push_back((s & 1 ? 1 : -1) * 3.0);
            pts.push_back((s & 2 ? 1 : -1) * 2.0);
            pts.push_back((s & 4 ? 1 : -1) * 1.0);
        }
        simul::Array<double> sel;
        simul::dynamics::SelectContactPoints(&pts[0], pts.size() / 3, &sel);
        int corners = 0;
        for (size_t i = 0; i + 2 < sel.size(); i += 3)
            if (simul::Abs(simul::Abs(sel[i]) - 3.0) < 1e-9 && simul::Abs(simul::Abs(sel[i + 1]) - 2.0) < 1e-9 && simul::Abs(simul::Abs(sel[i + 2]) - 1.0) < 1e-9) corners++;
        check(sel.size() / 3 <= 76 && sel.size() / 3 >= 8 && corners == 8, "contact point selection keeps the extreme corners and stays small");
    }
}


// ---- 강체끼리의 충돌 (볼록 껍질 접촉)
namespace {
double runBodies(simul::dynamics::RigidFall** b, size_t n, double maxSeconds) {
    double t = 0;
    const double dt = 1.0 / 240.0;
    while (t < maxSeconds && simul::dynamics::StepRigidBodies(b, n, dt, -9.80665)) t += dt;
    return t;
}
void placeBox(simul::dynamics::RigidFall& b, double x, double y, double z, double sx = 1.0, double sy = 1.0, double sz = 1.0) {
    b.SetShape(unitCube(sx, sy, sz));
    double r[9];
    rotX(0.0, r);
    const double p[3] = {x, y, z};
    b.SetPose(p, r);
}
// 두 축 정렬 상자의 겹친 깊이(음수면 떨어져 있다): 가장 얕은 축 방향의 겹침
double overlapDepth(const simul::dynamics::RigidFall& a, const simul::dynamics::RigidFall& b, double sa[3], double sb[3]) {
    double best = 1e9;
    for (int k = 0; k < 3; k++) {
        const double o = 0.5 * (sa[k] + sb[k]) - simul::Abs(a.pos[k] - b.pos[k]);
        best = simul::Min(best, o);
    }
    return best;
}
}  // namespace

void testRigidCollisions() {
    using simul::dynamics::RigidFall;
    using simul::dynamics::StepRigidBodies;
    // 바닥에 놓인 상자 위에 다른 상자를 떨어뜨린다: 위에 쌓여 멈춘다 (아래 상자는 그대로)
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.5);
        placeBox(b, 0.1, -0.05, 3.0);
        RigidFall* bodies[2] = {&a, &b};
        runBodies(bodies, 2, 3.0);   // a 가 먼저 잠들 시간 (b 는 이미 떨어지는 중)
        const double t = runBodies(bodies, 2, 30.0);
        check(a.asleep && b.asleep && t < 29.0, "a box dropped on another box stacks and both come to rest");
        check(nearly(b.pos[2], 1.5, 0.01), "the upper box rests on top of the lower one (center 1.5 m)");
        check(nearly(a.pos[2], 0.5, 0.01) && simul::Abs(a.pos[0]) < 0.02 && simul::Abs(a.pos[1]) < 0.02, "the lower box stays");
        check(faceDown(b, 2e-3), "the upper box lies flat");
    }
    // 가장자리 너머로 떨어뜨리면 쌓이지 못하고 미끄러져 바닥에 내려앉는다 (서로 겹치지 않는다)
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.5);
        placeBox(b, 0.8, 0.0, 2.5);
        RigidFall* bodies[2] = {&a, &b};
        const double t = runBodies(bodies, 2, 40.0);
        double sz[3] = {1, 1, 1};
        check(a.asleep && b.asleep && t < 39.0, "a box dropped over the edge ends up resting");
        check(overlapDepth(a, b, sz, sz) < 0.01, "the two boxes do not overlap at the end");
        check(nearly(b.LowestPointZ(), 0.0, 2e-3) || nearly(b.pos[2], 1.5, 0.02), "the box is on the floor or on top, never inside");
    }
    // 달려와 부딪히면 잠든 상자가 깨어나 밀려난다 (운동량이 옮는다)
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.5);
        placeBox(b, 1.6, 0.0, 0.5);
        RigidFall* bodies[2] = {&a, &b};
        runBodies(bodies, 2, 1.0);                      // 서로 멈춰 선다
        check(a.asleep && b.asleep, "both boxes are asleep before the hit");
        b.asleep = false;
        const double lin[3] = {-4.0, 0.0, 0.0}, ang[3] = {0.0, 0.0, 0.0};
        b.SetVelocity(lin, ang);
        double maxV = 0.0;
        for (int i = 0; i < 240 * 3; i++) {
            StepRigidBodies(bodies, 2, 1.0 / 240.0, -9.80665);
            maxV = simul::Max(maxV, -a.vel[0]);
        }
        check(maxV > 1.0, "the resting box is hit and pushed away (it woke up)");
        check(a.pos[0] < -0.2, "the box that was hit has moved");
        double sz[3] = {1, 1, 1};
        check(overlapDepth(a, b, sz, sz) < 0.01, "they do not overlap after the hit");
    }
    // collide = false 이면 서로 통과한다
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.5);
        placeBox(b, 0.0, 0.0, 2.0);
        a.collide = b.collide = false;
        RigidFall* bodies[2] = {&a, &b};
        runBodies(bodies, 2, 20.0);
        check(nearly(b.pos[2], 0.5, 0.01), "without collisions the boxes fall through each other");
    }
    // 고정된 몸체 위에도 쌓이고, 고정된 몸체는 밀리지 않는다
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 1.0, 2.0, 2.0, 0.2);   // 공중에 고정된 판 (윗면 1.1)
        a.fixed = true;
        placeBox(b, 0.2, 0.0, 3.0, 0.4, 0.4, 0.4);
        RigidFall* bodies[2] = {&a, &b};
        const double t = runBodies(bodies, 2, 30.0);
        check(b.asleep && t < 29.0, "a box dropped on a fixed plate comes to rest");
        check(nearly(b.pos[2], 1.3, 0.01) && nearly(a.pos[2], 1.0, 1e-12), "it rests on the plate and the plate did not move");
    }
    // 빠르게 떨어져도 얇은 판을 뚫지 않는다
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.05, 3.0, 3.0, 0.1);
        placeBox(b, 0.0, 0.0, 6.0, 0.3, 0.3, 0.3);
        RigidFall* bodies[2] = {&a, &b};
        runBodies(bodies, 2, 20.0);
        check(b.pos[2] > 0.1, "a fast box does not tunnel through a thin plate");
    }
    // 깊이 겹쳐 시작해도 서서히 밀려나 겹침이 풀린다 (튀어 오르지 않는다)
    {
        RigidFall a, b;
        placeBox(a, 0.0, 0.0, 0.5);
        placeBox(b, 0.0, 0.0, 1.3);                 // 0.2 겹침
        RigidFall* bodies[2] = {&a, &b};
        double maxV = 0.0;
        for (int i = 0; i < 240 * 5; i++) {
            StepRigidBodies(bodies, 2, 1.0 / 240.0, -9.80665);
            maxV = simul::Max(maxV, b.vel[2]);
        }
        check(nearly(b.pos[2], 1.5, 0.02) && maxV < 1.6, "an initial overlap is pushed apart smoothly");
    }
}


// ---- 바닥 위 자유낙하
void testFreeFall() {
    using simul::dynamics::FallBody;
    const double g = -9.80665;
    // 1 m 높이에서 튕김 없이 떨어뜨린다: 낙하 시간은 sqrt(2h/|g|)
    FallBody b;
    b.height = 1.0;
    b.restitution = 0.0;
    const double dt = 1.0 / 1000.0;
    double t = 0;
    int steps = 0;
    while (b.Step(dt, g) && steps < 100000) {
        t += dt;
        steps++;
    }
    t += dt;  // 마지막 스텝(닿는 스텝)
    check(steps < 100000 && simul::Abs(t - simul::Sqrt(2.0 * 1.0 / 9.80665)) < 3 * dt, "fall time matches sqrt(2h/g)");
    check(b.height == 0.0 && b.velocity == 0.0 && b.AtRest(), "comes to rest on the floor");
    // 도중의 높이도 h0 - g t^2 / 2 와 같다 (등가속도 식은 정확하다)
    FallBody c;
    c.height = 2.0;
    for (int i = 0; i < 200; i++) c.Step(0.002, g);  // 0.4 s
    check(simul::Abs(c.height - (2.0 + 0.5 * g * 0.4 * 0.4)) < 1e-9 && simul::Abs(c.velocity - g * 0.4) < 1e-9, "mid-air state is exact");
    // 튕김: 되튀는 속도는 e 배, 올라가는 높이는 e^2 배
    FallBody d;
    d.height = 1.0;
    d.restitution = 0.5;
    double peak = 0, prev = d.height;
    bool hit = false;
    for (int i = 0; i < 20000 && !(hit && d.velocity <= 0); i++) {
        d.Step(1.0 / 20000.0, g);
        if (d.velocity > 0) {
            hit = true;
            peak = simul::Max(peak, d.height);
        }
        prev = d.height;
    }
    (void)prev;
    check(hit && simul::Abs(peak - 0.25) < 0.01, "bounce peak is e^2 * h0");
    // 계속 튕기다가 결국 멈춘다
    FallBody e;
    e.height = 3.0;
    int n = 0;
    while (e.Step(1.0 / 240.0, g) && n < 240 * 60) n++;
    check(n < 240 * 60 && e.AtRest(), "bouncing eventually stops");
    // 안전: 바닥 아래, 중력 0 이상, NaN, dt <= 0
    FallBody u;
    u.height = -0.5;
    check(!u.Step(0.01, g) && u.height == -0.5, "below the floor is left alone");
    FallBody z;
    z.height = 1.0;
    check(!z.Step(0.01, 0.0) && z.height == 1.0, "no gravity, no fall");
    check(!z.Step(0.01, 9.8) && z.height == 1.0, "upward gravity does not pull it down");
    check(!z.Step(0.0, g) && !z.Step(-1.0, g), "dt <= 0 does nothing");
    FallBody nn;
    nn.height = NAN;
    check(!nn.Step(0.01, g), "NaN height does not move");
    FallBody big;
    big.height = 100.0;
    big.Step(50.0, g);  // 너무 긴 dt 는 0.1 s 로 잘린다
    check(simul::Abs(big.height - (100.0 + 0.5 * g * 0.01)) < 1e-9, "huge dt is clamped to 0.1 s");
    FallBody flat;  // 바닥에 놓인 물체
    check(!flat.Step(0.01, g) && flat.height == 0.0, "an object on the floor stays");
}

// ---- 질량 행렬, 역동역학(RNEA), 순동역학, 시뮬레이터
// kTree 의 모든 링크에 (회전된 질량중심과 함께 효과를 내는) 관성 텐서를 넣은 버전
simul::String treeWithInertia(const char* inertiaTag = "<inertia ixx='0.02' ixy='0.003' ixz='-0.002' iyy='0.03' iyz='0.001' izz='0.025'/>") {
    simul::String s = kTree;
    const simul::String in = simul::String(inertiaTag) + "</inertial>";
    size_t pos = 0;
    while ((pos = s.find("</inertial>", pos)) != simul::String::npos) {
        s.replace(pos, 11, in);
        pos += in.size();
    }
    return s;
}

urdf::JointValues randomTreePose(Rng& rng) {
    urdf::JointValues q;
    q["j1"] = static_cast<float>(rng.range(-2, 2));
    q["j2"] = static_cast<float>(rng.range(-0.15, 0.45));
    q["j4"] = static_cast<float>(rng.range(-2, 2));
    q["j5"] = static_cast<float>(rng.range(-2, 2));
    q["j7"] = static_cast<float>(rng.range(-3, 3));
    return q;
}

void testRneaAndMass() {
    const simul::String xml = treeWithInertia();
    const urdf::Robot r = load(xml.c_str());
    const simul::Array<simul::String> names = ActuatedJointNames(r);
    check(names.size() == 5 && names[0] == "j1" && names[4] == "j7", "actuated joint names");
    Rng rng(99);
    double worstStatic = 0, worstSym = 0, worstId = 0;
    bool pd = true;
    for (int iter = 0; iter < 60; iter++) {
        const urdf::JointValues q = randomTreePose(rng);
        // 1) 정지 상태의 RNEA = 중력 토크
        const JointTorques zero;
        const JointTorques id = InverseDynamics(r, q, zero, zero);
        const JointTorques gt = GravityTorques(r, q);
        for (size_t i = 0; i < names.size(); i++) worstStatic = simul::Max(worstStatic, simul::Abs(id.at(names[i]) - gt.at(names[i])));
        // 2) 질량 행렬: 대칭, 양의 정부호
        const MassMatrix m = ComputeMassMatrix(r, q);
        const size_t n = m.n();
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++) worstSym = simul::Max(worstSym, simul::Abs(m.At(i, j) - m.At(j, i)));
        for (int k = 0; k < 20; k++) {
            simul::Array<double> x(n);
            for (size_t i = 0; i < n; i++) x[i] = rng.range(-1, 1);
            double e = 0;
            for (size_t i = 0; i < n; i++)
                for (size_t j = 0; j < n; j++) e += x[i] * m.At(i, j) * x[j];
            if (!(e > 1e-6)) pd = false;
        }
        // 3) 중력 없이 속도 0 일 때 RNEA(qdd) = M qdd
        JointTorques qdd;
        for (size_t i = 0; i < n; i++) qdd[names[i]] = rng.range(-3, 3);
        const JointTorques tau = InverseDynamics(r, q, zero, qdd, Gravity(0, 0, 0));
        for (size_t i = 0; i < n; i++) {
            double mq = 0;
            for (size_t j = 0; j < n; j++) mq += m.At(i, j) * qdd.at(names[j]);
            worstId = simul::Max(worstId, simul::Abs(tau.at(names[i]) - mq));
        }
    }
    check(worstStatic < 1e-9, "RNEA at rest equals GravityTorques");
    check(worstSym < 1e-9, "mass matrix is symmetric");
    check(pd, "mass matrix is positive definite");
    check(worstId < 1e-9, "RNEA(qdd, no gravity) = M qdd");
    // 기구학 하나만 따로: 팔 길이 L, 점질량 m (관성 0) 이면 M = m L^2
    const urdf::Robot pend = load("<robot name='p'><link name='base'/><link name='arm'><inertial><origin xyz='0.5 0 0'/><mass value='2'/></inertial></link>"
                                  " <joint name='j' type='revolute'><parent link='base'/><child link='arm'/><axis xyz='0 1 0'/>"
                                  " <limit lower='-3.2' upper='3.2' effort='50' velocity='1'/></joint></robot>");
    check(nearly(ComputeMassMatrix(pend, urdf::JointValues()).At(0, 0), 2 * 0.25, 1e-12), "point mass: M = m L^2");
    // 관성 텐서가 더해진다: 축 둘레 관성 + m L^2
    const urdf::Robot pendI = load(kPendulum);
    check(nearly(ComputeMassMatrix(pendI, urdf::JointValues()).At(0, 0), 0.1 + 2 * 0.25, 1e-6), "M = I + m L^2 (parallel axis theorem)");
}

// 코리올리/원심력(자이로 항 포함)을 질량 행렬의 수치 미분으로 독립적으로 구해서 RNEA 와 맞춘다:
//   h_i = sum_jk (dM_ij/dq_k) qd_j qd_k  -  1/2 sum_jk (dM_jk/dq_i) qd_j qd_k
void testCoriolisAgainstMassDerivative() {
    const simul::String xml = treeWithInertia("<inertia ixx='0.6' ixy='0.1' ixz='-0.15' iyy='1.2' iyz='0.2' izz='0.9'/>");
    const urdf::Robot r = load(xml.c_str());
    const simul::Array<simul::String> names = ActuatedJointNames(r);
    const size_t n = names.size();
    Rng rng(21);
    double worst = 0, scale = 0;
    for (int iter = 0; iter < 25; iter++) {
        const urdf::JointValues q = randomTreePose(rng);
        JointTorques qd, zero;
        for (size_t i = 0; i < n; i++) qd[names[i]] = rng.range(-3, 3);
        // dM/dq_k (중앙 차분)
        simul::Array<MassMatrix> dM;
        const float eps = 2e-3f;
        for (size_t k = 0; k < n; k++) {
            urdf::JointValues a = q, b = q;
            a[names[k]] += eps;
            b[names[k]] -= eps;
            MassMatrix ma = ComputeMassMatrix(r, a, false), mb = ComputeMassMatrix(r, b, false);
            for (size_t e = 0; e < ma.m.size(); e++) ma.m[e] = (ma.m[e] - mb.m[e]) / (2.0 * static_cast<double>(eps));
            dM.push_back(ma);
        }
        // 속도 항: 중력 없이 qdd = 0 으로 RNEA
        const JointTorques h = InverseDynamics(r, q, qd, zero, Gravity(0, 0, 0), false);
        for (size_t i = 0; i < n; i++) {
            double ref = 0;
            for (size_t j = 0; j < n; j++)
                for (size_t k = 0; k < n; k++) {
                    ref += dM[k].At(i, j) * qd.at(names[j]) * qd.at(names[k]);
                    ref -= 0.5 * dM[i].At(j, k) * qd.at(names[j]) * qd.at(names[k]);
                }
            worst = simul::Max(worst, simul::Abs(h.at(names[i]) - ref));
            scale = simul::Max(scale, simul::Abs(ref));
        }
    }
    check(scale > 1.0, "velocity terms are large enough to matter");
    check(worst < 2e-3 * scale, "Coriolis/centrifugal/gyroscopic terms match the mass-matrix derivative");
}

void testSimulator() {
    // 1) 에너지 보존 (감쇠 0, 모터 꺼짐): 코리올리/원심력, 이동 관절, mimic, 회전된 관성 모두 맞아야 유지된다
    {
        // limit 에 부딪히면 에너지가 사라지므로 이 시험에서는 limit 을 넉넉하게 넓힌다 (limit 동작은 아래 6번에서 따로 본다)
        simul::String xml = treeWithInertia();
        static const char* const kLims[2] = {"lower='-3' upper='3'", "lower='-0.2' upper='0.5'"};
        for (int li = 0; li < 2; li++) {
            const char* lim = kLims[li];
            size_t pos = 0;
            while ((pos = xml.find(lim, pos)) != simul::String::npos) xml.replace(pos, simul::String(lim).size(), "lower='-100' upper='100'");
        }
        const urdf::Robot r = load(xml.c_str());
        Rng rng(5);
        urdf::JointValues q = randomTreePose(rng);
        q["j2"] = 0.15f;
        Simulator sim;
        sim.Reset(r, q);
        sim.SetOptions(DynamicsOptions());
        // 초기 속도: SetValue 는 속도를 0 으로 하므로, 한 번 흔들어서 속도를 만든 뒤 시작한다
        sim.Step(0.15);
        const double e0 = sim.TotalEnergy();
        double worst = 0, kmax = 0;
        for (int i = 0; i < 100; i++) {
            sim.Step(0.01);
            worst = simul::Max(worst, simul::Abs(sim.TotalEnergy() - e0));
            kmax = simul::Max(kmax, sim.KineticEnergy());
        }
        check(kmax > 0.2, "the arm really moves (kinetic energy builds up)");
        check(worst < 1e-4, "energy is conserved without damping (Coriolis, prismatic, mimic, inertia tensors)");
    }
    // 2) 진자: 평형(수직 아래, 90도) 주변의 작은 진동 주기 T = 2 pi sqrt(M / (m g L))
    {
        const urdf::Robot r = load(kPendulum);
        const double Mi = 0.1 + 2 * 0.25, w2 = 2 * G * 0.5 / Mi, T = 2 * 3.14159265358979 / simul::Sqrt(w2);
        const double th0 = 3.14159265358979 / 2 + 0.02;
        urdf::JointValues q;
        q["j"] = static_cast<float>(th0);
        Simulator sim;
        sim.Reset(r, q);
        sim.Step(T / 2);
        check(simul::Abs(static_cast<double>(sim.Values().at("j")) - (3.14159265358979 - th0)) < 1e-4, "pendulum is at the other side after half a period");
        sim.Step(T / 2);
        check(simul::Abs(static_cast<double>(sim.Values().at("j")) - th0) < 1e-4, "pendulum returns after one period");
    }
    // 3) 점성 감쇠: 중력 없이 qd(t) = qd0 exp(-d t / M)
    {
        const simul::String xml = simul::String(kPendulum);
        simul::String damped = xml;
        damped.insert(damped.find("</joint>"), "<dynamics damping='0.3' friction='0'/>");
        const urdf::Robot r = load(damped.c_str());
        Simulator sim;
        sim.Reset(r, urdf::JointValues());
        sim.SetGravity(Gravity(0, 0, 0));
        // 속도를 주기 위해 토크로 한 번 가속한다: tau = M * a  (감쇠가 있으니 아주 짧게)
        JointTorques tau;
        tau["j"] = 0.6 * 100.0;
        sim.SetTorques(tau);
        sim.Step(0.01);
        sim.SetTorques(JointTorques());
        const double v0 = sim.Velocities().at("j");
        sim.Step(1.0);
        const double v1 = sim.Velocities().at("j");
        check(v0 > 0.5 && simul::Abs(v1 / v0 - simul::Exp(-0.3 * 1.0 / 0.6)) < 1e-3, "viscous damping decays velocity exponentially");
        // defaultDamping 옵션은 감쇠가 없는 관절에 같은 효과를 준다
        const urdf::Robot plain = load(kPendulum);
        Simulator s2;
        s2.Reset(plain, urdf::JointValues());
        s2.SetGravity(Gravity(0, 0, 0));
        DynamicsOptions o;
        o.defaultDamping = 0.3;
        s2.SetOptions(o);
        s2.SetTorques(tau);
        s2.Step(0.01);
        s2.SetTorques(JointTorques());
        const double w0 = s2.Velocities().at("j");
        s2.Step(1.0);
        check(simul::Abs(s2.Velocities().at("j") / w0 - simul::Exp(-0.3 / 0.6)) < 1e-3, "defaultDamping applies to joints without <dynamics>");
    }
    // 4) 쿨롱 마찰: 일정한 감속 f / M
    {
        simul::String fr = kPendulum;
        fr.insert(fr.find("</joint>"), "<dynamics damping='0' friction='0.5'/>");
        const urdf::Robot r = load(fr.c_str());
        Simulator sim;
        sim.Reset(r, urdf::JointValues());
        sim.SetGravity(Gravity(0, 0, 0));
        JointTorques tau;
        tau["j"] = 0.6 * 100.0;
        sim.SetTorques(tau);
        sim.Step(0.01);
        const double v0 = sim.Velocities().at("j");  // 마찰이 있어서 약간 모자란다
        sim.SetTorques(JointTorques());
        sim.Step(0.4);
        check(simul::Abs(sim.Velocities().at("j") - (v0 - 0.5 / 0.6 * 0.4)) < 2e-3, "Coulomb friction decelerates at f / M");
        sim.Step(5.0);
        check(simul::Abs(sim.Velocities().at("j")) < 1e-3, "and the joint stops");
    }
    // 5) 모터 토크: 중력 없이 일정 토크 -> qdd = tau / M
    {
        const urdf::Robot r = load(kPendulum);
        JointTorques tau, qd;
        tau["j"] = 1.2;
        const JointTorques a = ForwardDynamics(r, urdf::JointValues(), qd, tau, Gravity(0, 0, 0));
        check(nearly(a.at("j"), 1.2 / 0.6, 1e-6), "qdd = tau / M");
        // 중력이 있고 수평이면 모터가 g 토크를 정확히 주면 가속도 0 (모터가 버틴다)
        const JointTorques hold = GravityTorques(r, urdf::JointValues());
        check(nearly(ForwardDynamics(r, urdf::JointValues(), qd, hold).at("j"), 0.0, 1e-6), "holding torque gives zero acceleration");
        // 모터가 꺼지면 수평 팔은 아래로(+y 축 회전의 + 방향) 처진다
        check(ForwardDynamics(r, urdf::JointValues(), qd, JointTorques()).at("j") > 0.0, "unpowered horizontal arm starts to fall");
    }
    // 6) limit: 위쪽 limit 에 닿으면 멈춘다
    {
        const urdf::Robot r = load(kPendulum);
        urdf::JointValues q;
        q["j"] = 3.1f;
        Simulator sim;
        sim.Reset(r, q);
        sim.SetGravity(Gravity(0, 0, 0));
        JointTorques tau;
        tau["j"] = 20.0;
        sim.SetTorques(tau);
        sim.Step(0.5);
        check(sim.Values().at("j") <= 3.2f + 1e-6f && sim.Velocities().at("j") <= 1e-9, "joint stops at its upper limit");
        sim.SetValue("j", 100.0);
        check(sim.Values().at("j") <= 3.2f + 1e-6f, "SetValue clamps to the limit");
    }
    // 7) 안전: 질량 없는 로봇, 준비 안 된 시뮬레이터, 구동 관절 없음
    {
        const urdf::Robot massless = load("<robot name='m'><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/>"
                                          "<limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>");
        JointTorques none;
        const JointTorques a = ForwardDynamics(massless, urdf::JointValues(), none, none);
        check(a.size() == 1 && a.at("j") == 0.0, "massless joint does not accelerate (and gives no NaN)");
        Simulator sim;
        sim.Step(1.0);
        check(!sim.IsReady() && sim.Values().empty(), "unready simulator does nothing");
        sim.Reset(massless, urdf::JointValues());
        sim.Step(1.0);
        check(simul::IsFinite(sim.TotalEnergy()) && sim.Values().at("j") == 0.0f, "massless robot stays put");
        const urdf::Robot fixedOnly = load("<robot name='f'><link name='a'/><link name='b'/><joint name='j' type='fixed'><parent link='a'/><child link='b'/></joint></robot>");
        sim.Reset(fixedOnly, urdf::JointValues());
        check(!sim.IsReady(), "no actuated joint means not ready");
    }
}

// limit 에 닿은 관절: 그 관절은 멈추고, 매달린 관절은 자기 운동량을 유지한 채 계속 움직여야 한다 (둘 다 같이 멈추면 안 된다)
void testJointLimitImpact() {
    const urdf::Robot r = load(
        "<robot name='two'><link name='base'/>"
        " <link name='a'><inertial><origin xyz='0.5 0 0'/><mass value='2'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.1' iyz='0' izz='0.1'/></inertial></link>"
        " <link name='b'><inertial><origin xyz='0.5 0 0'/><mass value='2'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.1' iyz='0' izz='0.1'/></inertial></link>"
        " <joint name='j1' type='revolute'><parent link='base'/><child link='a'/><axis xyz='0 1 0'/>"
        "  <limit lower='-3' upper='0.5' effort='50' velocity='5'/></joint>"
        " <joint name='j2' type='revolute'><parent link='a'/><child link='b'/><origin xyz='1 0 0'/><axis xyz='0 1 0'/>"
        "  <limit lower='-3' upper='3' effort='50' velocity='5'/></joint></robot>");
    Simulator sim;
    sim.Reset(r, urdf::JointValues());
    sim.SetOptions(DynamicsOptions());
    // 중력이 팔을 + 방향(아래)으로 끌어서 j1 이 위쪽 limit(0.5) 에 닿는다
    double maxJ2Speed = 0;
    for (int i = 0; i < 600; i++) {
        sim.Step(1.0 / 60.0);
        const JointTorques v = sim.Velocities();
        if (sim.Values().at("j1") >= 0.5f - 1e-6f) maxJ2Speed = simul::Max(maxJ2Speed, simul::Abs(v.at("j2")));
        check(sim.Values().at("j1") <= 0.5f + 1e-6f, "j1 never passes its limit");
    }
    check(sim.Values().at("j1") >= 0.49f, "j1 reaches the limit");
    check(maxJ2Speed > 0.5, "j2 keeps swinging after j1 hits the limit (momentum is not wiped out)");
    check(simul::IsFinite(sim.TotalEnergy()), "energy stays finite");
}

// 시뮬레이션 도중 모델을 돌리면(attach 관절 origin 이 바뀜) UpdateRobot 으로 새 배치를 알리면 중력은 월드 -Z 기준으로 새 배치에 작용한다.
// 샘플 팔을 X 축으로 90° 눕히면 shoulder_pitch 축(y)이 수직이 되어 처지지 않고, base_yaw 축(수평)이 처진다.
void testGravityFollowsModelRotation() {
    const simul::String file = simul::String(SIMUL_TEST_DATA_DIR) + "/Object/samples/arm.urdf";
    simul::String err;
    urdf::World upright;
    check(upright.Add(file, &err), "add arm");
    upright.SetForceComposite(true);
    urdf::Robot a;
    check(upright.Build(urdf::LoadOptions(), &a, &err), "build upright");
    urdf::World lying;
    lying.Add(file, &err);
    lying.SetForceComposite(true);
    lying.SetPlacement(0, dhMat::Quat(1.5707963f, dhVector(1, 0, 0)));
    urdf::Robot b;
    check(lying.Build(urdf::LoadOptions(), &b, &err), "build lying");

    Simulator sim;
    sim.Reset(a, urdf::JointValues());
    sim.SetOptions(DynamicsOptions());
    sim.UpdateRobot(b);   // 시뮬레이션 도중 모델을 눕혔다
    for (int i = 0; i < 30; i++) sim.Step(1.0 / 60.0);
    const urdf::JointValues q = sim.Values();
    check(simul::Abs(q.at("shoulder_pitch")) < 1e-4f, "lying arm: vertical shoulder axis does not sag");
    check(simul::Abs(q.at("base_yaw")) > 0.1f, "lying arm: horizontal base axis sags under world gravity");
}

}  // namespace

int main() {
    testGravityFollowsModelRotation();
    testJointLimitImpact();
    testFreeFall();
    testRigidFall();
    testRigidCollisions();
    testRneaAndMass();
    testCoriolisAgainstMassDerivative();
    testSimulator();
    testPendulum();
    testPrismatic();
    testGradientOfPotential();
    testStructure();
    testCenterOfMass();
    testPerJointLoads();
    testContributions();
    if (g_fail == 0) printf("test_cdynamics: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
