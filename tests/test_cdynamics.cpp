// CDynamics 테스트: 중력 토크. 해석해(진자, 슬라이더), 위치에너지 기울기(독립 구현인 float FK 로 수치 미분)와의 대조,
// 중력 방향 변화, mimic 합산, 관절 제한, 질량중심
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "CDynamics/Gravity.h"
#include "CURDF/Urdf.h"

using namespace amr;
using namespace amr::dynamics;

namespace {

int g_fail = 0;
const double G = kStandardGravity;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-5) { return std::fabs(a - b) <= eps; }

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
    std::string err;
    if (!urdf::LoadUrdfFromString(xml, &r, &err)) {
        std::printf("URDF load failed: %s\n", err.c_str());
        std::exit(2);
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
        check(nearly(GravityTorques(r, q)["j"], -2 * G * 0.5 * std::cos(th), 2e-5), "pendulum tau = -m g L cos(theta)");
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
        const double expected = -2 * 0.5 * G * std::cos(th - f);
        check(nearly(GravityTorques(r, q, Gravity(-G * std::sin(f), 0, -G * std::cos(f)))["j"], expected, 3e-5), "tilted gravity vector");
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
    const std::vector<dhMat> T = r.LinkTransforms(q);
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
    const std::vector<int> act = r.ActuatedJoints();
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
        const double n = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
        g = Gravity(g.x / n * G, g.y / n * G, g.z / n * G);

        const JointTorques t = GravityTorques(r, q, g);
        check(t.size() == 5, "torques for the 5 actuated joints only");
        for (size_t a = 0; a < act.size(); a++) {
            const std::string name = r.joints[static_cast<size_t>(act[a])].name;
            const double h = 0.01;
            urdf::JointValues qp = q, qm = q;
            qp[name] = q[name] + static_cast<float>(h);
            qm[name] = q[name] - static_cast<float>(h);
            const double fd = (potential(r, qp, g) - potential(r, qm, g)) / (2 * h);
            const double got = t.find(name)->second;
            const bool ok = std::fabs(got - fd) <= 0.01 + 0.004 * std::fabs(fd);
            if (!ok) std::printf("  joint %s: analytic %.5f vs gradient %.5f\n", name.c_str(), got, fd);
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


const JointLoad* findLoad(const std::vector<JointLoad>& v, const char* name) {
    for (size_t i = 0; i < v.size(); i++)
        if (v[i].name == name) return &v[i];
    return NULL;
}

double sumOf(const std::vector<LinkContribution>& v) {
    double s = 0;
    for (size_t i = 0; i < v.size(); i++) s += v[i].value;
    return s;
}

void testPerJointLoads() {
    // 진자: 힘은 위로 m g, 모멘트는 -y 방향으로 m g L, torque 는 그 모멘트의 y 성분
    const urdf::Robot p = load(kPendulum);
    std::vector<JointLoad> l = GravityLoads(p, urdf::JointValues());
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
    check(j6 && j6->movable && j6->mimic && std::fabs(j6->torque) > 1e-6, "j6 mimic has its own share");
    const JointTorques tm = GravityTorques(t, q);
    check(j4 && nearly(tm.find("j4")->second, j4->torque + (-0.5) * j6->torque, 1e-9), "master torque = own + multiplier * mimic's own");
    for (size_t i = 0; i < l.size(); i++) {
        if (!l[i].movable || l[i].mimic) continue;
        // mimic 이 달리지 않은 구동 관절은 GravityTorques 와 똑같다
        if (l[i].name != "j4") check(nearly(tm.find(l[i].name)->second, l[i].torque, 1e-9), "non-master actuated joint matches GravityTorques");
    }
    // 하중의 힘은 -M g: j1 아래쪽 전체 질량 (l1..l8 = 2+1.5+0.8+1.2+0.5+0.7+0.4 = 7.1 kg)
    const JointLoad* j1 = findLoad(l, "j1");
    check(j1 && nearly(std::sqrt(j1->force[0] * j1->force[0] + j1->force[1] * j1->force[1] + j1->force[2] * j1->force[2]), 7.1 * G, 1e-5), "|force| = M g");
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
    std::vector<LinkContribution> s = GravityContributions(two, urdf::JointValues(), "s");
    check(s.size() == 2 && s[0].link == "b" && s[1].link == "c", "shoulder: contributions from b and c (d has no mass)");
    check(s.size() == 2 && nearly(s[0].value, -G * 0.5) && nearly(s[1].value, -G * 1.5), "shoulder: b = -g*0.5, c = -g*1.5");
    const std::vector<LinkContribution> e = GravityContributions(two, urdf::JointValues(), "e");
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
        const std::vector<JointLoad> loads = GravityLoads(t, q, g);
        for (size_t i = 0; i < loads.size(); i++) {
            if (!loads[i].movable) continue;
            check(nearly(sumOf(GravityContributions(t, q, loads[i].name, g)), loads[i].torque, 1e-9), "sum of link contributions == joint torque");
        }
    }
}

}  // namespace

int main() {
    testPendulum();
    testPrismatic();
    testGradientOfPotential();
    testStructure();
    testCenterOfMass();
    testPerJointLoads();
    testContributions();
    if (g_fail == 0) std::printf("test_cdynamics: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
