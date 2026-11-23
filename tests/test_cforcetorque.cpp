// CSensor 힘/토크(F/T) 센서 테스트: 정지 무게/모멘트(해석해), 센서 자세, 가속 관성력, 외력, 부호, 편향/잡음/포화, URDF 읽기, 예외
#include <math.h>
#include <stdio.h>

#include "CBase/Math.h"
#include "CSensor/ForceTorque.h"
#include "CURDF/Urdf.h"

using namespace simul;
using namespace simul::sensor;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

const double G = dynamics::kStandardGravity;

// base --(slide: prismatic x, 센서 붙이기 좋은 fixed 'wrist')-- tool.
// tool: 2 kg, 질량중심 (0.1, 0, 0)
const char* kUrdf =
    "<robot name='r'>"
    " <link name='base'><inertial><mass value='5'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <link name='arm'><inertial><origin xyz='0 0 0'/><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <link name='tool'><inertial><origin xyz='0.1 0 0'/><mass value='2'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <joint name='slide' type='prismatic'><parent link='base'/><child link='arm'/><axis xyz='1 0 0'/><limit lower='-5' upper='5' effort='100' velocity='1'/></joint>"
    " <joint name='wrist' type='fixed'><parent link='arm'/><child link='tool'/><origin xyz='0 0 0'/></joint>"
    " <gazebo reference='wrist'><sensor name='ft' type='force_torque'><pose>0 0 0 0 0 0</pose>"
    "  <force_torque><measure_direction>child_to_parent</measure_direction></force_torque><noise><stddev>0.5</stddev><torque_stddev>0.05</torque_stddev></noise></sensor></gazebo>"
    "</robot>";

bool load(const char* xml, urdf::Robot* r) {
    String err;
    if (!urdf::LoadUrdfFromString(xml, r, &err)) {
        printf("load error: %s\n", err.c_str());
        return false;
    }
    return true;
}

void testStatic() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    FtSettings s;
    s.joint = "wrist";
    ForceTorque ft(s);
    const FtReading m = ft.MeasureStatic(r, urdf::JointValues());
    check(m.valid, "valid");
    // 아래쪽 전체(tool 2 kg)를 부모가 위로 받친다: Fz = +2 g
    check(near(m.force[0], 0, 1e-9) && near(m.force[1], 0, 1e-9) && near(m.force[2], 2 * G, 1e-4), "static force = +m g up");
    // 질량중심이 x=0.1: 모멘트 = r x F = (0.1,0,0) x (0,0,2g) = (0, -0.2 g, 0)
    check(near(m.torque[0], 0, 1e-6) && near(m.torque[1], -0.2 * G, 1e-4) && near(m.torque[2], 0, 1e-6), "static moment = r x F");
    check(near(m.ForceNorm(), 2 * G, 1e-4), "force norm");
}

void testDirectionAndPose() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    FtSettings s;
    s.joint = "wrist";
    s.direction = kFtChildToParent;
    FtReading m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(near(m.force[2], -2 * G, 1e-4), "child_to_parent flips sign");
    // 센서 원점을 질량중심으로 옮기면 모멘트가 0 (부호 반전 없이 보려고 parent_to_child)
    s.direction = kFtParentToChild;
    s.pose.v[12] = 0.1f;
    m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(near(m.torque[1], 0, 1e-4) && near(m.force[2], 2 * G, 1e-4), "sensor at COM: no moment");
    // 센서를 y 축으로 90도 돌리면 (Ry(90): 센서 x -> -z, 센서 z -> +x) 위쪽 힘은 센서 -x 방향
    s.pose = dhMat();
    s.pose.v[0] = 0; s.pose.v[2] = -1;   // 열 0 = 센서 x 축의 부모 좌표 = (0, 0, -1)
    s.pose.v[8] = 1; s.pose.v[10] = 0;   // 열 2 = 센서 z 축의 부모 좌표 = (1, 0, 0)
    m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(near(m.force[0], -2 * G, 1e-4) && near(m.force[2], 0, 1e-4), "rotated sensor frame");
}

void testAccelerationAndExternal() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    FtSettings s;
    s.joint = "wrist";
    ForceTorque ft(s);
    // slide 를 x 로 a = 3 m/s^2 가속: 아래쪽 (tool 2 kg) 에 필요한 x 힘 = 2 * 3
    dynamics::JointTorques qdd;
    qdd["slide"] = 3.0;
    FtReading m = ft.Measure(r, urdf::JointValues(), dynamics::JointTorques(), qdd);
    check(near(m.force[0], 6.0, 1e-6) && near(m.force[2], 2 * G, 1e-4), "inertial force while accelerating");
    // 중력 없음 + 가속 없음 = 0
    m = ft.Measure(r, urdf::JointValues(), dynamics::JointTorques(), dynamics::JointTorques(), dynamics::Gravity(0, 0, 0));
    check(near(m.ForceNorm(), 0, 1e-9) && near(m.TorqueNorm(), 0, 1e-9), "no gravity, no motion: zero");
    // 외력: tool 에 위로 5 N (질량중심에서 작용). 전달 하중이 5 N 줄어든다
    FtExternalWrench e;
    e.link = "tool";
    e.point[0] = 0.1;
    e.force[2] = 5.0;
    m = ft.MeasureStatic(r, urdf::JointValues(), dynamics::Gravity(), &e, 1);
    check(near(m.force[2], 2 * G - 5.0, 1e-4), "external force reduces transmitted load");
    check(near(m.torque[1], -(2 * G - 5.0) * 0.1, 1e-4), "external moment about sensor");
    // 센서 위쪽 링크(arm) 에 건 외력은 느끼지 못한다
    e.link = "arm";
    m = ft.MeasureStatic(r, urdf::JointValues(), dynamics::Gravity(), &e, 1);
    check(near(m.force[2], 2 * G, 1e-4), "external on parent side is ignored");
    check(IsBelowJoint(r, "wrist", "tool") && !IsBelowJoint(r, "wrist", "arm") && !IsBelowJoint(r, "wrist", "nope"), "IsBelowJoint");
}

void testNoiseBiasSaturation() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    FtSettings s;
    s.joint = "wrist";
    s.forceBias[2] = 1.0f;
    s.torqueBias[0] = 0.25f;
    FtReading m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(near(m.force[2], 2 * G + 1.0, 1e-4) && near(m.torque[0], 0.25, 1e-6), "bias");
    s.forceBias[2] = 0.0f;
    s.forceNoiseStdDev = 0.5f;
    s.seed = 7;
    ForceTorque a(s), b(s);
    FtReading ma = a.MeasureStatic(r, urdf::JointValues()), mb = b.MeasureStatic(r, urdf::JointValues());
    check(ma.force[2] == mb.force[2] && ma.force[0] == mb.force[0], "same seed: same noise");
    check(ma.force[0] != 0.0, "noise applied");
    double sum = 0, sq = 0;
    const int N = 2000;
    for (int i = 0; i < N; i++) {
        const double v = a.MeasureStatic(r, urdf::JointValues()).force[0];
        sum += v;
        sq += v * v;
    }
    const double mean = sum / N, sd = sqrt(sq / N - mean * mean);
    check(near(mean, 0, 0.05) && near(sd, 0.5, 0.05), "noise statistics");
    s.forceNoiseStdDev = 0.0f;
    s.forceRange = 10.0f;
    m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(m.saturated && near(m.force[2], 10.0, 1e-9), "force saturates at range");
    s.forceRange = 0.0f;
    check(!ForceTorque(s).MeasureStatic(r, urdf::JointValues()).saturated, "no range: not saturated");
}

void testUrdfAndErrors() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    check(r.forceTorques.size() == 1 && r.forceTorques[0].joint == "wrist" && r.forceTorques[0].childToParent, "urdf force_torque parsed");
    check(near(r.forceTorques[0].forceNoise, 0.5, 1e-6) && near(r.forceTorques[0].torqueNoise, 0.05, 1e-6), "urdf noise");
    const FtSettings s = FtSettingsFromDef(r.forceTorques[0]);
    check(s.direction == kFtChildToParent && s.joint == "wrist", "settings from def");
    const FtReading m = ForceTorque(s).MeasureStatic(r, urdf::JointValues());
    check(m.force[2] < 0, "def direction applied (child_to_parent)");
    // 없는 관절
    FtSettings bad;
    bad.joint = "nope";
    check(!ForceTorque(bad).MeasureStatic(r, urdf::JointValues()).valid, "unknown joint: invalid reading");
    // URDF 오류
    urdf::Robot r2;
    String err;
    String x = kUrdf;
    const String badRef = "<gazebo reference='ghost'><sensor name='f' type='force_torque'/></gazebo></robot>";
    String xml = x.substr(0, x.size() - 8) + badRef;
    check(!urdf::LoadUrdfFromString(xml, &r2, &err), "sensor on missing joint rejected");
    String xml2 = x;
    size_t p = xml2.find("child_to_parent");
    xml2.replace(p, 15, "sideways");
    check(!urdf::LoadUrdfFromString(xml2, &r2, &err), "bad measure_direction rejected");
}

}  // namespace

int main() {
    testStatic();
    testDirectionAndPose();
    testAccelerationAndExternal();
    testNoiseBiasSaturation();
    testUrdfAndErrors();
    if (g_fail == 0) printf("test_cforcetorque: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
