// CSensor 엔코더/오도메트리 테스트: 양자화, 방향, 증분/절대, 감속비, 속도, 이동 관절, 잡음, URDF, 차동 구동 오도메트리(직진/제자리 회전/원호), 예외
#include <math.h>
#include <stdio.h>

#include "CSensor/Encoder.h"
#include "CSensor/Odometry.h"
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
const double kPi = 3.14159265358979323846;

// 차동 2륜 + 이동 관절 하나. 왼쪽 바퀴 축은 +y 로 굴러가는 방향이 앞(+x)이 되도록 두 바퀴 모두 axis y.
const char* kUrdf =
    "<robot name='r'>"
    " <link name='base'><inertial><mass value='5'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <link name='wl'/><link name='wr'/><link name='lift'/>"
    " <joint name='left_wheel' type='continuous'><parent link='base'/><child link='wl'/><origin xyz='0 0.25 0'/><axis xyz='0 1 0'/></joint>"
    " <joint name='right_wheel' type='continuous'><parent link='base'/><child link='wr'/><origin xyz='0 -0.25 0'/><axis xyz='0 1 0'/></joint>"
    " <joint name='slide' type='prismatic'><parent link='base'/><child link='lift'/><axis xyz='0 0 1'/><limit lower='0' upper='1' effort='10' velocity='1'/></joint>"
    " <gazebo reference='left_wheel'><sensor name='enc_l' type='encoder'><encoder><resolution>2048</resolution><gear_ratio>10</gear_ratio><invert>true</invert></encoder>"
    "  <noise><stddev>0.5</stddev></noise></sensor></gazebo>"
    "</robot>";

bool load(const char* xml, urdf::Robot* r) {
    String err;
    if (!urdf::LoadUrdfFromString(xml, r, &err)) {
        printf("load error: %s\n", err.c_str());
        return false;
    }
    return true;
}

EncoderSettings enc(const char* joint, double res, double gear = 1.0) {
    EncoderSettings s;
    s.joint = joint;
    s.resolution = res;
    s.gearRatio = gear;
    return s;
}

void testQuantization() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    Encoder e(enc("right_wheel", 1000.0));
    check(near(e.CountsPerUnit(), 1000.0, 1e-9), "counts per rev");
    urdf::JointValues q;
    q["right_wheel"] = 0.0f;
    EncoderReading m = e.Measure(r, q);
    check(m.valid && m.ticks == 0, "incremental: first sample is zero");
    q["right_wheel"] = static_cast<float>(kPi);  // 반 바퀴 = 500 카운트
    m = e.Measure(r, q);
    check(m.ticks == 500 || m.ticks == 499, "half turn ~ 500 counts");
    check(near(m.position, kPi, 2 * kPi / 1000.0), "position within 1 count");
    // 한 카운트보다 작은 움직임은 안 보인다
    Encoder e2(enc("right_wheel", 1000.0));
    q["right_wheel"] = 1.0f;
    e2.Measure(r, q);
    q["right_wheel"] = 1.0f + static_cast<float>(2 * kPi / 1000.0 * 0.4);
    check(e2.Measure(r, q).ticks == 0, "sub-count motion is invisible");
    // 영점 다시 잡기
    q["right_wheel"] = 2.0f;
    e2.Reset();
    check(e2.Measure(r, q).ticks == 0, "reset re-zeros");
}

void testDirectionAbsoluteGear() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    urdf::JointValues q;
    q["right_wheel"] = 0.5f;
    EncoderSettings s = enc("right_wheel", 1000.0, 2.0);   // 관절 1회전 = 2000 카운트
    s.absolute = true;
    EncoderReading m = Encoder(s).Measure(r, q);
    check(near(static_cast<double>(m.ticks), 0.5 / (2 * kPi) * 2000.0, 1.0), "absolute with gear ratio");
    s.invert = true;
    m = Encoder(s).Measure(r, q);
    check(m.ticks < 0 && m.position < 0, "invert flips direction");
    // 속도: dt 를 주면 카운트 차이로, 안 주면 qd 로
    Encoder e(enc("right_wheel", 4096.0));
    urdf::JointValues q0, q1;
    q0["right_wheel"] = 0.0f;
    q1["right_wheel"] = 0.1f;
    e.Measure(r, q0, Map<String, double>(), 0.01);
    EncoderReading v = e.Measure(r, q1, Map<String, double>(), 0.01);
    check(near(v.velocity, 10.0, 2 * kPi / 4096.0 / 0.01), "velocity from counts");
    Map<String, double> qd;
    qd["right_wheel"] = 3.0;
    check(near(Encoder(enc("right_wheel", 1000.0)).Measure(r, q0, qd).velocity, 3.0, 1e-12), "velocity from qd");
    EncoderSettings si = enc("right_wheel", 1000.0);
    si.invert = true;
    check(near(Encoder(si).Measure(r, q0, qd).velocity, -3.0, 1e-12), "qd velocity follows invert");
}

void testPrismaticAndErrors() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    urdf::JointValues q;
    q["slide"] = 0.25f;
    EncoderSettings s = enc("slide", 4000.0);   // 1 m 당 4000 카운트
    s.absolute = true;
    check(Encoder(s).Measure(r, q).ticks == 1000, "prismatic: counts per meter");
    check(!Encoder(enc("nope", 100.0)).Measure(r, q).valid, "unknown joint invalid");
    check(!Encoder(enc("slide", 0.0)).Measure(r, q).valid, "zero resolution invalid");
    check(!Encoder(enc("slide", -5.0)).Measure(r, q).valid, "negative resolution invalid");
}

void testNoiseAndUrdf() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    check(r.encoders.size() == 1 && r.encoders[0].joint == "left_wheel" && r.encoders[0].invert && !r.encoders[0].absolute, "urdf encoder parsed");
    check(near(r.encoders[0].resolution, 2048, 1e-9) && near(r.encoders[0].gearRatio, 10, 1e-9) && near(r.encoders[0].noiseCounts, 0.5, 1e-6), "urdf values");
    EncoderSettings s = EncoderSettingsFromDef(r.encoders[0]);
    check(s.joint == "left_wheel" && s.invert, "settings from def");
    s.absolute = true;
    s.seed = 3;
    Encoder a(s), b(s);
    urdf::JointValues q;
    q["left_wheel"] = 1.0f;
    check(a.Measure(r, q).ticks == b.Measure(r, q).ticks, "same seed same noise");
    // 잡음이 있어도 평균은 맞다
    double sum = 0;
    const int N = 2000;
    for (int i = 0; i < N; i++) sum += static_cast<double>(a.Measure(r, q).ticks);
    check(near(sum / N, -1.0 / (2 * kPi) * 20480.0, 1.0), "noisy mean near truth");
    urdf::Robot r2;
    String err;
    String x = kUrdf;
    String bad = x.substr(0, x.size() - 8) + "<gazebo reference='ghost'><sensor name='e' type='encoder'/></gazebo></robot>";
    check(!urdf::LoadUrdfFromString(bad, &r2, &err), "encoder on missing joint rejected");
    String bad2 = x;
    bad2.replace(bad2.find("<resolution>2048"), 16, "<resolution>-1");
    check(!urdf::LoadUrdfFromString(bad2, &r2, &err), "negative resolution rejected");
    String bad3 = x;
    bad3.replace(bad3.find(">true</invert"), 5, ">yes<");
    check(!urdf::LoadUrdfFromString(bad3, &r2, &err), "bad bool rejected");
}

void testOdometry() {
    DiffDriveGeometry g;
    g.wheelRadius = 0.1;
    g.trackWidth = 0.5;
    DiffDriveOdometry o(g);
    // 직진: 두 바퀴 10 rad -> 1 m
    o.Update(10.0, 10.0, 1.0);
    check(near(o.Pose().x, 1.0, 1e-9) && near(o.Pose().y, 0, 1e-9) && near(o.Pose().yaw, 0, 1e-9) && near(o.Pose().v, 1.0, 1e-9), "straight");
    // 제자리 회전: 왼쪽 -, 오른쪽 + : 한 바퀴 (2 pi * track/2 = 1.5708 m 호길이) -> 각 바퀴 15.708 rad
    o.Reset();
    const double a = kPi * g.trackWidth / 2.0 / g.wheelRadius;   // 반 바퀴 (pi rad) 회전
    o.Update(-a, a, 1.0);
    check(near(fabs(o.Pose().yaw), kPi, 1e-9) && near(o.Pose().x, 0, 1e-9) && near(o.Pose().w, kPi, 1e-9), "spin in place half turn");
    // 원호: 반지름 R=1 인 반원 (왼쪽 커브). 바깥(오른쪽) 호길이 = pi*(R+t/2), 안쪽 = pi*(R-t/2). 작은 스텝으로 적분하면 (0, 2) 에 yaw=pi
    o.Reset();
    const int n = 1000;
    const double dl = kPi * (1.0 - 0.25) / g.wheelRadius / n, dr = kPi * (1.0 + 0.25) / g.wheelRadius / n;
    for (int i = 0; i < n; i++) o.Update(dl, dr, 0.01);
    check(near(o.Pose().x, 0.0, 1e-4) && near(o.Pose().y, 2.0, 1e-4) && near(fabs(o.Pose().yaw), kPi, 1e-6), "half circle");
    check(near(o.Pose().v, (dl + dr) / 2 * g.wheelRadius / 0.01, 1e-9), "speed of last interval");
    // 초기 자세
    o.Reset(1.0, 2.0, kPi / 2);
    o.Update(10.0, 10.0, 1.0);   // 1 m, +y 방향
    check(near(o.Pose().x, 1.0, 1e-9) && near(o.Pose().y, 3.0, 1e-9), "initial pose");
}

void testEncodersToOdometry() {
    // 엔코더 -> 오도메트리 전체 경로: 좌우 바퀴를 직접 돌려 원호 주행을 만들고 오도메트리가 해석해에 가까운지 본다
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    EncoderSettings sl = enc("left_wheel", 4096.0, 1.0), sr = enc("right_wheel", 4096.0, 1.0);
    sl.invert = true;   // 두 바퀴 axis 가 같은 +y 이므로 앞으로 구르는 방향(+x 쪽)이 + 가 되도록 맞춘다
    sr.invert = true;
    Encoder el(sl), er(sr);
    DiffDriveGeometry g;
    g.wheelRadius = 0.1;
    g.trackWidth = 0.5;
    DiffDriveOdometry o(g);
    const int n = 2000;
    const double totalL = kPi * 0.75 / 0.1, totalR = kPi * 1.25 / 0.1;   // 위와 같은 반원
    for (int i = 0; i <= n; i++) {
        urdf::JointValues q;
        q["left_wheel"] = static_cast<float>(-totalL * i / n);    // axis +y 에서 앞으로 구르면 음의 각도
        q["right_wheel"] = static_cast<float>(-totalR * i / n);
        o.Update(el.Measure(r, q), er.Measure(r, q), 0.01);
    }
    check(near(o.Pose().x, 0.0, 0.01) && near(o.Pose().y, 2.0, 0.01) && near(fabs(o.Pose().yaw), kPi, 0.01), "encoder -> odometry half circle");
    // 첫 호출은 기준만 잡는다
    DiffDriveOdometry o2(g);
    EncoderReading bad;
    check(!o2.Update(bad, bad, 0.1), "invalid readings ignored");
}

}  // namespace

int main() {
    testQuantization();
    testDirectionAbsoluteGear();
    testPrismaticAndErrors();
    testNoiseAndUrdf();
    testOdometry();
    testEncodersToOdometry();
    if (g_fail == 0) printf("test_cencoder: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
