// CSensor IMU 테스트: 정지 비력(+g), 기울기, 자이로, 구심/접선 가속도, 루트 운동, 자유낙하, 자세 쿼터니언, 편향/잡음/포화, URDF 읽기, 예외
#include <math.h>
#include <stdio.h>

#include "CSensor/Imu.h"
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
const double kPi = 3.14159265358979323846;

// base --(yaw: continuous z)--> top --(tilt: continuous y)--> head
const char* kUrdf =
    "<robot name='r'>"
    " <link name='base'><inertial><mass value='5'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <link name='top'><inertial><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <link name='head'><inertial><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
    " <joint name='yaw' type='continuous'><parent link='base'/><child link='top'/><axis xyz='0 0 1'/></joint>"
    " <joint name='tilt' type='continuous'><parent link='top'/><child link='head'/><axis xyz='0 1 0'/></joint>"
    " <gazebo reference='head'><sensor name='imu0' type='imu'><pose>0.5 0 0 0 0 0</pose>"
    "  <imu><linear_acceleration><x><noise type='gaussian'><stddev>0.2</stddev></noise></x></linear_acceleration>"
    "       <angular_velocity><x><noise type='gaussian'><stddev>0.01</stddev></noise></x></angular_velocity>"
    "       <orientation><noise><stddev>0.002</stddev></noise></orientation></imu></sensor></gazebo>"
    "</robot>";

bool load(const char* xml, urdf::Robot* r) {
    String err;
    if (!urdf::LoadUrdfFromString(xml, r, &err)) {
        printf("load error: %s\n", err.c_str());
        return false;
    }
    return true;
}

ImuSettings on(const char* link, float x = 0.0f) {
    ImuSettings s;
    s.link = link;
    s.pose.v[12] = x;
    return s;
}

void testStatic() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    const ImuReading m = Imu(on("top")).Measure(r, urdf::JointValues());
    check(m.valid, "valid");
    check(near(m.accel[0], 0, 1e-9) && near(m.accel[1], 0, 1e-9) && near(m.accel[2], G, 1e-9), "level: accel = +g up");
    check(near(m.gyro[0], 0, 1e-12) && near(m.gyro[1], 0, 1e-12) && near(m.gyro[2], 0, 1e-12), "static: gyro 0");
    check(near(m.orientation[0], 1, 1e-9) && near(m.orientation[1], 0, 1e-9), "identity orientation");
    // 중력이 없으면 가속도 0
    const ImuReading z = Imu(on("top")).Measure(r, urdf::JointValues(), dynamics::JointTorques(), dynamics::JointTorques(), dynamics::Gravity(0, 0, 0));
    check(near(z.accel[2], 0, 1e-12), "no gravity: 0");
}

void testTiltAndOrientation() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    urdf::JointValues q;
    const double th = 30.0 * kPi / 180.0;
    q["tilt"] = static_cast<float>(th);
    const ImuReading m = Imu(on("head")).Measure(r, q);
    // R = Ry(th): 센서에서 본 중력 방향 = R^T (0, 0, g) = (-sin g, 0, cos g)
    check(near(m.accel[0], -sin(th) * G, 1e-5) && near(m.accel[1], 0, 1e-9) && near(m.accel[2], cos(th) * G, 1e-5), "tilted: gravity in sensor frame");
    check(near(sqrt(m.accel[0] * m.accel[0] + m.accel[2] * m.accel[2]), G, 1e-5), "norm = g");
    // 자세: y 축 th 회전 -> (cos(th/2), 0, sin(th/2), 0)
    check(near(m.orientation[0], cos(th / 2), 1e-6) && near(m.orientation[2], sin(th / 2), 1e-6) && near(m.orientation[1], 0, 1e-6), "orientation y-rot");
    // yaw 90 도 -> (cos45, 0, 0, sin45)
    urdf::JointValues q2;
    q2["yaw"] = static_cast<float>(kPi / 2);
    const ImuReading m2 = Imu(on("top")).Measure(r, q2);
    check(near(m2.orientation[0], sqrt(0.5), 1e-6) && near(m2.orientation[3], sqrt(0.5), 1e-6), "orientation z-rot 90");
    check(near(m2.accel[2], G, 1e-5), "yaw doesn't change gravity");
}

void testRotation() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    dynamics::JointTorques qd, qdd;
    qd["yaw"] = 2.0;
    ImuReading m = Imu(on("top")).Measure(r, urdf::JointValues(), qd);
    check(near(m.gyro[2], 2.0, 1e-9) && near(m.gyro[0], 0, 1e-9), "gyro z = yaw rate");
    // 원점에서 0.5 m: 구심 가속도 = -w^2 r (축 쪽 = 센서 -x)
    m = Imu(on("top", 0.5f)).Measure(r, urdf::JointValues(), qd);
    check(near(m.accel[0], -2.0 * 2.0 * 0.5, 1e-6) && near(m.accel[2], G, 1e-9), "centripetal acceleration");
    // 각가속도 3: 접선 가속도 = alpha x r = (0,0,3) x (0.5,0,0) = (0, 1.5, 0)
    qd.clear();
    qdd["yaw"] = 3.0;
    m = Imu(on("top", 0.5f)).Measure(r, urdf::JointValues(), qd, qdd);
    check(near(m.accel[1], 1.5, 1e-6) && near(m.accel[0], 0, 1e-6), "tangential acceleration");
    // 기울어진 링크의 자이로: yaw 가 도는 동안 tilt 30 도 -> 센서 z 성분 = cos, x 성분 = -sin 이 아니라 R^T (0,0,w)
    urdf::JointValues q;
    const double th = 30.0 * kPi / 180.0;
    q["tilt"] = static_cast<float>(th);
    qd["yaw"] = 1.0;
    m = Imu(on("head")).Measure(r, q, qd);
    check(near(m.gyro[0], -sin(th), 1e-5) && near(m.gyro[2], cos(th), 1e-5), "gyro in tilted sensor frame");
}

void testRootMotion() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    dynamics::RootMotion root;
    root.linAcc[0] = 1.0;
    ImuReading m = Imu(on("base")).Measure(r, urdf::JointValues(), dynamics::JointTorques(), dynamics::JointTorques(), dynamics::Gravity(), root);
    check(near(m.accel[0], 1.0, 1e-9) && near(m.accel[2], G, 1e-9), "AMR accelerating forward");
    // 자유낙하: a = g -> 비력 0
    dynamics::RootMotion fall;
    fall.linAcc[2] = -G;
    m = Imu(on("base")).Measure(r, urdf::JointValues(), dynamics::JointTorques(), dynamics::JointTorques(), dynamics::Gravity(), fall);
    check(near(m.accel[2], 0, 1e-9), "free fall reads zero");
    // 루트가 z 축으로 돌면 그 위의 링크 자이로에도 더해진다
    dynamics::RootMotion spin;
    spin.omega[2] = 1.0;
    dynamics::JointTorques qd;
    qd["yaw"] = 2.0;
    m = Imu(on("top")).Measure(r, urdf::JointValues(), qd, dynamics::JointTorques(), dynamics::Gravity(), spin);
    check(near(m.gyro[2], 3.0, 1e-9), "root spin adds to joint rate");
}

void testNoiseBiasSaturation() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    ImuSettings s = on("top");
    s.accelBias[0] = 0.1f;
    s.gyroBias[2] = 0.02f;
    ImuReading m = Imu(s).Measure(r, urdf::JointValues());
    check(near(m.accel[0], 0.1, 1e-6) && near(m.gyro[2], 0.02, 1e-7), "bias");
    s.accelBias[0] = 0.0f;
    s.gyroBias[2] = 0.0f;
    s.accelNoiseStdDev = 0.3f;
    s.seed = 5;
    Imu a(s), b(s);
    const ImuReading ma = a.Measure(r, urdf::JointValues()), mb = b.Measure(r, urdf::JointValues());
    check(ma.accel[0] == mb.accel[0] && ma.accel[2] == mb.accel[2], "same seed same noise");
    double sum = 0, sq = 0;
    const int N = 2000;
    for (int i = 0; i < N; i++) {
        const double v = a.Measure(r, urdf::JointValues()).accel[0];
        sum += v;
        sq += v * v;
    }
    const double mean = sum / N, sd = sqrt(sq / N - mean * mean);
    check(near(mean, 0, 0.03) && near(sd, 0.3, 0.03), "noise statistics");
    s.accelNoiseStdDev = 0.0f;
    s.accelRange = 5.0f;
    m = Imu(s).Measure(r, urdf::JointValues());
    check(m.saturated && near(m.accel[2], 5.0, 1e-9), "accel saturates");
    // 자세 잡음: 단위 쿼터니언 유지
    s.accelRange = 0.0f;
    s.orientationNoiseStdDev = 0.05f;
    m = Imu(s).Measure(r, urdf::JointValues());
    const double n2 = m.orientation[0] * m.orientation[0] + m.orientation[1] * m.orientation[1] + m.orientation[2] * m.orientation[2] + m.orientation[3] * m.orientation[3];
    check(near(n2, 1.0, 1e-9) && m.orientation[0] < 1.0, "orientation noise keeps unit quaternion");
}

void testUrdfAndErrors() {
    urdf::Robot r;
    check(load(kUrdf, &r), "load");
    check(r.imus.size() == 1 && r.imus[0].link == "head", "urdf imu parsed");
    check(near(r.imus[0].accelNoise, 0.2, 1e-6) && near(r.imus[0].gyroNoise, 0.01, 1e-7) && near(r.imus[0].orientationNoise, 0.002, 1e-7), "urdf noise");
    const ImuSettings s = ImuSettingsFromDef(r.imus[0]);
    check(s.link == "head" && near(s.pose.v[12], 0.5, 1e-6), "settings from def");
    check(!Imu(on("nope")).Measure(r, urdf::JointValues()).valid, "unknown link invalid");
    urdf::Robot r2;
    String err;
    String x = kUrdf;
    String xml = x.substr(0, x.size() - 8) + "<gazebo reference='ghost'><sensor name='i' type='imu'/></gazebo></robot>";
    check(!urdf::LoadUrdfFromString(xml, &r2, &err), "imu on missing link rejected");
    String neg = x;
    neg.replace(neg.find("0.2</stddev>"), 3, "-.2");
    check(!urdf::LoadUrdfFromString(neg, &r2, &err), "negative noise rejected");
}

}  // namespace

int main() {
    testStatic();
    testTiltAndOrientation();
    testRotation();
    testRootMotion();
    testNoiseBiasSaturation();
    testUrdfAndErrors();
    if (g_fail == 0) printf("test_cimu: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
