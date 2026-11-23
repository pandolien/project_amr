#include "CSensor/Imu.h"

#include <math.h>

#include "DynMath.h"

namespace simul {
namespace sensor {

using namespace dynamics::detail;

namespace {

const double kPi = 3.14159265358979323846;

double Clamp(double v, double range, bool* hit) {
    if (!(range > 0.0)) return v;
    if (v > range) {
        *hit = true;
        return range;
    }
    if (v < -range) {
        *hit = true;
        return -range;
    }
    return v;
}

// 회전행렬 -> 쿼터니언 (w, x, y, z, w >= 0)
void ToQuat(const M3& r, double q[4]) {
    const double tr = r.m[0][0] + r.m[1][1] + r.m[2][2];
    if (tr > 0.0) {
        const double s = sqrt(tr + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (r.m[2][1] - r.m[1][2]) / s;
        q[2] = (r.m[0][2] - r.m[2][0]) / s;
        q[3] = (r.m[1][0] - r.m[0][1]) / s;
    } else if (r.m[0][0] > r.m[1][1] && r.m[0][0] > r.m[2][2]) {
        const double s = sqrt(1.0 + r.m[0][0] - r.m[1][1] - r.m[2][2]) * 2.0;
        q[0] = (r.m[2][1] - r.m[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (r.m[0][1] + r.m[1][0]) / s;
        q[3] = (r.m[0][2] + r.m[2][0]) / s;
    } else if (r.m[1][1] > r.m[2][2]) {
        const double s = sqrt(1.0 + r.m[1][1] - r.m[0][0] - r.m[2][2]) * 2.0;
        q[0] = (r.m[0][2] - r.m[2][0]) / s;
        q[1] = (r.m[0][1] + r.m[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (r.m[1][2] + r.m[2][1]) / s;
    } else {
        const double s = sqrt(1.0 + r.m[2][2] - r.m[0][0] - r.m[1][1]) * 2.0;
        q[0] = (r.m[1][0] - r.m[0][1]) / s;
        q[1] = (r.m[0][2] + r.m[2][0]) / s;
        q[2] = (r.m[1][2] + r.m[2][1]) / s;
        q[3] = 0.25 * s;
    }
    if (q[0] < 0.0)
        for (int i = 0; i < 4; i++) q[i] = -q[i];
}

}  // namespace

ImuSettings ImuSettingsFromDef(const urdf::ImuDef& d) {
    ImuSettings s;
    s.link = d.link;
    s.pose = d.pose;
    s.accelNoiseStdDev = d.accelNoise;
    s.gyroNoiseStdDev = d.gyroNoise;
    s.orientationNoiseStdDev = d.orientationNoise;
    return s;
}

Imu::Imu(const ImuSettings& settings) : settings_(settings), rng_(settings.seed) {}

void Imu::SetSettings(const ImuSettings& s) {
    settings_ = s;
    rng_ = s.seed;
}

unsigned Imu::nextRandom() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_ >> 8;
}

float Imu::gaussian() {  // Box-Muller (Lidar 와 같은 방식)
    const float u1 = (static_cast<float>(nextRandom() & 0xFFFFFF) + 1.0f) / 16777217.0f;
    const float u2 = static_cast<float>(nextRandom() & 0xFFFFFF) / 16777216.0f;
    return static_cast<float>(sqrt(-2.0 * log(static_cast<double>(u1))) * cos(2.0 * kPi * static_cast<double>(u2)));
}

ImuReading Imu::Measure(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::JointTorques& qd, const dynamics::JointTorques& qdd,
                        const dynamics::Gravity& g, const dynamics::RootMotion& root) {
    ImuReading out;
    const int li = robot.FindLink(settings_.link);
    if (li < 0) return out;
    const size_t l = static_cast<size_t>(li);

    const simul::Array<dynamics::LinkMotion> mo = dynamics::LinkMotions(robot, q, qd, qdd, g, root, true);
    const Kinematics k = Forward(robot, q, true);
    const V3 w(mo[l].omega[0], mo[l].omega[1], mo[l].omega[2]);
    const V3 al(mo[l].alpha[0], mo[l].alpha[1], mo[l].alpha[2]);
    const V3 f(mo[l].specificForce[0], mo[l].specificForce[1], mo[l].specificForce[2]);

    // 센서 점의 비력: 링크 원점에서 r 만큼 떨어진 점은 각가속도와 구심 가속도가 더해진다
    const V3 r = k.R[l] * TransOf(settings_.pose);
    const V3 fs = f + Cross(al, r) + Cross(w, Cross(w, r));
    const M3 Rs = k.R[l] * RotOf(settings_.pose);
    M3 Rt;
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) Rt.m[a][b] = Rs.m[b][a];
    const V3 acc = Rt * fs;
    const V3 gyr = Rt * w;

    double av[3] = {acc.x, acc.y, acc.z}, gv[3] = {gyr.x, gyr.y, gyr.z};
    bool hit = false;
    for (int a = 0; a < 3; a++) {
        av[a] += static_cast<double>(settings_.accelBias[a]);
        gv[a] += static_cast<double>(settings_.gyroBias[a]);
        if (settings_.accelNoiseStdDev > 0.0f) av[a] += static_cast<double>(settings_.accelNoiseStdDev * gaussian());
        if (settings_.gyroNoiseStdDev > 0.0f) gv[a] += static_cast<double>(settings_.gyroNoiseStdDev * gaussian());
        out.accel[a] = Clamp(av[a], static_cast<double>(settings_.accelRange), &hit);
        out.gyro[a] = Clamp(gv[a], static_cast<double>(settings_.gyroRange), &hit);
    }
    out.saturated = hit;

    // 자세: 센서 -> 루트 회전. 잡음은 작은 회전벡터를 오른쪽에서 곱한다.
    M3 R = Rs;
    if (settings_.orientationNoiseStdDev > 0.0f) {
        const double rx = static_cast<double>(settings_.orientationNoiseStdDev * gaussian());
        const double ry = static_cast<double>(settings_.orientationNoiseStdDev * gaussian());
        const double rz = static_cast<double>(settings_.orientationNoiseStdDev * gaussian());
        const double ang = sqrt(rx * rx + ry * ry + rz * rz);
        if (ang > 1e-12) R = Rs * AxisAngle(V3(rx / ang, ry / ang, rz / ang), ang);
    }
    ToQuat(R, out.orientation);
    out.valid = true;
    return out;
}

}  // namespace sensor
}  // namespace simul
