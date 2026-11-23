#include "CSensor/ForceTorque.h"

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

}  // namespace

double FtReading::ForceNorm() const { return sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]); }
double FtReading::TorqueNorm() const { return sqrt(torque[0] * torque[0] + torque[1] * torque[1] + torque[2] * torque[2]); }

bool IsBelowJoint(const urdf::Robot& robot, const simul::String& joint, const simul::String& link) {
    const int ji = robot.FindJoint(joint);
    const int li = robot.FindLink(link);
    if (ji < 0 || li < 0) return false;
    const int child = robot.JointChild(static_cast<size_t>(ji));
    int cur = li;
    for (size_t depth = 0; depth <= robot.links.size(); depth++) {  // 위로 올라가며 센서 관절의 자식을 만나는지 본다
        if (cur == child) return true;
        int up = -1;
        for (size_t j = 0; j < robot.joints.size(); j++) {
            if (robot.JointChild(j) == cur) {
                up = robot.JointParent(j);
                break;
            }
        }
        if (up < 0) return false;
        cur = up;
    }
    return false;
}

FtSettings FtSettingsFromDef(const urdf::FtDef& d) {
    FtSettings s;
    s.joint = d.joint;
    s.pose = d.pose;
    s.direction = d.childToParent ? kFtChildToParent : kFtParentToChild;
    s.forceNoiseStdDev = d.forceNoise;
    s.torqueNoiseStdDev = d.torqueNoise;
    return s;
}

ForceTorque::ForceTorque(const FtSettings& settings) : settings_(settings), rng_(settings.seed) {}

void ForceTorque::SetSettings(const FtSettings& s) {
    settings_ = s;
    rng_ = s.seed;
}

unsigned ForceTorque::nextRandom() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_ >> 8;
}

float ForceTorque::gaussian() {  // Box-Muller (Lidar 와 같은 방식)
    const float u1 = (static_cast<float>(nextRandom() & 0xFFFFFF) + 1.0f) / 16777217.0f;
    const float u2 = static_cast<float>(nextRandom() & 0xFFFFFF) / 16777216.0f;
    return static_cast<float>(sqrt(-2.0 * log(static_cast<double>(u1))) * cos(2.0 * kPi * static_cast<double>(u2)));
}

FtReading ForceTorque::MeasureStatic(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::Gravity& g, const FtExternalWrench* external,
                                     size_t externalCount) {
    return Measure(robot, q, dynamics::JointTorques(), dynamics::JointTorques(), g, external, externalCount);
}

FtReading ForceTorque::Measure(const urdf::Robot& robot, const urdf::JointValues& q, const dynamics::JointTorques& qd,
                               const dynamics::JointTorques& qdd, const dynamics::Gravity& g, const FtExternalWrench* external, size_t externalCount) {
    FtReading out;
    const int ji = robot.FindJoint(settings_.joint);
    if (ji < 0) return out;
    const size_t j = static_cast<size_t>(ji);
    const int ci = robot.JointChild(j);
    if (ci < 0) return out;

    // 1) 관절이 아래쪽에 전달해야 하는 하중 (루트 좌표계, 자식 링크 원점 기준)
    const simul::Array<dynamics::JointWrench> w = dynamics::JointWrenches(robot, q, qd, qdd, g, true);
    V3 f(w[j].force[0], w[j].force[1], w[j].force[2]);
    V3 n(w[j].moment[0], w[j].moment[1], w[j].moment[2]);

    // 2) 아래쪽에 작용하는 외력은 그만큼 전달 하중을 줄여 준다 (원점으로 옮겨서 뺀다)
    const Kinematics k = Forward(robot, q, true);
    const V3 o = k.p[static_cast<size_t>(ci)];
    for (size_t e = 0; e < externalCount; e++) {
        if (!IsBelowJoint(robot, settings_.joint, external[e].link)) continue;
        const V3 fe(external[e].force[0], external[e].force[1], external[e].force[2]);
        const V3 pe(external[e].point[0], external[e].point[1], external[e].point[2]);
        const V3 te(external[e].torque[0], external[e].torque[1], external[e].torque[2]);
        f = f - fe;
        n = n - (Cross(pe - o, fe) + te);
    }

    // 3) 센서 좌표계: 자식 링크 좌표계 * pose. 모멘트는 센서 원점 기준으로 옮긴다.
    const M3 Rs = k.R[static_cast<size_t>(ci)] * RotOf(settings_.pose);
    const V3 s = o + k.R[static_cast<size_t>(ci)] * TransOf(settings_.pose);
    n = n + Cross(o - s, f);
    M3 Rt;
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) Rt.m[a][b] = Rs.m[b][a];
    V3 fs = Rt * f;
    V3 ns = Rt * n;
    if (settings_.direction == kFtChildToParent) {
        fs = fs * -1.0;
        ns = ns * -1.0;
    }

    // 4) 편향, 잡음, 포화
    double fv[3] = {fs.x, fs.y, fs.z}, nv[3] = {ns.x, ns.y, ns.z};
    bool hit = false;
    for (int a = 0; a < 3; a++) {
        fv[a] += static_cast<double>(settings_.forceBias[a]);
        nv[a] += static_cast<double>(settings_.torqueBias[a]);
        if (settings_.forceNoiseStdDev > 0.0f) fv[a] += static_cast<double>(settings_.forceNoiseStdDev * gaussian());
        if (settings_.torqueNoiseStdDev > 0.0f) nv[a] += static_cast<double>(settings_.torqueNoiseStdDev * gaussian());
        out.force[a] = Clamp(fv[a], static_cast<double>(settings_.forceRange), &hit);
        out.torque[a] = Clamp(nv[a], static_cast<double>(settings_.torqueRange), &hit);
    }
    out.saturated = hit;
    out.valid = true;
    return out;
}

}  // namespace sensor
}  // namespace simul
