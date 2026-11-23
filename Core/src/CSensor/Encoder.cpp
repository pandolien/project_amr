#include "CSensor/Encoder.h"

#include <math.h>

namespace simul {
namespace sensor {

namespace {
const double kPi = 3.14159265358979323846;
}

EncoderSettings EncoderSettingsFromDef(const urdf::EncoderDef& d) {
    EncoderSettings s;
    s.joint = d.joint;
    s.resolution = d.resolution;
    s.gearRatio = d.gearRatio;
    s.invert = d.invert;
    s.absolute = d.absolute;
    s.noiseCounts = d.noiseCounts;
    return s;
}

Encoder::Encoder(const EncoderSettings& settings) : settings_(settings), rng_(settings.seed), hasZero_(false), hasPrev_(false), zero_(0), prev_(0) {}

void Encoder::SetSettings(const EncoderSettings& s) {
    settings_ = s;
    rng_ = s.seed;
    Reset();
}

void Encoder::Reset() {
    hasZero_ = false;
    hasPrev_ = false;
    zero_ = 0;
    prev_ = 0;
}

double Encoder::CountsPerUnit() const {
    const double c = settings_.resolution * settings_.gearRatio;
    if (!(c > 0.0) || !(c < 1e12)) return 0.0;  // NaN, 0, 음수, 너무 큰 값
    return c;
}

unsigned Encoder::nextRandom() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_ >> 8;
}

float Encoder::gaussian() {  // Box-Muller (Lidar 와 같은 방식)
    const float u1 = (static_cast<float>(nextRandom() & 0xFFFFFF) + 1.0f) / 16777217.0f;
    const float u2 = static_cast<float>(nextRandom() & 0xFFFFFF) / 16777216.0f;
    return static_cast<float>(sqrt(-2.0 * log(static_cast<double>(u1))) * cos(2.0 * kPi * static_cast<double>(u2)));
}

EncoderReading Encoder::Measure(const urdf::Robot& robot, const urdf::JointValues& q, const simul::Map<simul::String, double>& qd, double dt) {
    EncoderReading out;
    const int ji = robot.FindJoint(settings_.joint);
    const double perUnit = CountsPerUnit();
    if (ji < 0 || !(perUnit > 0.0)) return out;
    const urdf::Joint& j = robot.joints[static_cast<size_t>(ji)];
    if (!j.IsMovable()) return out;

    // 관절 값 -> 카운트. 회전 관절은 1회전 = 2 pi rad 당 perUnit 카운트, 이동 관절은 1 m 당
    const double value = static_cast<double>(robot.EffectiveJointValue(q, ji, true));
    const double unitsPerValue = j.IsAngular() ? 1.0 / (2.0 * kPi) : 1.0;
    double counts = value * unitsPerValue * perUnit;
    if (settings_.invert) counts = -counts;
    if (settings_.noiseCounts > 0.0f) counts += static_cast<double>(settings_.noiseCounts * gaussian());
    const int64_t raw = static_cast<int64_t>(floor(counts));

    if (!settings_.absolute && !hasZero_) {
        zero_ = raw;
        hasZero_ = true;
    }
    out.ticks = raw - (settings_.absolute ? 0 : zero_);

    out.position = static_cast<double>(out.ticks) / perUnit / unitsPerValue;  // 카운트 방향 기준 (invert 면 관절 값과 부호 반대)

    double vel = 0.0;
    if (dt > 0.0 && hasPrev_) {
        vel = static_cast<double>(out.ticks - prev_) / perUnit / unitsPerValue / dt;
    } else {
        const simul::Map<simul::String, double>::const_iterator it = qd.find(settings_.joint);
        if (it != qd.end()) vel = settings_.invert ? -it->second : it->second;
    }
    out.velocity = vel;
    prev_ = out.ticks;
    hasPrev_ = true;
    out.valid = true;
    return out;
}

}  // namespace sensor
}  // namespace simul
