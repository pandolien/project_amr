#include "CSensor/Odometry.h"

#include <math.h>

namespace simul {
namespace sensor {

namespace {
const double kPi = 3.14159265358979323846;

double WrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}
}  // namespace

DiffDriveOdometry::DiffDriveOdometry(const DiffDriveGeometry& g) : geom_(g), hasPrev_(false), prevLeft_(0), prevRight_(0) {}

void DiffDriveOdometry::Reset(double x, double y, double yaw) {
    pose_ = Pose2D();
    pose_.x = x;
    pose_.y = y;
    pose_.yaw = WrapPi(yaw);
    hasPrev_ = false;
}

void DiffDriveOdometry::Update(double dLeftAngle, double dRightAngle, double dt) {
    if (!(geom_.trackWidth > 0.0)) return;
    const double dl = dLeftAngle * geom_.wheelRadius, dr = dRightAngle * geom_.wheelRadius;
    const double ds = 0.5 * (dl + dr);
    const double dth = (dr - dl) / geom_.trackWidth;
    const double mid = pose_.yaw + 0.5 * dth;
    pose_.x += ds * cos(mid);
    pose_.y += ds * sin(mid);
    pose_.yaw = WrapPi(pose_.yaw + dth);
    if (dt > 0.0) {
        pose_.v = ds / dt;
        pose_.w = dth / dt;
    }
}

bool DiffDriveOdometry::Update(const EncoderReading& left, const EncoderReading& right, double dt) {
    if (!left.valid || !right.valid) return false;
    if (!hasPrev_) {
        prevLeft_ = left.position;
        prevRight_ = right.position;
        hasPrev_ = true;
        return false;
    }
    Update(left.position - prevLeft_, right.position - prevRight_, dt);
    prevLeft_ = left.position;
    prevRight_ = right.position;
    return true;
}

}  // namespace sensor
}  // namespace simul
