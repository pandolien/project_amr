#include "3d/dhPose.h"

#include "CBase/Math.h"

dhPose::dhPose() {}

dhPose::dhPose(const dhVector& p, const dhQuat& q) : position(p), orientation(q) {}

dhPose dhPose::FromMat(const dhMat& m) {
    return dhPose(m.O(), m.GetQuat().Normalized());
}

dhMat dhPose::ToMat() const {
    dhMat m = orientation.Normalized().GetMat();
    m.v[12] = position.x;
    m.v[13] = position.y;
    m.v[14] = position.z;
    return m;
}

dhPose dhPose::Normalized() const { return dhPose(position, orientation.Normalized()); }

dhPose dhPose::Inv() const {
    const dhQuat qi = orientation.Normalized().Conjugate();
    const dhVector p = qi.Rotate(position);
    return dhPose(dhVector(-p.x, -p.y, -p.z), qi);
}

dhPose dhPose::operator*(const dhPose& o) const {
    const dhQuat q = orientation.Normalized();
    return dhPose(position + q.Rotate(o.position), (q * o.orientation.Normalized()).Normalized());
}

dhVector dhPose::operator*(const dhVector& p) const { return position + orientation.Normalized().Rotate(p); }

void dhPose::ToArray(double out[7]) const {
    dhQuat q = orientation.Normalized();
    if (q.w < 0) q = dhQuat(-q.x, -q.y, -q.z, -q.w);
    out[0] = position.x;
    out[1] = position.y;
    out[2] = position.z;
    out[3] = q.x;
    out[4] = q.y;
    out[5] = q.z;
    out[6] = q.w;
}

bool dhPose::FromArray(const double in[7], dhPose* out) {
    if (!out) return false;
    for (int i = 0; i < 7; i++)
        if (!simul::IsFinite(in[i])) return false;
    const double n = simul::Sqrt(in[3] * in[3] + in[4] * in[4] + in[5] * in[5] + in[6] * in[6]);
    if (!(n > 1e-12)) return false;
    out->position = dhVector(static_cast<float>(in[0]), static_cast<float>(in[1]), static_cast<float>(in[2]));
    out->orientation = dhQuat(static_cast<float>(in[3] / n), static_cast<float>(in[4] / n), static_cast<float>(in[5] / n),
                              static_cast<float>(in[6] / n));
    return true;
}
