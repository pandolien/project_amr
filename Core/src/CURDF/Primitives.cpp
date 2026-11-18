#include "CURDF/Primitives.h"

namespace simul {
namespace urdf {

bool MakePrimitiveRobot(const String& name, const PrimitiveSpec& spec, Robot* out, String* error) {
    Geometry g;
    g.type = spec.type;
    float ixx = 0, iyy = 0, izz = 0;
    const float m = spec.mass;
    if (spec.type == kGeomBox) {
        if (!(spec.size[0] > 0.0f && spec.size[1] > 0.0f && spec.size[2] > 0.0f)) {
            if (error) *error = "박스의 크기는 모두 0 보다 커야 합니다";
            return false;
        }
        g.size[0] = spec.size[0]; g.size[1] = spec.size[1]; g.size[2] = spec.size[2];
        const float x = spec.size[0], y = spec.size[1], z = spec.size[2];
        ixx = m / 12.0f * (y * y + z * z);
        iyy = m / 12.0f * (x * x + z * z);
        izz = m / 12.0f * (x * x + y * y);
    } else if (spec.type == kGeomCylinder) {
        if (!(spec.radius > 0.0f && spec.length > 0.0f)) {
            if (error) *error = "원통의 반지름과 길이는 0 보다 커야 합니다";
            return false;
        }
        g.radius = spec.radius; g.length = spec.length;
        ixx = iyy = m / 12.0f * (3.0f * spec.radius * spec.radius + spec.length * spec.length);
        izz = 0.5f * m * spec.radius * spec.radius;
    } else if (spec.type == kGeomSphere) {
        if (!(spec.radius > 0.0f)) {
            if (error) *error = "구의 반지름은 0 보다 커야 합니다";
            return false;
        }
        g.radius = spec.radius;
        ixx = iyy = izz = 0.4f * m * spec.radius * spec.radius;
    } else {
        if (error) *error = "박스, 원통, 구만 만들 수 있습니다";
        return false;
    }
    Robot r;
    r.name = name;
    Link l;
    l.name = name;
    Visual v;
    v.geometry = g;
    v.hasMaterial = true;
    v.material.name = name + "_color";
    v.material.hasColor = true;
    v.material.color = spec.color;
    l.visuals.push_back(v);
    Collision c;
    c.geometry = g;
    l.collisions.push_back(c);
    if (m > 0.0f) {
        l.hasInertial = true;
        l.inertial.mass = m;
        l.inertial.ixx = ixx;
        l.inertial.iyy = iyy;
        l.inertial.izz = izz;
    }
    r.links.push_back(l);
    if (!FinalizeRobot(&r, error)) return false;
    *out = r;
    return true;
}

}  // namespace urdf
}  // namespace simul
