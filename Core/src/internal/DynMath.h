#ifndef CORE_SRC_INTERNAL_DYNMATH_H
#define CORE_SRC_INTERNAL_DYNMATH_H

// CDynamics 가 함께 쓰는 double 수학과 순기구학 (공개하지 않는 내부 헤더)
#include <math.h>

#include "CBase/Array.h"

#include "CURDF/Urdf.h"

namespace simul {
namespace dynamics {
namespace detail {

struct V3 {
    double x, y, z;
    V3() : x(0), y(0), z(0) {}
    V3(double px, double py, double pz) : x(px), y(py), z(pz) {}
};

inline V3 operator+(const V3& a, const V3& b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3 operator-(const V3& a, const V3& b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3 operator*(const V3& a, double s) { return V3(a.x * s, a.y * s, a.z * s); }
inline double Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(const V3& a, const V3& b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

// 3x3 회전행렬 (행 우선: m[행][열])
struct M3 {
    double m[3][3];
    M3() {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) m[i][j] = i == j ? 1.0 : 0.0;
    }
};

inline V3 operator*(const M3& a, const V3& v) {
    return V3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
              a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}

inline M3 operator*(const M3& a, const M3& b) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

// dhMat(열 우선 float) 의 회전 부분과 이동 부분을 double 로 옮긴다
inline M3 RotOf(const dhMat& h) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = static_cast<double>(h.v[j * 4 + i]);
    return r;
}

inline V3 TransOf(const dhMat& h) { return V3(h.v[12], h.v[13], h.v[14]); }

// 단위벡터 axis 둘레로 angle 만큼 회전 (Rodrigues)
inline M3 AxisAngle(const V3& a, double angle) {
    const double c = cos(angle), s = sin(angle), t = 1.0 - c;
    M3 r;
    r.m[0][0] = t * a.x * a.x + c;
    r.m[0][1] = t * a.x * a.y - s * a.z;
    r.m[0][2] = t * a.x * a.z + s * a.y;
    r.m[1][0] = t * a.x * a.y + s * a.z;
    r.m[1][1] = t * a.y * a.y + c;
    r.m[1][2] = t * a.y * a.z - s * a.x;
    r.m[2][0] = t * a.x * a.z - s * a.y;
    r.m[2][1] = t * a.y * a.z + s * a.x;
    r.m[2][2] = t * a.z * a.z + c;
    return r;
}

// 순기구학 결과 (double): 링크의 월드(루트 기준) 회전/위치, 관절의 월드 축/원점
struct Kinematics {
    simul::Array<M3> R;      // 링크별
    simul::Array<V3> p;
    simul::Array<V3> axisW;  // 관절별 (움직이는 관절만 의미 있음)
    simul::Array<V3> originW;
};

inline Kinematics Forward(const urdf::Robot& robot, const urdf::JointValues& q, bool clamp) {
    Kinematics k;
    k.R.assign(robot.links.size(), M3());
    k.p.assign(robot.links.size(), V3());
    k.axisW.assign(robot.joints.size(), V3());
    k.originW.assign(robot.joints.size(), V3());
    for (size_t o = 0; o < robot.order.size(); o++) {
        const int ji = robot.order[o];
        const urdf::Joint& j = robot.joints[static_cast<size_t>(ji)];
        const int pi = robot.JointParent(static_cast<size_t>(ji)), ci = robot.JointChild(static_cast<size_t>(ji));
        if (pi < 0 || ci < 0) continue;
        const M3 Rj = k.R[static_cast<size_t>(pi)] * RotOf(j.origin);                       // 관절 좌표계 (움직이기 전)
        const V3 oj = k.R[static_cast<size_t>(pi)] * TransOf(j.origin) + k.p[static_cast<size_t>(pi)];
        const V3 axis(j.axis.x, j.axis.y, j.axis.z);
        k.axisW[static_cast<size_t>(ji)] = Rj * axis;
        k.originW[static_cast<size_t>(ji)] = oj;
        const double v = static_cast<double>(robot.EffectiveJointValue(q, ji, clamp));
        if (j.IsAngular()) {
            k.R[static_cast<size_t>(ci)] = Rj * AxisAngle(axis, v);
            k.p[static_cast<size_t>(ci)] = oj;
        } else if (j.type == urdf::kJointPrismatic) {
            k.R[static_cast<size_t>(ci)] = Rj;
            k.p[static_cast<size_t>(ci)] = oj + Rj * (axis * v);
        } else {
            k.R[static_cast<size_t>(ci)] = Rj;
            k.p[static_cast<size_t>(ci)] = oj;
        }
    }
    return k;
}

// 관절 하나가 따라가는 구동 관절(마스터)과 배율. mimic 이 아니면 자기 자신과 1.
struct MasterOf {
    int index;
    double factor;
};

inline MasterOf FindMaster(const urdf::Robot& robot, size_t ji) {
    MasterOf r;
    r.index = static_cast<int>(ji);
    r.factor = 1.0;
    for (size_t depth = 0; depth < robot.joints.size() && robot.joints[static_cast<size_t>(r.index)].hasMimic; depth++) {
        const int m = robot.JointMimic(static_cast<size_t>(r.index));
        if (m < 0) break;
        r.factor *= static_cast<double>(robot.joints[static_cast<size_t>(r.index)].mimicMultiplier);
        r.index = m;
    }
    return r;
}

// 링크 질량과 월드 질량중심. 질량이 없으면 mass = 0.
inline void LinkMass(const urdf::Robot& robot, const Kinematics& k, size_t link, double* mass, V3* com) {
    const urdf::Link& l = robot.links[link];
    *mass = 0.0;
    *com = V3();
    if (!l.hasInertial || !(l.inertial.mass > 0.0f)) return;
    *mass = static_cast<double>(l.inertial.mass);
    *com = k.R[link] * TransOf(l.inertial.origin) + k.p[link];
}


}  // namespace detail
}  // namespace dynamics
}  // namespace simul

#endif  // CORE_SRC_INTERNAL_DYNMATH_H
