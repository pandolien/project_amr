#include "CDynamics/Gravity.h"

#include <math.h>

namespace amr {
namespace dynamics {

namespace {

struct V3 {
    double x, y, z;
    V3() : x(0), y(0), z(0) {}
    V3(double px, double py, double pz) : x(px), y(py), z(pz) {}
};

V3 operator+(const V3& a, const V3& b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
V3 operator-(const V3& a, const V3& b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
V3 operator*(const V3& a, double s) { return V3(a.x * s, a.y * s, a.z * s); }
double Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 Cross(const V3& a, const V3& b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

// 3x3 회전행렬 (행 우선: m[행][열])
struct M3 {
    double m[3][3];
    M3() {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) m[i][j] = i == j ? 1.0 : 0.0;
    }
};

V3 operator*(const M3& a, const V3& v) {
    return V3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
              a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}

M3 operator*(const M3& a, const M3& b) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

// dhMat(열 우선 float) 의 회전 부분과 이동 부분을 double 로 옮긴다
M3 RotOf(const dhMat& h) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = static_cast<double>(h.v[j * 4 + i]);
    return r;
}

V3 TransOf(const dhMat& h) { return V3(h.v[12], h.v[13], h.v[14]); }

// 단위벡터 axis 둘레로 angle 만큼 회전 (Rodrigues)
M3 AxisAngle(const V3& a, double angle) {
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
    std::vector<M3> R;      // 링크별
    std::vector<V3> p;
    std::vector<V3> axisW;  // 관절별 (움직이는 관절만 의미 있음)
    std::vector<V3> originW;
};

Kinematics Forward(const urdf::Robot& robot, const urdf::JointValues& q, bool clamp) {
    Kinematics k;
    k.R.assign(robot.links.size(), M3());
    k.p.assign(robot.links.size(), V3());
    k.axisW.assign(robot.joints.size(), V3());
    k.originW.assign(robot.joints.size(), V3());
    for (size_t o = 0; o < robot.order.size(); o++) {
        const int ji = robot.order[o];
        const urdf::Joint& j = robot.joints[static_cast<size_t>(ji)];
        const int pi = robot.FindLink(j.parent), ci = robot.FindLink(j.child);
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

// 링크 질량과 월드 질량중심. 질량이 없으면 mass = 0.
void LinkMass(const urdf::Robot& robot, const Kinematics& k, size_t link, double* mass, V3* com) {
    const urdf::Link& l = robot.links[link];
    *mass = 0.0;
    *com = V3();
    if (!l.hasInertial || !(l.inertial.mass > 0.0f)) return;
    *mass = static_cast<double>(l.inertial.mass);
    *com = k.R[link] * TransOf(l.inertial.origin) + k.p[link];
}

}  // namespace

JointTorques GravityTorques(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g, bool clampToLimits) {
    JointTorques out;
    const Kinematics k = Forward(robot, q, clampToLimits);
    const V3 gv(g.x, g.y, g.z);

    // 링크 아래쪽(자손 전체)의 질량 합과 (질량 x 위치) 합. 자식이 먼저 오도록 order 를 거꾸로 훑는다.
    std::vector<double> sm(robot.links.size(), 0.0);
    std::vector<V3> smp(robot.links.size());
    for (size_t i = 0; i < robot.links.size(); i++) {
        double m;
        V3 c;
        LinkMass(robot, k, i, &m, &c);
        sm[i] = m;
        smp[i] = c * m;
    }
    for (size_t o = robot.order.size(); o-- > 0;) {
        const urdf::Joint& j = robot.joints[static_cast<size_t>(robot.order[o])];
        const int pi = robot.FindLink(j.parent), ci = robot.FindLink(j.child);
        if (pi < 0 || ci < 0) continue;
        sm[static_cast<size_t>(pi)] += sm[static_cast<size_t>(ci)];
        smp[static_cast<size_t>(pi)] = smp[static_cast<size_t>(pi)] + smp[static_cast<size_t>(ci)];
    }

    // 움직이는 관절마다 자기 자식 쪽 전체가 만드는 중력 토크(힘)
    //   회전: tau = -a . ((sum m (p - o)) x g)      이동: tau = -a . (M g)
    std::vector<double> own(robot.joints.size(), 0.0);
    for (size_t ji = 0; ji < robot.joints.size(); ji++) {
        const urdf::Joint& j = robot.joints[ji];
        if (!j.IsMovable()) continue;
        const int ci = robot.FindLink(j.child);
        if (ci < 0) continue;
        const double M = sm[static_cast<size_t>(ci)];
        if (j.IsAngular()) {
            const V3 moment = Cross(smp[static_cast<size_t>(ci)] - k.originW[ji] * M, gv);
            own[ji] = -Dot(k.axisW[ji], moment);
        } else {
            own[ji] = -Dot(k.axisW[ji], gv * M);
        }
    }

    // 구동 관절의 결과. mimic 관절은 원본(마스터) 관절에 multiplier 배로 합친다 (q_mimic = mult * q_master + offset).
    // 한계에 걸려 값이 고정된 mimic 관절은 따로 처리하지 않는다.
    std::vector<double> total(robot.joints.size(), 0.0);
    for (size_t ji = 0; ji < robot.joints.size(); ji++) {
        if (!robot.joints[ji].IsMovable()) continue;
        size_t cur = ji;
        double factor = 1.0;
        for (size_t depth = 0; depth < robot.joints.size() && robot.joints[cur].hasMimic; depth++) {
            const int m = robot.FindJoint(robot.joints[cur].mimicJoint);
            if (m < 0) break;
            factor *= static_cast<double>(robot.joints[cur].mimicMultiplier);
            cur = static_cast<size_t>(m);
        }
        total[cur] += factor * own[ji];
    }
    const std::vector<int> act = robot.ActuatedJoints();
    for (size_t i = 0; i < act.size(); i++) out[robot.joints[static_cast<size_t>(act[i])].name] = total[static_cast<size_t>(act[i])];
    return out;
}

bool CenterOfMass(const urdf::Robot& robot, const urdf::JointValues& q, double outCom[3], double* outMass, bool clampToLimits) {
    const Kinematics k = Forward(robot, q, clampToLimits);
    double total = 0.0;
    V3 sum;
    for (size_t i = 0; i < robot.links.size(); i++) {
        double m;
        V3 c;
        LinkMass(robot, k, i, &m, &c);
        total += m;
        sum = sum + c * m;
    }
    if (!(total > 0.0)) return false;
    if (outCom) {
        outCom[0] = sum.x / total;
        outCom[1] = sum.y / total;
        outCom[2] = sum.z / total;
    }
    if (outMass) *outMass = total;
    return true;
}

}  // namespace dynamics
}  // namespace amr
