#include "CDynamics/Gravity.h"

#include <math.h>

#include <algorithm>

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

namespace {

// 중력 계산에 공통으로 필요한 값: 순기구학, 링크 아래쪽(자손 전체)의 질량 합과 (질량 x 위치) 합,
// 움직이는 관절마다 자기 자식 쪽 전체가 만드는 토크(힘)
struct Common {
    Kinematics k;
    V3 g;
    std::vector<double> sm;
    std::vector<V3> smp;
    std::vector<double> own;  // 관절별 (mimic 합산 전)
};

Common Prepare(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& grav, bool clamp) {
    Common c;
    c.k = Forward(robot, q, clamp);
    c.g = V3(grav.x, grav.y, grav.z);
    c.sm.assign(robot.links.size(), 0.0);
    c.smp.assign(robot.links.size(), V3());
    for (size_t i = 0; i < robot.links.size(); i++) {
        double m;
        V3 com;
        LinkMass(robot, c.k, i, &m, &com);
        c.sm[i] = m;
        c.smp[i] = com * m;
    }
    // 자식이 먼저 오도록 order 를 거꾸로 훑는다
    for (size_t o = robot.order.size(); o-- > 0;) {
        const urdf::Joint& j = robot.joints[static_cast<size_t>(robot.order[o])];
        const int pi = robot.FindLink(j.parent), ci = robot.FindLink(j.child);
        if (pi < 0 || ci < 0) continue;
        c.sm[static_cast<size_t>(pi)] += c.sm[static_cast<size_t>(ci)];
        c.smp[static_cast<size_t>(pi)] = c.smp[static_cast<size_t>(pi)] + c.smp[static_cast<size_t>(ci)];
    }
    //   회전: tau = -a . ((sum m (p - o)) x g)      이동: tau = -a . (M g)
    c.own.assign(robot.joints.size(), 0.0);
    for (size_t ji = 0; ji < robot.joints.size(); ji++) {
        const urdf::Joint& j = robot.joints[ji];
        if (!j.IsMovable()) continue;
        const int ci = robot.FindLink(j.child);
        if (ci < 0) continue;
        const double M = c.sm[static_cast<size_t>(ci)];
        if (j.IsAngular()) {
            const V3 moment = Cross(c.smp[static_cast<size_t>(ci)] - c.k.originW[ji] * M, c.g);
            c.own[ji] = -Dot(c.k.axisW[ji], moment);
        } else {
            c.own[ji] = -Dot(c.k.axisW[ji], c.g * M);
        }
    }
    return c;
}

}  // namespace

JointTorques GravityTorques(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g, bool clampToLimits) {
    JointTorques out;
    const Common c = Prepare(robot, q, g, clampToLimits);

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
        total[cur] += factor * c.own[ji];
    }
    const std::vector<int> act = robot.ActuatedJoints();
    for (size_t i = 0; i < act.size(); i++) out[robot.joints[static_cast<size_t>(act[i])].name] = total[static_cast<size_t>(act[i])];
    return out;
}

std::vector<JointLoad> GravityLoads(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g, bool clampToLimits) {
    const Common c = Prepare(robot, q, g, clampToLimits);
    std::vector<JointLoad> out(robot.joints.size());
    for (size_t ji = 0; ji < robot.joints.size(); ji++) {
        const urdf::Joint& j = robot.joints[ji];
        JointLoad& l = out[ji];
        l.name = j.name;
        l.movable = j.IsMovable();
        l.mimic = j.hasMimic;
        l.torque = c.own[ji];
        const int ci = robot.FindLink(j.child);
        if (ci < 0) continue;
        const double M = c.sm[static_cast<size_t>(ci)];
        const V3 force = c.g * (-M);
        // 관절 원점 기준 모멘트. 아직 움직이지 않는 관절(fixed 등)도 originW 는 순기구학에서 채워진다.
        const V3 moment = Cross(c.smp[static_cast<size_t>(ci)] - c.k.originW[ji] * M, c.g) * -1.0;
        l.force[0] = force.x;
        l.force[1] = force.y;
        l.force[2] = force.z;
        l.moment[0] = moment.x;
        l.moment[1] = moment.y;
        l.moment[2] = moment.z;
    }
    return out;
}

std::vector<LinkContribution> GravityContributions(const urdf::Robot& robot, const urdf::JointValues& q,
                                                   const std::string& jointName, const Gravity& g, bool clampToLimits) {
    std::vector<LinkContribution> out;
    const int ji = robot.FindJoint(jointName);
    if (ji < 0) return out;
    const urdf::Joint& j = robot.joints[static_cast<size_t>(ji)];
    if (!j.IsMovable()) return out;
    const int root = robot.FindLink(j.child);
    if (root < 0) return out;
    const Common c = Prepare(robot, q, g, clampToLimits);

    // 이 관절의 자식 링크에서 시작해 아래쪽 링크를 모두 모은다 (깊이 우선)
    std::vector<int> stack(1, root), subtree;
    while (!stack.empty()) {
        const int l = stack.back();
        stack.pop_back();
        subtree.push_back(l);
        for (size_t k = 0; k < robot.joints.size(); k++)
            if (robot.FindLink(robot.joints[k].parent) == l) {
                const int ch = robot.FindLink(robot.joints[k].child);
                if (ch >= 0) stack.push_back(ch);
            }
    }
    std::sort(subtree.begin(), subtree.end());
    const V3 o = c.k.originW[static_cast<size_t>(ji)];
    const V3 a = c.k.axisW[static_cast<size_t>(ji)];
    for (size_t i = 0; i < subtree.size(); i++) {
        double m;
        V3 com;
        LinkMass(robot, c.k, static_cast<size_t>(subtree[i]), &m, &com);
        if (!(m > 0.0)) continue;
        LinkContribution lc;
        lc.link = robot.links[static_cast<size_t>(subtree[i])].name;
        lc.value = j.IsAngular() ? -Dot(a, Cross(com - o, c.g * m)) : -Dot(a, c.g * m);
        out.push_back(lc);
    }
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
