#include "CDynamics/Gravity.h"

#include <math.h>

#include "CBase/Util.h"

#include "DynMath.h"

namespace simul {
namespace dynamics {

using namespace detail;

namespace {

// 중력 계산에 공통으로 필요한 값: 순기구학, 링크 아래쪽(자손 전체)의 질량 합과 (질량 x 위치) 합,
// 움직이는 관절마다 자기 자식 쪽 전체가 만드는 토크(힘)
struct Common {
    Kinematics k;
    V3 g;
    simul::Array<double> sm;
    simul::Array<V3> smp;
    simul::Array<double> own;  // 관절별 (mimic 합산 전)
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
    simul::Array<double> total(robot.joints.size(), 0.0);
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
    const simul::Array<int> act = robot.ActuatedJoints();
    for (size_t i = 0; i < act.size(); i++) out[robot.joints[static_cast<size_t>(act[i])].name] = total[static_cast<size_t>(act[i])];
    return out;
}

simul::Array<JointLoad> GravityLoads(const urdf::Robot& robot, const urdf::JointValues& q, const Gravity& g, bool clampToLimits) {
    const Common c = Prepare(robot, q, g, clampToLimits);
    simul::Array<JointLoad> out(robot.joints.size());
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

simul::Array<LinkContribution> GravityContributions(const urdf::Robot& robot, const urdf::JointValues& q,
                                                   const simul::String& jointName, const Gravity& g, bool clampToLimits) {
    simul::Array<LinkContribution> out;
    const int ji = robot.FindJoint(jointName);
    if (ji < 0) return out;
    const urdf::Joint& j = robot.joints[static_cast<size_t>(ji)];
    if (!j.IsMovable()) return out;
    const int root = robot.FindLink(j.child);
    if (root < 0) return out;
    const Common c = Prepare(robot, q, g, clampToLimits);

    // 이 관절의 자식 링크에서 시작해 아래쪽 링크를 모두 모은다 (깊이 우선)
    simul::Array<int> stack(1, root), subtree;
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
    simul::Sort(subtree.begin(), subtree.end());
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
}  // namespace simul
