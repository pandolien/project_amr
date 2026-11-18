#include "CDynamics/Dynamics.h"

#include <math.h>

#include "CBase/Util.h"
#include "CBase/Math.h"
#include "CBase/Map.h"

#include "DynMath.h"

namespace simul {
namespace dynamics {

using namespace detail;

namespace {

// 3x3 대칭 관성 텐서를 월드 좌표로: R_link * R_origin * I * (R_link * R_origin)^T
M3 WorldInertia(const urdf::Link& l, const M3& Rlink) {
    M3 I;
    const urdf::Inertial& in = l.inertial;
    I.m[0][0] = in.ixx; I.m[0][1] = in.ixy; I.m[0][2] = in.ixz;
    I.m[1][0] = in.ixy; I.m[1][1] = in.iyy; I.m[1][2] = in.iyz;
    I.m[2][0] = in.ixz; I.m[2][1] = in.iyz; I.m[2][2] = in.izz;
    const M3 R = Rlink * RotOf(in.origin);
    M3 Rt;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Rt.m[i][j] = R.m[j][i];
    return R * I * Rt;
}

// 한 자세(q)에서의 재귀 뉴턴-오일러. 순기구학은 한 번만 하고 속도/가속도/중력만 바꿔 여러 번 부른다.
struct Rnea {
    const urdf::Robot& robot;
    Kinematics k;
    simul::Array<int> act;           // 구동 관절 (robot.joints 인덱스)
    simul::Array<MasterOf> master;   // 관절마다 (robot.joints 인덱스 기준)
    simul::Array<int> actPos;        // 관절 -> act 안의 위치 (구동 관절이 아니면 -1)
    simul::Array<double> mass;       // 링크별 질량
    simul::Array<V3> comOff;         // 링크 좌표 -> 월드 방향으로 돌린 질량중심 오프셋 (링크 원점 기준)
    simul::Array<M3> inertiaW;

    Rnea(const urdf::Robot& r, const urdf::JointValues& q, bool clamp) : robot(r), k(Forward(r, q, clamp)) {
        act = r.ActuatedJoints();
        actPos.assign(r.joints.size(), -1);
        for (size_t i = 0; i < act.size(); i++) actPos[static_cast<size_t>(act[i])] = static_cast<int>(i);
        master.resize(r.joints.size());
        for (size_t j = 0; j < r.joints.size(); j++) master[j] = FindMaster(r, j);
        mass.assign(r.links.size(), 0.0);
        comOff.assign(r.links.size(), V3());
        inertiaW.assign(r.links.size(), M3());
        for (size_t i = 0; i < r.links.size(); i++) {
            const urdf::Link& l = r.links[i];
            if (!l.hasInertial || !(l.inertial.mass > 0.0f)) continue;
            mass[i] = static_cast<double>(l.inertial.mass);
            comOff[i] = k.R[i] * TransOf(l.inertial.origin);
            inertiaW[i] = WorldInertia(l, k.R[i]);
        }
    }

    // qd, qdd: 구동 관절 순서의 값. gravity: 떨어지는 방향의 가속도 벡터 (0 벡터면 중력 없음).
    // 반환: 구동 관절별 토크/힘 (mimic 관절의 몫은 마스터에 배율을 곱해 합친다)
    simul::Array<double> Run(const simul::Array<double>& qd, const simul::Array<double>& qdd, const V3& gravity) const {
        const size_t nl = robot.links.size(), nj = robot.joints.size();
        simul::Array<V3> w(nl), al(nl), ac(nl);
        // 루트: 정지. 중력은 루트가 위로 -g 만큼 가속한다고 놓는 방법으로 넣는다.
        for (size_t i = 0; i < nl; i++) ac[i] = V3(-gravity.x, -gravity.y, -gravity.z);
        // 앞으로: 속도와 가속도 전파
        for (size_t o = 0; o < robot.order.size(); o++) {
            const size_t ji = static_cast<size_t>(robot.order[o]);
            const urdf::Joint& j = robot.joints[ji];
            const int pi = robot.FindLink(j.parent), ci = robot.FindLink(j.child);
            if (pi < 0 || ci < 0) continue;
            const size_t p = static_cast<size_t>(pi), c = static_cast<size_t>(ci);
            double jd = 0.0, jdd = 0.0;
            if (j.IsMovable()) {
                const MasterOf& m = master[ji];
                const int pos = actPos[static_cast<size_t>(m.index)];
                if (pos >= 0) {
                    jd = m.factor * qd[static_cast<size_t>(pos)];
                    jdd = m.factor * qdd[static_cast<size_t>(pos)];
                }
            }
            const V3 r = k.p[c] - k.p[p];
            const V3 aP = ac[p] + Cross(al[p], r) + Cross(w[p], Cross(w[p], r));
            const V3 ax = k.axisW[ji];
            if (j.IsMovable() && j.IsAngular()) {
                w[c] = w[p] + ax * jd;
                al[c] = al[p] + ax * jdd + Cross(w[p], ax * jd);
                ac[c] = aP;
            } else if (j.IsMovable()) {  // 이동 관절
                w[c] = w[p];
                al[c] = al[p];
                ac[c] = aP + Cross(w[p], ax * jd) * 2.0 + ax * jdd;
            } else {
                w[c] = w[p];
                al[c] = al[p];
                ac[c] = aP;
            }
        }
        // 링크 하나가 자기 질량 때문에 필요로 하는 힘 F 와 (질량중심 기준) 모멘트 N, 그리고 링크 원점 기준 모멘트
        simul::Array<V3> f(nl), n(nl);
        for (size_t i = 0; i < nl; i++) {
            if (!(mass[i] > 0.0)) continue;
            const V3 rc = comOff[i];
            const V3 acm = ac[i] + Cross(al[i], rc) + Cross(w[i], Cross(w[i], rc));
            const V3 F = acm * mass[i];
            const V3 N = inertiaW[i] * al[i] + Cross(w[i], inertiaW[i] * w[i]);
            f[i] = F;
            n[i] = N + Cross(rc, F);
        }
        // 뒤로: 자식의 합계를 부모에 더하면서 관절 토크를 읽는다
        simul::Array<double> tauJoint(nj, 0.0);
        for (size_t o = robot.order.size(); o-- > 0;) {
            const size_t ji = static_cast<size_t>(robot.order[o]);
            const urdf::Joint& j = robot.joints[ji];
            const int pi = robot.FindLink(j.parent), ci = robot.FindLink(j.child);
            if (pi < 0 || ci < 0) continue;
            const size_t p = static_cast<size_t>(pi), c = static_cast<size_t>(ci);
            if (j.IsMovable()) tauJoint[ji] = j.IsAngular() ? Dot(k.axisW[ji], n[c]) : Dot(k.axisW[ji], f[c]);
            f[p] = f[p] + f[c];
            n[p] = n[p] + n[c] + Cross(k.p[c] - k.p[p], f[c]);
        }
        simul::Array<double> out(act.size(), 0.0);
        for (size_t ji = 0; ji < nj; ji++) {
            if (!robot.joints[ji].IsMovable()) continue;
            const MasterOf& m = master[ji];
            const int pos = actPos[static_cast<size_t>(m.index)];
            if (pos >= 0) out[static_cast<size_t>(pos)] += m.factor * tauJoint[ji];
        }
        return out;
    }
};

simul::Array<double> ToVec(const urdf::Robot& robot, const simul::Array<int>& act, const JointTorques& m) {
    simul::Array<double> v(act.size(), 0.0);
    for (size_t i = 0; i < act.size(); i++) {
        const JointTorques::const_iterator it = m.find(robot.joints[static_cast<size_t>(act[i])].name);
        if (it != m.end()) v[i] = it->second;
    }
    return v;
}

JointTorques ToMap(const urdf::Robot& robot, const simul::Array<int>& act, const simul::Array<double>& v) {
    JointTorques m;
    for (size_t i = 0; i < act.size(); i++) m[robot.joints[static_cast<size_t>(act[i])].name] = v[i];
    return m;
}

V3 GVec(const Gravity& g) { return V3(g.x, g.y, g.z); }

// n x n 선형 방정식 A x = b (가우스 소거, 부분 피벗). 대각이 거의 0 인(관성이 없는) 자유도는 x = 0 으로 둔다.
simul::Array<double> Solve(simul::Array<double> A, simul::Array<double> b, size_t n) {
    simul::Array<bool> frozen(n, false);
    for (size_t i = 0; i < n; i++) {
        if (!(A[i * n + i] > 1e-9)) {
            frozen[i] = true;
            for (size_t j = 0; j < n; j++) {
                A[i * n + j] = 0.0;
                A[j * n + i] = 0.0;
            }
            A[i * n + i] = 1.0;
            b[i] = 0.0;
        }
    }
    for (size_t c = 0; c < n; c++) {
        size_t piv = c;
        for (size_t r = c + 1; r < n; r++)
            if (fabs(A[r * n + c]) > fabs(A[piv * n + c])) piv = r;
        if (fabs(A[piv * n + c]) < 1e-14) continue;
        if (piv != c) {
            for (size_t j = 0; j < n; j++) simul::Swap(A[c * n + j], A[piv * n + j]);
            simul::Swap(b[c], b[piv]);
        }
        for (size_t r = c + 1; r < n; r++) {
            const double f = A[r * n + c] / A[c * n + c];
            if (f == 0.0) continue;
            for (size_t j = c; j < n; j++) A[r * n + j] -= f * A[c * n + j];
            b[r] -= f * b[c];
        }
    }
    simul::Array<double> x(n, 0.0);
    for (size_t r = n; r-- > 0;) {
        double s = b[r];
        for (size_t j = r + 1; j < n; j++) s -= A[r * n + j] * x[j];
        x[r] = fabs(A[r * n + r]) < 1e-14 ? 0.0 : s / A[r * n + r];
    }
    return x;
}

// 마찰 토크 (관절마다 -(d qd + f tanh(qd/eps)) 를 마스터에 배율을 곱해 합친다)
simul::Array<double> PassiveTorques(const urdf::Robot& robot, const Rnea& dyn, const simul::Array<double>& qd, const DynamicsOptions& opt) {
    simul::Array<double> out(dyn.act.size(), 0.0);
    const double eps = opt.frictionSmoothing > 0.0 ? opt.frictionSmoothing : 1e-6;
    for (size_t ji = 0; ji < robot.joints.size(); ji++) {
        const urdf::Joint& j = robot.joints[ji];
        if (!j.IsMovable()) continue;
        const MasterOf& m = dyn.master[ji];
        const int pos = dyn.actPos[static_cast<size_t>(m.index)];
        if (pos < 0) continue;
        const double v = m.factor * qd[static_cast<size_t>(pos)];
        const double d = j.damping > 0.0f ? static_cast<double>(j.damping) : opt.defaultDamping;
        const double tau = d * v + static_cast<double>(j.friction) * tanh(v / eps);
        out[static_cast<size_t>(pos)] += m.factor * tau;
    }
    return out;
}

MassMatrix BuildMass(const urdf::Robot& robot, const Rnea& dyn) {
    MassMatrix mm;
    const size_t n = dyn.act.size();
    for (size_t i = 0; i < n; i++) mm.joints.push_back(robot.joints[static_cast<size_t>(dyn.act[i])].name);
    mm.m.assign(n * n, 0.0);
    const simul::Array<double> zero(n, 0.0);
    for (size_t c = 0; c < n; c++) {
        simul::Array<double> e(n, 0.0);
        e[c] = 1.0;
        const simul::Array<double> col = dyn.Run(zero, e, V3());
        for (size_t r = 0; r < n; r++) mm.m[r * n + c] = col[r];
    }
    return mm;
}

// atLimit[i]: 0 이면 자유, +1 이면 위쪽 limit 에 닿아 있고 속도가 0, -1 이면 아래쪽 limit 에 닿아 있고 속도가 0.
// limit 에 닿은 관절이 limit 쪽으로 더 밀리면(가속도가 limit 쪽) 그 관절을 고정한(가속도 0) 나머지 관절의 운동방정식을 다시 푼다.
// 이렇게 해야 막힌 관절을 통해 힘이 올바르게 전달되고, 막힌 관절에 매달린 다른 관절들이 제 운동을 한다
// (예전처럼 막힌 관절을 자유롭다고 보고 풀면 다른 관절의 가속도가 틀어진다).
simul::Array<double> Accel(const urdf::Robot& robot, const Rnea& dyn, const simul::Array<double>& qd, const simul::Array<double>& tau, const V3& g,
                          const DynamicsOptions& opt, const simul::Array<int>* atLimit = nullptr) {
    const size_t n = dyn.act.size();
    if (n == 0) return simul::Array<double>();
    const MassMatrix mm = BuildMass(robot, dyn);
    const simul::Array<double> zero(n, 0.0);
    const simul::Array<double> bias = dyn.Run(qd, zero, g);  // C qd + g
    const simul::Array<double> pas = PassiveTorques(robot, dyn, qd, opt);
    simul::Array<double> rhs(n);
    for (size_t i = 0; i < n; i++) rhs[i] = tau[i] - bias[i] - pas[i];
    simul::Array<double> a = Solve(mm.m, rhs, n);
    if (!atLimit) return a;

    simul::Array<bool> locked(n, false);
    bool any = false;
    for (size_t i = 0; i < n; i++)
        if ((*atLimit)[i] != 0 && a[i] * (*atLimit)[i] > 0.0) { locked[i] = true; any = true; }   // limit 쪽으로 밀리는 관절
    if (!any) return a;
    for (size_t iter = 0; iter < n + 1; iter++) {
        // 고정한 관절의 행/열을 지우고(가속도 0) 나머지를 푼다
        simul::Array<double> A = mm.m, b = rhs;
        for (size_t i = 0; i < n; i++) {
            if (!locked[i]) continue;
            for (size_t j = 0; j < n; j++) { A[i * n + j] = 0.0; A[j * n + i] = 0.0; }
            A[i * n + i] = 1.0;
            b[i] = 0.0;
        }
        a = Solve(A, b, n);
        // 구속력 lambda = (M a - rhs): limit 이 관절을 안쪽으로 미는 쪽이어야 한다. 반대면(관절이 limit 에서 떨어지려는 것) 풀어 준다.
        bool released = false;
        for (size_t i = 0; i < n; i++) {
            if (!locked[i]) continue;
            double lambda = -rhs[i];
            for (size_t j = 0; j < n; j++) lambda += mm.m[i * n + j] * a[j];
            if (lambda * (*atLimit)[i] > 1e-12) {   // 위쪽 limit 인데 위로 당기는 구속력 = 말이 안 되므로 풀어 준다
                locked[i] = false;
                released = true;
            }
        }
        if (!released) break;
    }
    return a;
}

}  // namespace

simul::Array<simul::String> ActuatedJointNames(const urdf::Robot& robot) {
    simul::Array<simul::String> names;
    const simul::Array<int> act = robot.ActuatedJoints();
    for (size_t i = 0; i < act.size(); i++) names.push_back(robot.joints[static_cast<size_t>(act[i])].name);
    return names;
}

JointTorques InverseDynamics(const urdf::Robot& robot, const urdf::JointValues& q, const JointTorques& qd, const JointTorques& qdd,
                             const Gravity& g, bool clampToLimits) {
    const Rnea dyn(robot, q, clampToLimits);
    return ToMap(robot, dyn.act, dyn.Run(ToVec(robot, dyn.act, qd), ToVec(robot, dyn.act, qdd), GVec(g)));
}

MassMatrix ComputeMassMatrix(const urdf::Robot& robot, const urdf::JointValues& q, bool clampToLimits) {
    const Rnea dyn(robot, q, clampToLimits);
    return BuildMass(robot, dyn);
}

JointTorques ForwardDynamics(const urdf::Robot& robot, const urdf::JointValues& q, const JointTorques& qd, const JointTorques& tau,
                             const Gravity& g, const DynamicsOptions& opt) {
    const Rnea dyn(robot, q, true);
    return ToMap(robot, dyn.act, Accel(robot, dyn, ToVec(robot, dyn.act, qd), ToVec(robot, dyn.act, tau), GVec(g), opt));
}

// ---- 시뮬레이터

Simulator::Simulator() : ready_(false) {}

void Simulator::Reset(const urdf::Robot& robot, const urdf::JointValues& q) {
    robot_ = robot;
    act_ = robot_.ActuatedJoints();
    q_.assign(act_.size(), 0.0);
    qd_.assign(act_.size(), 0.0);
    for (size_t i = 0; i < act_.size(); i++) {
        const urdf::Joint& j = robot_.joints[static_cast<size_t>(act_[i])];
        const urdf::JointValues::const_iterator it = q.find(j.name);
        if (it != q.end()) q_[i] = static_cast<double>(it->second);
    }
    ready_ = !act_.empty();
}

void Simulator::UpdateRobot(const urdf::Robot& robot) {
    const simul::Array<int> act = robot.ActuatedJoints();
    bool same = ready_ && act.size() == act_.size();
    for (size_t i = 0; same && i < act.size(); i++) same = robot.joints[static_cast<size_t>(act[i])].name == robot_.joints[static_cast<size_t>(act_[i])].name;
    if (!same) {
        Reset(robot, urdf::JointValues());
        return;
    }
    robot_ = robot;
    act_ = act;
}

void Simulator::SetValue(const simul::String& joint, double value) {
    for (size_t i = 0; i < act_.size(); i++) {
        const urdf::Joint& j = robot_.joints[static_cast<size_t>(act_[i])];
        if (j.name != joint) continue;
        if (j.type != urdf::kJointContinuous && j.hasLimit) value = simul::Max(static_cast<double>(j.lower), simul::Min(static_cast<double>(j.upper), value));
        q_[i] = value;
        qd_[i] = 0.0;
    }
}

urdf::JointValues Simulator::toValues(const simul::Array<double>& q) const {
    urdf::JointValues v;
    for (size_t i = 0; i < act_.size(); i++) v[robot_.joints[static_cast<size_t>(act_[i])].name] = static_cast<float>(q[i]);
    return v;
}

urdf::JointValues Simulator::Values() const { return toValues(q_); }

JointTorques Simulator::Velocities() const { return ToMap(robot_, act_, qd_); }

void Simulator::derivative(const simul::Array<double>& q, const simul::Array<double>& qd, simul::Array<double>* dq, simul::Array<double>* dqd) const {
    const Rnea dyn(robot_, toValues(q), false);
    *dq = qd;
    // limit 에 닿아 멈춘 관절 표시 (위쪽 +1, 아래쪽 -1)
    simul::Array<int> atLimit(act_.size(), 0);
    for (size_t i = 0; i < act_.size(); i++) {
        const urdf::Joint& j = robot_.joints[static_cast<size_t>(act_[i])];
        if (j.type == urdf::kJointContinuous || !j.hasLimit || qd[i] != 0.0) continue;
        if (q[i] >= static_cast<double>(j.upper) - 1e-9) atLimit[i] = 1;
        else if (q[i] <= static_cast<double>(j.lower) + 1e-9) atLimit[i] = -1;
    }
    *dqd = Accel(robot_, dyn, qd, ToVec(robot_, act_, tau_), GVec(gravity_), options_, &atLimit);
}

void Simulator::Step(double dt) {
    if (!ready_ || !(dt > 0.0)) return;
    const int sub = simul::Max(1, static_cast<int>(ceil(dt * 500.0)));
    const double h = dt / sub;
    const size_t n = q_.size();
    simul::Array<size_t> hit;
    for (int s = 0; s < sub; s++) {
        simul::Array<double> k1q, k1v, k2q, k2v, k3q, k3v, k4q, k4v, q2(n), v2(n);
        derivative(q_, qd_, &k1q, &k1v);
        for (size_t i = 0; i < n; i++) { q2[i] = q_[i] + 0.5 * h * k1q[i]; v2[i] = qd_[i] + 0.5 * h * k1v[i]; }
        derivative(q2, v2, &k2q, &k2v);
        for (size_t i = 0; i < n; i++) { q2[i] = q_[i] + 0.5 * h * k2q[i]; v2[i] = qd_[i] + 0.5 * h * k2v[i]; }
        derivative(q2, v2, &k3q, &k3v);
        for (size_t i = 0; i < n; i++) { q2[i] = q_[i] + h * k3q[i]; v2[i] = qd_[i] + h * k3v[i]; }
        derivative(q2, v2, &k4q, &k4v);
        for (size_t i = 0; i < n; i++) {
            q_[i] += h / 6.0 * (k1q[i] + 2 * k2q[i] + 2 * k3q[i] + k4q[i]);
            qd_[i] += h / 6.0 * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]);
            if (!simul::IsFinite(q_[i]) || !simul::IsFinite(qd_[i])) {  // 수치가 터지면 그 관절을 멈춘다
                q_[i] = 0.0;
                qd_[i] = 0.0;
            }
            const urdf::Joint& j = robot_.joints[static_cast<size_t>(act_[i])];
            if (j.type != urdf::kJointContinuous && j.hasLimit) {  // limit 에 닿으면 멈춘다 (아래에서 충격량을 다른 관절에도 전달)
                if (q_[i] > static_cast<double>(j.upper)) { q_[i] = static_cast<double>(j.upper); if (qd_[i] > 0) hit.push_back(i); }
                if (q_[i] < static_cast<double>(j.lower)) { q_[i] = static_cast<double>(j.lower); if (qd_[i] < 0) hit.push_back(i); }
            }
        }
        // 완전 비탄성 충돌: 닿은 관절의 속도만 0 으로 만드는 충격량을 가하면 다른 관절의 속도도 질량 행렬을 따라 바뀐다
        //   M dqd = lambda e_i,  (qd_i + dqd_i) = 0  ->  dqd = -qd_i / (M^-1)_ii * M^-1 e_i
        //   (예전에는 그 관절 속도만 0 으로 했다: 막힌 관절에 매달린 다른 관절이 휘두르는 운동량이 같이 사라졌다)
        for (size_t k = 0; k < hit.size(); k++) {
            const size_t i = hit[k];
            const MassMatrix mm = ComputeMassMatrix(robot_, toValues(q_), false);
            simul::Array<double> e(n, 0.0);
            e[i] = 1.0;
            const simul::Array<double> x = Solve(mm.m, e, n);
            if (!(x[i] > 1e-12)) { qd_[i] = 0.0; continue; }
            const double lambda = -qd_[i] / x[i];
            for (size_t r = 0; r < n; r++) qd_[r] += lambda * x[r];
            qd_[i] = 0.0;
        }
        hit.clear();
    }
}

double Simulator::KineticEnergy() const {
    if (!ready_) return 0.0;
    const MassMatrix mm = ComputeMassMatrix(robot_, toValues(q_), false);
    const size_t n = mm.n();
    double e = 0.0;
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++) e += 0.5 * qd_[i] * mm.At(i, j) * qd_[j];
    return e;
}

double Simulator::PotentialEnergy() const {
    if (!ready_) return 0.0;
    double com[3], mass = 0.0;
    if (!CenterOfMass(robot_, toValues(q_), com, &mass, false)) return 0.0;
    return -mass * (gravity_.x * com[0] + gravity_.y * com[1] + gravity_.z * com[2]);
}

}  // namespace dynamics
}  // namespace simul
