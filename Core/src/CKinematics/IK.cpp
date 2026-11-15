#include "CKinematics/IK.h"

#include <math.h>

#include "CBase/Math.h"
#include "CBase/Util.h"

#include "DynMath.h"

namespace amr {
namespace kinematics {

using namespace amr::dynamics::detail;

namespace {

// 작업 공간 오차 6 벡터: [위치 오차 (목표 - 현재), 회전 오차 (회전 벡터)]
struct TaskError {
    V3 pos;
    V3 rot;
};

// R 의 회전 벡터 (축 * 각도, 각도 0..pi)
V3 RotationVector(const M3& R) {
    const double tr = R.m[0][0] + R.m[1][1] + R.m[2][2];
    const V3 v(0.5 * (R.m[2][1] - R.m[1][2]), 0.5 * (R.m[0][2] - R.m[2][0]), 0.5 * (R.m[1][0] - R.m[0][1]));  // sin(t) * 축
    const double s = Sqrt(Dot(v, v));
    double c = 0.5 * (tr - 1.0);
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    const double angle = atan2(s, c);
    if (s > 1e-9) return v * (angle / s);
    if (c > 0.0) return V3();  // 각도 0
    // 각도가 pi 근처: 축은 (R + I)/2 = a a^T 의 가장 큰 열에서 얻는다
    int k = 0;
    for (int i = 1; i < 3; i++)
        if (R.m[i][i] > R.m[k][k]) k = i;
    V3 a(0.5 * (R.m[0][k] + (k == 0 ? 1.0 : 0.0)), 0.5 * (R.m[1][k] + (k == 1 ? 1.0 : 0.0)), 0.5 * (R.m[2][k] + (k == 2 ? 1.0 : 0.0)));
    const double n = Sqrt(Dot(a, a));
    if (n < 1e-12) return V3();
    return a * (angle / n);
}

M3 Transpose(const M3& a) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
    return r;
}

TaskError ComputeError(const Kinematics& k, size_t link, const V3& tp, const M3& tR) {
    TaskError e;
    e.pos = tp - k.p[link];
    e.rot = RotationVector(tR * Transpose(k.R[link]));  // 월드 기준 회전: R_target * R_current^T
    return e;
}

// 연립방정식 A x = b (n x n, 행 우선) 을 부분 피벗 가우스 소거로 푼다. 특이하면 false.
bool SolveLinear(amr::Array<double>* A, amr::Array<double>* b, size_t n) {
    amr::Array<double>& a = *A;
    amr::Array<double>& x = *b;
    for (size_t c = 0; c < n; c++) {
        size_t piv = c;
        for (size_t r = c + 1; r < n; r++)
            if (Abs(a[r * n + c]) > Abs(a[piv * n + c])) piv = r;
        if (Abs(a[piv * n + c]) < 1e-14) return false;
        if (piv != c) {
            for (size_t k = 0; k < n; k++) Swap(a[c * n + k], a[piv * n + k]);
            Swap(x[c], x[piv]);
        }
        for (size_t r = c + 1; r < n; r++) {
            const double f = a[r * n + c] / a[c * n + c];
            if (f == 0.0) continue;
            for (size_t k = c; k < n; k++) a[r * n + k] -= f * a[c * n + k];
            x[r] -= f * x[c];
        }
    }
    for (size_t ci = n; ci-- > 0;) {
        double s = x[ci];
        for (size_t k = ci + 1; k < n; k++) s -= a[ci * n + k] * x[k];
        x[ci] = s / a[ci * n + ci];
    }
    return true;
}

struct Variable {
    int joint;        // 구동 관절 인덱스
    double lower, upper;
    bool limited;
    bool angular;
};

double ErrorNorm(const TaskError& e, const IkOptions& o) {
    double s = Dot(e.pos, e.pos);
    if (o.useOrientation) s += o.orientationWeight * o.orientationWeight * Dot(e.rot, e.rot);
    return Sqrt(s);
}

void SetJointValue(urdf::JointValues* q, const urdf::Robot& robot, int joint, double v) {
    (*q)[robot.joints[static_cast<size_t>(joint)].name] = static_cast<float>(v);
}

}  // namespace

void PoseError(const urdf::Robot& robot, const urdf::JointValues& q, int link, const dhMat& target, double* positionError,
               double* orientationError) {
    const Kinematics k = Forward(robot, q, true);
    const TaskError e = ComputeError(k, static_cast<size_t>(link), TransOf(target), RotOf(target));
    if (positionError) *positionError = Sqrt(Dot(e.pos, e.pos));
    if (orientationError) *orientationError = Sqrt(Dot(e.rot, e.rot));
}

namespace {

unsigned NextRandom(unsigned* s) {
    *s = *s * 1664525u + 1013904223u;
    return *s >> 8;
}

// 한 번 푼다. randomState 가 NULL 이 아니면 시작 값을 한계 안의 무작위 값으로 바꾼다 (재시도용).
IkResult SolveOnce(const urdf::Robot& robot, const urdf::JointValues& start, int link, const dhMat& target, const IkOptions& opt,
                   unsigned* randomState) {
    IkResult res;
    res.q = start;
    if (link < 0 || static_cast<size_t>(link) >= robot.links.size()) return res;
    const size_t L = static_cast<size_t>(link);
    const V3 tp = TransOf(target);
    const M3 tR = RotOf(target);

    // 목표 링크까지의 관절 사슬 (조상 관절)
    amr::Array<char> inChain(robot.joints.size(), 0);
    {
        int cur = link;
        for (size_t guard = 0; guard <= robot.joints.size(); guard++) {
            int parentJoint = -1;
            for (size_t j = 0; j < robot.joints.size(); j++)
                if (robot.FindLink(robot.joints[j].child) == cur) { parentJoint = static_cast<int>(j); break; }
            if (parentJoint < 0) break;
            inChain[static_cast<size_t>(parentJoint)] = 1;
            cur = robot.FindLink(robot.joints[static_cast<size_t>(parentJoint)].parent);
            if (cur < 0) break;
        }
    }

    // 변수 = 구동 관절 중 사슬에 영향을 주는 것 (자신 또는 mimic 으로 딸린 관절이 사슬에 있는 것)
    const amr::Array<int> actuated = robot.ActuatedJoints();
    amr::Array<Variable> vars;
    amr::Array<int> varOfActuated(robot.joints.size(), -1);
    amr::Array<double> factorOf(robot.joints.size(), 0.0);  // 관절 -> 마스터 변수로의 배율
    amr::Array<int> masterOf(robot.joints.size(), -1);
    for (size_t j = 0; j < robot.joints.size(); j++) {
        if (!robot.joints[j].IsMovable()) continue;
        const MasterOf m = FindMaster(robot, j);
        masterOf[j] = m.index;
        factorOf[j] = m.factor;
    }
    for (size_t a = 0; a < actuated.size(); a++) {
        bool used = false;
        for (size_t j = 0; j < robot.joints.size(); j++)
            if (inChain[j] && masterOf[j] == actuated[a] && robot.joints[j].IsMovable()) used = true;
        if (!used) continue;
        const urdf::Joint& jt = robot.joints[static_cast<size_t>(actuated[a])];
        Variable v;
        v.joint = actuated[a];
        v.angular = jt.IsAngular();
        v.limited = opt.respectLimits && jt.type != urdf::kJointContinuous && jt.hasLimit;
        v.lower = static_cast<double>(jt.lower);
        v.upper = static_cast<double>(jt.upper);
        varOfActuated[static_cast<size_t>(actuated[a])] = static_cast<int>(vars.size());
        vars.push_back(v);
    }
    const size_t n = vars.size();
    if (n == 0) return res;

    // 현재 값 (한계 안으로)
    amr::Array<double> x(n, 0.0);
    for (size_t i = 0; i < n; i++) {
        const urdf::Joint& jt = robot.joints[static_cast<size_t>(vars[i].joint)];
        double v = static_cast<double>(robot.EffectiveJointValue(start, vars[i].joint, false));
        if (vars[i].limited) {
            if (v < vars[i].lower) v = vars[i].lower;
            if (v > vars[i].upper) v = vars[i].upper;
        }
        if (randomState) {
            const double u = static_cast<double>(NextRandom(randomState) & 0xFFFF) / 65536.0;
            const double lo = vars[i].limited ? vars[i].lower : -3.14159265358979;
            const double hi = vars[i].limited ? vars[i].upper : 3.14159265358979;
            v = vars[i].angular || vars[i].limited ? lo + (hi - lo) * u : v;  // 한계 없는 이동 관절은 그대로
        }
        x[i] = v;
        (void)jt;
    }
    urdf::JointValues q = start;
    for (size_t i = 0; i < n; i++) SetJointValue(&q, robot, vars[i].joint, x[i]);

    const size_t m = opt.useOrientation ? 6 : 3;
    double lambda = opt.initialDamping;
    Kinematics kin = Forward(robot, q, opt.respectLimits);
    TaskError err = ComputeError(kin, L, tp, tR);
    double cost = ErrorNorm(err, opt);

    int iter = 0;
    for (; iter < opt.maxIterations; iter++) {
        const double perr = Sqrt(Dot(err.pos, err.pos));
        const double rerr = Sqrt(Dot(err.rot, err.rot));
        if (perr <= opt.positionTolerance && (!opt.useOrientation || rerr <= opt.orientationTolerance)) break;

        // 자코비안 (m x n): 열 = 변수 하나가 끝단을 움직이는 방식
        amr::Array<double> J(m * n, 0.0);
        for (size_t j = 0; j < robot.joints.size(); j++) {
            if (!inChain[j] || !robot.joints[j].IsMovable()) continue;
            const int vi = varOfActuated[static_cast<size_t>(masterOf[j])];
            if (vi < 0) continue;
            const double f = factorOf[j];
            const V3 a = kin.axisW[j];
            V3 lin, ang;
            if (robot.joints[j].IsAngular()) {
                lin = Cross(a, kin.p[L] - kin.originW[j]);
                ang = a;
            } else {
                lin = a;
            }
            const size_t c = static_cast<size_t>(vi);
            J[0 * n + c] += f * lin.x;
            J[1 * n + c] += f * lin.y;
            J[2 * n + c] += f * lin.z;
            if (opt.useOrientation) {
                J[3 * n + c] += f * opt.orientationWeight * ang.x;
                J[4 * n + c] += f * opt.orientationWeight * ang.y;
                J[5 * n + c] += f * opt.orientationWeight * ang.z;
            }
        }
        amr::Array<double> e(m, 0.0);
        e[0] = err.pos.x; e[1] = err.pos.y; e[2] = err.pos.z;
        if (opt.useOrientation) {
            e[3] = opt.orientationWeight * err.rot.x;
            e[4] = opt.orientationWeight * err.rot.y;
            e[5] = opt.orientationWeight * err.rot.z;
        }

        // 한계에 걸린 변수를 고정하며 푼다 (활성 집합). 고정된 변수는 열을 0 으로 둔다.
        amr::Array<char> fixedVar(n, 0);
        amr::Array<double> dx(n, 0.0);
        bool solved = false;
        for (size_t pass = 0; pass <= n; pass++) {
            amr::Array<double> A(m * m, 0.0), y = e;
            for (size_t r = 0; r < m; r++)
                for (size_t c2 = 0; c2 < m; c2++) {
                    double s = 0.0;
                    for (size_t k = 0; k < n; k++)
                        if (!fixedVar[k]) s += J[r * n + k] * J[c2 * n + k];
                    A[r * m + c2] = s + (r == c2 ? lambda * lambda : 0.0);
                }
            if (!SolveLinear(&A, &y, m)) break;
            for (size_t k = 0; k < n; k++) {
                double s = 0.0;
                if (!fixedVar[k])
                    for (size_t r = 0; r < m; r++) s += J[r * n + k] * y[r];
                dx[k] = s;
            }
            // 한 번에 너무 많이 움직이지 않게
            double big = 0.0;
            for (size_t k = 0; k < n; k++) big = Max(big, Abs(dx[k]));
            if (big > opt.maxStep) {
                const double sc = opt.maxStep / big;
                for (size_t k = 0; k < n; k++) dx[k] *= sc;
            }
            // 한계를 넘는 변수가 있으면 한계에 고정하고 다시 푼다
            bool violated = false;
            for (size_t k = 0; k < n; k++) {
                if (fixedVar[k] || !vars[k].limited) continue;
                const double nx = x[k] + dx[k];
                if (nx < vars[k].lower - 1e-12 || nx > vars[k].upper + 1e-12) {
                    fixedVar[k] = 1;
                    violated = true;
                }
            }
            if (!violated) { solved = true; break; }
        }
        if (!solved) {
            lambda *= 4.0;
            if (lambda > 1e4) break;
            continue;
        }

        // 후보 값. 한계 안으로 잘라낸다.
        amr::Array<double> xn = x;
        for (size_t k = 0; k < n; k++) {
            double v = x[k] + dx[k];
            if (vars[k].limited) {
                if (v < vars[k].lower) v = vars[k].lower;
                if (v > vars[k].upper) v = vars[k].upper;
            }
            xn[k] = v;
        }
        urdf::JointValues qn = q;
        for (size_t k = 0; k < n; k++) SetJointValue(&qn, robot, vars[k].joint, xn[k]);
        const Kinematics kn = Forward(robot, qn, opt.respectLimits);
        const TaskError en = ComputeError(kn, L, tp, tR);
        const double cn = ErrorNorm(en, opt);
        if (cn < cost) {  // 줄었으면 받아들이고 감쇠를 줄인다
            x = xn; q = qn; kin = kn; err = en; cost = cn;
            lambda = Max(lambda * 0.5, 1e-4);
        } else {          // 늘었으면 버리고 감쇠를 키운다 (더 조심스럽게)
            lambda *= 4.0;
            if (lambda > 1e4) break;
        }
    }

    res.iterations = iter;
    res.q = q;
    res.positionError = Sqrt(Dot(err.pos, err.pos));
    res.orientationError = opt.useOrientation ? Sqrt(Dot(err.rot, err.rot)) : 0.0;
    res.converged = res.positionError <= opt.positionTolerance && (!opt.useOrientation || res.orientationError <= opt.orientationTolerance);
    return res;
}

}  // namespace

IkResult SolveIk(const urdf::Robot& robot, const urdf::JointValues& start, int link, const dhMat& target, const IkOptions& opt) {
    IkResult best = SolveOnce(robot, start, link, target, opt, NULL);
    unsigned state = 12345u;
    for (int r = 0; r < opt.restarts && !best.converged; r++) {
        const IkResult t = SolveOnce(robot, start, link, target, opt, &state);
        if (t.converged || t.positionError + t.orientationError < best.positionError + best.orientationError) best = t;
    }
    return best;
}

}  // namespace kinematics
}  // namespace amr
