#include "CDynamics/RigidFall.h"

#include "CBase/Math.h"

namespace simul {
namespace dynamics {
namespace {

inline void Cross(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline double Dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// o = R v (R 행 우선)
inline void Rot(const double r[9], const double v[3], double o[3]) {
    o[0] = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
    o[1] = r[3] * v[0] + r[4] * v[1] + r[5] * v[2];
    o[2] = r[6] * v[0] + r[7] * v[1] + r[8] * v[2];
}

// 월드 관성 역행렬 * v :  R diag(1/I) R^T v
inline void InvInertiaTimes(const double r[9], const double inertia[3], const double v[3], double o[3]) {
    double b[3] = {r[0] * v[0] + r[3] * v[1] + r[6] * v[2], r[1] * v[0] + r[4] * v[1] + r[7] * v[2], r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
    b[0] /= inertia[0];
    b[1] /= inertia[1];
    b[2] /= inertia[2];
    Rot(r, b, o);
}

// 월드 관성 * v :  R diag(I) R^T v
inline void InertiaTimes(const double r[9], const double inertia[3], const double v[3], double o[3]) {
    double b[3] = {r[0] * v[0] + r[3] * v[1] + r[6] * v[2], r[1] * v[0] + r[4] * v[1] + r[7] * v[2], r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
    b[0] *= inertia[0];
    b[1] *= inertia[1];
    b[2] *= inertia[2];
    Rot(r, b, o);
}

// R 을 각속도 w 로 h 초 돌린다 (로드리게스) 그리고 직교화한다
void Integrate(double r[9], const double w[3], double h) {
    const double wn = Sqrt(Dot(w, w));
    const double ang = wn * h;
    if (ang < 1e-12) return;
    const double k[3] = {w[0] / wn, w[1] / wn, w[2] / wn};
    const double c = Cos(ang), s = Sin(ang), t = 1.0 - c;
    const double d[9] = {t * k[0] * k[0] + c,        t * k[0] * k[1] - s * k[2], t * k[0] * k[2] + s * k[1],
                         t * k[0] * k[1] + s * k[2], t * k[1] * k[1] + c,        t * k[1] * k[2] - s * k[0],
                         t * k[0] * k[2] - s * k[1], t * k[1] * k[2] + s * k[0], t * k[2] * k[2] + c};
    double n[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) n[i * 3 + j] = d[i * 3] * r[j] + d[i * 3 + 1] * r[3 + j] + d[i * 3 + 2] * r[6 + j];
    // 직교화 (열 기준 그람-슈미트)
    double c0[3] = {n[0], n[3], n[6]}, c1[3] = {n[1], n[4], n[7]}, c2[3];
    double l = Sqrt(Dot(c0, c0));
    c0[0] /= l; c0[1] /= l; c0[2] /= l;
    const double p = Dot(c0, c1);
    c1[0] -= p * c0[0]; c1[1] -= p * c0[1]; c1[2] -= p * c0[2];
    l = Sqrt(Dot(c1, c1));
    c1[0] /= l; c1[1] /= l; c1[2] /= l;
    Cross(c0, c1, c2);
    r[0] = c0[0]; r[3] = c0[1]; r[6] = c0[2];
    r[1] = c1[0]; r[4] = c1[1]; r[7] = c1[2];
    r[2] = c2[0]; r[5] = c2[1]; r[8] = c2[2];
}

}  // namespace

RigidFall::RigidFall() : planeZ(0.0), restitution(0.3), friction(0.6), restSpeed(0.4), angularDamping(0.05), asleep(false), stillSteps_(0) {
    for (int i = 0; i < 3; i++) {
        com[i] = 0.0;
        inertia[i] = 1.0 / 6.0;
        pos[i] = vel[i] = omega[i] = 0.0;
    }
    for (int i = 0; i < 9; i++) rot[i] = (i % 4 == 0) ? 1.0 : 0.0;
}

void RigidFall::SetShape(const simul::Array<double>& bodyPoints) {
    points = bodyPoints;
    if (points.size() < 3) return;
    double lo[3] = {kDoubleMax, kDoubleMax, kDoubleMax}, hi[3] = {-kDoubleMax, -kDoubleMax, -kDoubleMax};
    for (size_t i = 0; i + 2 < points.size(); i += 3)
        for (int a = 0; a < 3; a++) {
            if (points[i + static_cast<size_t>(a)] < lo[a]) lo[a] = points[i + static_cast<size_t>(a)];
            if (points[i + static_cast<size_t>(a)] > hi[a]) hi[a] = points[i + static_cast<size_t>(a)];
        }
    double e[3];
    for (int a = 0; a < 3; a++) {
        com[a] = 0.5 * (lo[a] + hi[a]);
        e[a] = hi[a] - lo[a];
        if (e[a] < 1e-4) e[a] = 1e-4;   // 아주 얇아도 관성이 0 이 되지 않게
    }
    inertia[0] = (e[1] * e[1] + e[2] * e[2]) / 12.0;
    inertia[1] = (e[0] * e[0] + e[2] * e[2]) / 12.0;
    inertia[2] = (e[0] * e[0] + e[1] * e[1]) / 12.0;
}

void RigidFall::SetPose(const double position[3], const double rotation[9]) {
    for (int i = 0; i < 3; i++) {
        pos[i] = position[i];
        vel[i] = omega[i] = 0.0;
    }
    for (int i = 0; i < 9; i++) rot[i] = rotation[i];
    asleep = false;
    stillSteps_ = 0;
}

void RigidFall::SetVelocity(const double linear[3], const double angular[3]) {
    for (int i = 0; i < 3; i++) {
        vel[i] = linear[i];
        omega[i] = angular[i];
    }
    asleep = false;
    stillSteps_ = 0;
}

void RigidFall::WorldCom(double out[3]) const {
    double c[3];
    Rot(rot, com, c);
    for (int i = 0; i < 3; i++) out[i] = pos[i] + c[i];
}

double RigidFall::LowestPointZ() const {
    double lo = kDoubleMax;
    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        const double z = pos[2] + rot[6] * points[i] + rot[7] * points[i + 1] + rot[8] * points[i + 2];
        if (z < lo) lo = z;
    }
    return lo == kDoubleMax ? pos[2] : lo;
}

int RigidFall::ContactCount(double tol) const {
    int n = 0;
    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        const double z = pos[2] + rot[6] * points[i] + rot[7] * points[i + 1] + rot[8] * points[i + 2];
        if (z <= planeZ + tol) n++;
    }
    return n;
}

double RigidFall::KineticEnergy() const {
    double iw[3];
    InertiaTimes(rot, inertia, omega, iw);
    return 0.5 * Dot(vel, vel) + 0.5 * Dot(omega, iw);
}

bool RigidFall::Step(double dt, double gz) {
    if (!(dt > 0.0) || !(gz < 0.0) || !IsFinite(pos[0]) || !IsFinite(pos[1]) || !IsFinite(pos[2]) || !IsFinite(vel[2]) || !IsFinite(omega[0]) || points.size() < 3) {
        for (int i = 0; i < 3; i++) vel[i] = omega[i] = 0.0;
        return false;
    }
    if (asleep) return false;
    if (dt > 0.1) dt = 0.1;
    int sub = static_cast<int>(Ceil(dt * 480.0));
    if (sub < 1) sub = 1;
    const double h = dt / sub;
    const size_t np = points.size() / 3;
    const double kContactTol = 2e-4;   // 이 안이면 바닥에 닿은 것으로 본다 (m)

    for (int s = 0; s < sub; s++) {
        // 1) 중력과 자이로 효과 (자유 회전: I w' = -w x I w), 공기 저항
        vel[2] += gz * h;
        double Iw[3], tau[3], dw[3];
        InertiaTimes(rot, inertia, omega, Iw);
        Cross(omega, Iw, tau);
        tau[0] = -tau[0]; tau[1] = -tau[1]; tau[2] = -tau[2];
        InvInertiaTimes(rot, inertia, tau, dw);
        const double damp = 1.0 / (1.0 + angularDamping * h);
        for (int i = 0; i < 3; i++) omega[i] = (omega[i] + dw[i] * h) * damp;

        // 2) 바닥에 닿은(또는 들어간) 점을 모은다
        double comW[3];
        WorldCom(comW);
        struct Contact { double r[3]; double depth; double bounce; double jn; double jt[2]; double mN; double mT[2]; };
        Contact cs[80];
        int nc = 0;
        for (size_t p = 0; p < np && nc < 80; p++) {
            const double bp[3] = {points[p * 3], points[p * 3 + 1], points[p * 3 + 2]};
            double w[3];
            Rot(rot, bp, w);
            const double z = pos[2] + w[2];
            if (z > planeZ + kContactTol + 0.05) continue;     // 멀리 있는 점은 볼 필요 없다
            double r[3] = {pos[0] + w[0] - comW[0], pos[1] + w[1] - comW[1], z - comW[2]};
            double wv[3], vp[3];
            Cross(omega, r, wv);
            for (int i = 0; i < 3; i++) vp[i] = vel[i] + wv[i];
            // 이번 스텝에 바닥을 지나칠 점도 미리 막는다 (빠르게 떨어져도 뚫고 들어가 속도를 잃지 않게)
            if (z + (vp[2] < 0.0 ? vp[2] * h : 0.0) > planeZ + kContactTol) continue;
            Contact& c = cs[nc++];
            c.r[0] = r[0]; c.r[1] = r[1]; c.r[2] = r[2];
            c.depth = planeZ - z;
            c.bounce = (vp[2] < -restSpeed) ? -restitution * vp[2] : 0.0;   // 닿을 때의 속도로 정한 되튐 속도
            c.jn = c.jt[0] = c.jt[1] = 0.0;
            // 접촉 방향마다 유효 질량 (단위 질량): 1 / (1 + (r x n) . I^-1 (r x n))
            const double dirs[3][3] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
            double m[3];
            for (int d = 0; d < 3; d++) {
                double rn[3], irn[3];
                Cross(c.r, dirs[d], rn);
                InvInertiaTimes(rot, inertia, rn, irn);
                m[d] = 1.0 / (1.0 + Dot(rn, irn));
            }
            c.mN = m[0];
            c.mT[0] = m[1];
            c.mT[1] = m[2];
        }
        // 3) 충격량을 순차적으로 푼다 (누적 충격량으로 0 이상/마찰 원뿔을 지킨다)
        for (int it = 0; it < 12 && nc > 0; it++) {
            for (int k = 0; k < nc; k++) {
                Contact& c = cs[k];
                double wv[3], vp[3];
                Cross(omega, c.r, wv);
                for (int i = 0; i < 3; i++) vp[i] = vel[i] + wv[i];
                // 법선
                double dj = c.mN * (c.bounce - vp[2]);
                double nj = c.jn + dj;
                if (nj < 0.0) nj = 0.0;
                dj = nj - c.jn;
                c.jn = nj;
                {
                    const double imp[3] = {0, 0, dj};
                    double rxj[3], dwv[3];
                    Cross(c.r, imp, rxj);
                    InvInertiaTimes(rot, inertia, rxj, dwv);
                    vel[2] += dj;
                    for (int i = 0; i < 3; i++) omega[i] += dwv[i];
                }
                // 마찰 (x, y 두 방향, 각각 mu * jn 이내)
                for (int a = 0; a < 2; a++) {
                    Cross(omega, c.r, wv);
                    const double vt = vel[a] + wv[a];
                    double jt = c.jt[a] - c.mT[a] * vt;
                    const double lim = friction * c.jn;
                    if (jt > lim) jt = lim;
                    if (jt < -lim) jt = -lim;
                    const double d = jt - c.jt[a];
                    c.jt[a] = jt;
                    double imp[3] = {0, 0, 0};
                    imp[a] = d;
                    double rxj[3], dwv[3];
                    Cross(c.r, imp, rxj);
                    InvInertiaTimes(rot, inertia, rxj, dwv);
                    vel[a] += d;
                    for (int i = 0; i < 3; i++) omega[i] += dwv[i];
                }
            }
        }
        // 4) 이동과 회전 (질량 중심 기준), 바닥 아래로 들어간 만큼 위로 민다
        double comNew[3] = {comW[0] + vel[0] * h, comW[1] + vel[1] * h, comW[2] + vel[2] * h};
        Integrate(rot, omega, h);
        double c2[3];
        Rot(rot, com, c2);
        for (int i = 0; i < 3; i++) pos[i] = comNew[i] - c2[i];
        const double low = LowestPointZ();
        if (low < planeZ) {
            pos[2] += planeZ - low;
            if (vel[2] < 0.0) vel[2] = 0.0;
        }
        // 5) 가만히 서 있는가: 점 셋 이상이 바닥에 닿고 속도가 아주 작은 채로 잠깐 지속되면 잠든다
        const double speed2 = Dot(vel, vel), spin2 = Dot(omega, omega);
        if (speed2 < 4e-4 && spin2 < 2.5e-3 && ContactCount(1e-3) >= 3) {
            if (++stillSteps_ >= 48) {
                for (int i = 0; i < 3; i++) vel[i] = omega[i] = 0.0;
                asleep = true;
                return false;
            }
        } else {
            stillSteps_ = 0;
        }
    }
    return true;
}

void SelectContactPoints(const double* xyz, size_t count, simul::Array<double>* out) {
    out->clear();
    if (count == 0 || xyz == nullptr) return;
    // 방향: 상자의 26 방향 + 피보나치 구 50 개
    simul::Array<double> dirs;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++) {
                if (x == 0 && y == 0 && z == 0) continue;
                const double l = Sqrt(static_cast<double>(x * x + y * y + z * z));
                dirs.push_back(x / l);
                dirs.push_back(y / l);
                dirs.push_back(z / l);
            }
    const int nf = 50;
    const double golden = 3.14159265358979323846 * (3.0 - Sqrt(5.0));
    for (int i = 0; i < nf; i++) {
        const double z = 1.0 - 2.0 * (i + 0.5) / nf;
        const double r = Sqrt(1.0 - z * z);
        dirs.push_back(r * Cos(golden * i));
        dirs.push_back(r * Sin(golden * i));
        dirs.push_back(z);
    }
    const size_t nd = dirs.size() / 3;
    simul::Array<double> best(nd, -kDoubleMax);
    simul::Array<size_t> bi(nd, 0);
    for (size_t p = 0; p < count; p++) {
        const double x = xyz[p * 3], y = xyz[p * 3 + 1], z = xyz[p * 3 + 2];
        for (size_t d = 0; d < nd; d++) {
            const double v = x * dirs[d * 3] + y * dirs[d * 3 + 1] + z * dirs[d * 3 + 2];
            if (v > best[d]) {
                best[d] = v;
                bi[d] = p;
            }
        }
    }
    for (size_t d = 0; d < nd; d++) {
        bool dup = false;
        for (size_t k = 0; k < d && !dup; k++) dup = bi[k] == bi[d];
        if (dup) continue;
        out->push_back(xyz[bi[d] * 3]);
        out->push_back(xyz[bi[d] * 3 + 1]);
        out->push_back(xyz[bi[d] * 3 + 2]);
    }
}

}  // namespace dynamics
}  // namespace simul
