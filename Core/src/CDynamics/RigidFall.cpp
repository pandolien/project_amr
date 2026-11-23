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

RigidFall::RigidFall()
    : planeZ(0.0), restitution(0.3), friction(0.6), restSpeed(0.4), angularDamping(0.05), asleep(false), fixed(false), collide(true), moved(false), boundRadius(0.0), stillSteps_(0) {
    for (int i = 0; i < 3; i++) {
        com[i] = 0.0;
        inertia[i] = 1.0 / 6.0;
        pos[i] = vel[i] = omega[i] = 0.0;
        boundCenter[i] = 0.0;
    }
    for (int i = 0; i < 9; i++) rot[i] = (i % 4 == 0) ? 1.0 : 0.0;
}

// 점들의 볼록 껍질 면: 세 점이 이루는 평면 중 나머지 점이 모두 한쪽에 있는 것 (점이 76 개 이하라 전수 검사로 충분하다)
static void BuildHull(const simul::Array<double>& pts, simul::Array<double>* planes) {
    planes->clear();
    const size_t n = pts.size() / 3;
    if (n < 3) return;
    double scale = 0.0;
    for (size_t i = 0; i < pts.size(); i++) scale = scale < Abs(pts[i]) ? Abs(pts[i]) : scale;
    const double eps = 1e-9 + 1e-7 * scale;
    for (size_t i = 0; i < n; i++)
        for (size_t j = i + 1; j < n; j++)
            for (size_t k = j + 1; k < n; k++) {
                const double* A = &pts[i * 3];
                const double* B = &pts[j * 3];
                const double* C = &pts[k * 3];
                const double u[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]}, w[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
                double nn[3];
                Cross(u, w, nn);
                const double len = Sqrt(Dot(nn, nn));
                if (len < 1e-12 + 1e-9 * scale * scale) continue;
                nn[0] /= len; nn[1] /= len; nn[2] /= len;
                const double d = Dot(nn, A);
                double hi = -kDoubleMax, lo = kDoubleMax;
                for (size_t m = 0; m < n; m++) {
                    const double v = Dot(nn, &pts[m * 3]) - d;
                    if (v > hi) hi = v;
                    if (v < lo) lo = v;
                }
                double sign = 0.0;
                if (hi <= eps) sign = 1.0;            // 모두 안쪽 (n 이 바깥을 향한다)
                else if (lo >= -eps) sign = -1.0;     // 모두 반대쪽: 뒤집는다
                else continue;
                const double fn[3] = {nn[0] * sign, nn[1] * sign, nn[2] * sign};
                const double fd = d * sign;
                bool dup = false;
                for (size_t q = 0; q + 3 < planes->size() && !dup; q += 4)
                    dup = Dot(fn, &(*planes)[q]) > 1.0 - 1e-9 && Abs(fd - (*planes)[q + 3]) < 1e-7 + 1e-7 * scale;
                if (dup) continue;
                planes->push_back(fn[0]);
                planes->push_back(fn[1]);
                planes->push_back(fn[2]);
                planes->push_back(fd);
            }
}

void RigidFall::SetShape(const simul::Array<double>& bodyPoints) {
    points = bodyPoints;
    planes.clear();
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
    for (int a = 0; a < 3; a++) boundCenter[a] = com[a];
    double r2 = 0.0;
    for (size_t i = 0; i + 2 < points.size(); i += 3) {
        const double d[3] = {points[i] - com[0], points[i + 1] - com[1], points[i + 2] - com[2]};
        const double l = Dot(d, d);
        if (l > r2) r2 = l;
    }
    boundRadius = Sqrt(r2);
    BuildHull(points, &planes);
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
    RigidFall* self = this;
    return StepRigidBodies(&self, 1, dt, gz);
}

namespace {

struct Contact {
    int a, b;            // 몸체 번호. b = -1 이면 바닥
    double ra[3], rb[3]; // 접촉점에서 각 몸체 질량 중심까지의 벡터 (월드)
    double n[3];         // 법선: b 에서 a 쪽 (a 를 밀어내는 방향)
    double t1[3], t2[3]; // 마찰 방향
    double gap;          // 떨어진 거리 (m, 겹치면 음수)
    double vmin;         // 이 접촉이 허용하는 법선 속도의 하한 (되튐, 겹침 해소)
    double mu, e;
    double mn, m1, m2;   // 방향별 유효 질량
    double jn, j1, j2;   // 누적 충격량
};

// 몸체 b 가 질량이 있는 움직이는 몸체인가 (잠들었거나 고정이면 아니다)
inline bool Dynamic(const RigidFall* r) { return !r->asleep && !r->fixed; }

// 방향 d 로 충격량 1 을 접촉점에 줄 때 몸체 body 의 유효 역질량 (단위 질량): 1 + (r x d) . I^-1 (r x d)
inline double InvEff(const RigidFall* body, const double r[3], const double d[3]) {
    if (!Dynamic(body)) return 0.0;
    double rd[3], ir[3];
    Cross(r, d, rd);
    InvInertiaTimes(body->rot, body->inertia, rd, ir);
    return 1.0 + Dot(rd, ir);
}

inline void Velocity(const RigidFall* body, const double r[3], double out[3]) {
    double wv[3];
    Cross(body->omega, r, wv);
    for (int i = 0; i < 3; i++) out[i] = body->vel[i] + wv[i];
}

// 충격량 j*d 를 접촉점에 준다: a 는 +, b 는 -
inline void ApplyImpulse(RigidFall* body, const double r[3], const double d[3], double j) {
    if (!Dynamic(body)) return;
    double rxj[3], dw[3];
    const double imp[3] = {d[0] * j, d[1] * j, d[2] * j};
    Cross(r, imp, rxj);
    InvInertiaTimes(body->rot, body->inertia, rxj, dw);
    for (int i = 0; i < 3; i++) {
        body->vel[i] += imp[i];
        body->omega[i] += dw[i];
    }
}

void Tangents(const double n[3], double t1[3], double t2[3]) {
    const double ref[3] = {Abs(n[0]) < 0.9 ? 1.0 : 0.0, Abs(n[0]) < 0.9 ? 0.0 : 1.0, 0.0};
    Cross(n, ref, t1);
    const double l = Sqrt(Dot(t1, t1));
    t1[0] /= l; t1[1] /= l; t1[2] /= l;
    Cross(n, t1, t2);
}

}  // namespace

bool StepRigidBodies(RigidFall* const* bodies, size_t count, double dt, double gz) {
    const size_t kMaxContacts = 600;
    bool any = false;
    for (size_t i = 0; i < count; i++) bodies[i]->moved = false;
    if (!(dt > 0.0) || !(gz < 0.0)) {
        for (size_t i = 0; i < count; i++)
            if (Dynamic(bodies[i])) for (int k = 0; k < 3; k++) bodies[i]->vel[k] = bodies[i]->omega[k] = 0.0;
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        RigidFall* b = bodies[i];
        if (!IsFinite(b->pos[0]) || !IsFinite(b->pos[1]) || !IsFinite(b->pos[2]) || !IsFinite(b->vel[2]) || !IsFinite(b->omega[0]) || b->points.size() < 3) {
            for (int k = 0; k < 3; k++) b->vel[k] = b->omega[k] = 0.0;
            b->asleep = true;
        }
    }
    if (dt > 0.1) dt = 0.1;
    int sub = static_cast<int>(Ceil(dt * 480.0));
    if (sub < 1) sub = 1;
    const double h = dt / sub;
    const double kContactTol = 2e-4;   // 이 안이면 닿은 것으로 본다 (m)
    simul::Array<Contact> cs;
    cs.reserve(kMaxContacts + 8);
    simul::Array<int> touching(count, 0);

    for (int s = 0; s < sub; s++) {
        bool anyAwake = false;
        for (size_t i = 0; i < count; i++) anyAwake = anyAwake || Dynamic(bodies[i]);
        if (!anyAwake) break;
        // 1) 중력, 자이로 효과, 감쇠
        for (size_t i = 0; i < count; i++) {
            RigidFall* b = bodies[i];
            if (!Dynamic(b)) continue;
            b->vel[2] += gz * h;
            double Iw[3], tau[3], dw[3];
            InertiaTimes(b->rot, b->inertia, b->omega, Iw);
            Cross(b->omega, Iw, tau);
            tau[0] = -tau[0]; tau[1] = -tau[1]; tau[2] = -tau[2];
            InvInertiaTimes(b->rot, b->inertia, tau, dw);
            const double damp = 1.0 / (1.0 + b->angularDamping * h);
            for (int k = 0; k < 3; k++) b->omega[k] = (b->omega[k] + dw[k] * h) * damp;
        }
        // 2) 접촉 모으기
        cs.clear();
        for (size_t i = 0; i < count; i++) {
            RigidFall* b = bodies[i];
            if (!Dynamic(b)) continue;
            double comW[3];
            b->WorldCom(comW);
            const size_t np = b->points.size() / 3;
            for (size_t p = 0; p < np && cs.size() < kMaxContacts; p++) {
                const double bp[3] = {b->points[p * 3], b->points[p * 3 + 1], b->points[p * 3 + 2]};
                double w[3];
                Rot(b->rot, bp, w);
                const double z = b->pos[2] + w[2];
                if (z > b->planeZ + 0.05 + kContactTol) continue;
                Contact c;
                c.a = static_cast<int>(i);
                c.b = -1;
                c.ra[0] = b->pos[0] + w[0] - comW[0]; c.ra[1] = b->pos[1] + w[1] - comW[1]; c.ra[2] = z - comW[2];
                c.rb[0] = c.rb[1] = c.rb[2] = 0.0;
                c.n[0] = 0.0; c.n[1] = 0.0; c.n[2] = 1.0;
                c.gap = z - b->planeZ;
                c.mu = b->friction;
                c.e = b->restitution;
                cs.push_back(c);
            }
        }
        // 몸체끼리: 경계 구가 겹치는 쌍마다 서로의 점이 상대의 볼록 껍질 안에 들어왔는가
        for (size_t i = 0; i < count; i++) {
            for (size_t j = i + 1; j < count; j++) {
                RigidFall* A = bodies[i];
                RigidFall* B = bodies[j];
                if (!A->collide || !B->collide || A->planes.empty() || B->planes.empty()) continue;
                if (!Dynamic(A) && !Dynamic(B)) continue;
                double ca[3], cb[3];
                A->WorldCom(ca);
                B->WorldCom(cb);
                double d[3] = {ca[0] - cb[0], ca[1] - cb[1], ca[2] - cb[2]};
                const double speed = Sqrt(Dot(A->vel, A->vel)) + Sqrt(Dot(B->vel, B->vel)) + Sqrt(Dot(A->omega, A->omega)) * A->boundRadius + Sqrt(Dot(B->omega, B->omega)) * B->boundRadius;
                const double reach = A->boundRadius + B->boundRadius + 0.02 + speed * h;
                if (Dot(d, d) > reach * reach) continue;
                for (int pass = 0; pass < 2; pass++) {
                    RigidFall* P = pass == 0 ? A : B;     // 점을 내는 몸체
                    RigidFall* Q = pass == 0 ? B : A;     // 껍질을 내는 몸체
                    const int pi = pass == 0 ? static_cast<int>(i) : static_cast<int>(j);
                    const int qi = pass == 0 ? static_cast<int>(j) : static_cast<int>(i);
                    // Q 의 면을 월드로
                    const size_t nq = Q->planes.size() / 4;
                    double nw[160][3];
                    double dw[160];
                    const size_t qn = nq < 160 ? nq : 160;
                    for (size_t f = 0; f < qn; f++) {
                        const double nb[3] = {Q->planes[f * 4], Q->planes[f * 4 + 1], Q->planes[f * 4 + 2]};
                        Rot(Q->rot, nb, nw[f]);
                        dw[f] = Q->planes[f * 4 + 3] + Dot(nw[f], Q->pos);
                    }
                    double pc[3], qc[3];
                    P->WorldCom(pc);
                    Q->WorldCom(qc);
                    const double relv = Sqrt((P->vel[0] - Q->vel[0]) * (P->vel[0] - Q->vel[0]) + (P->vel[1] - Q->vel[1]) * (P->vel[1] - Q->vel[1]) + (P->vel[2] - Q->vel[2]) * (P->vel[2] - Q->vel[2])) +
                                       (Sqrt(Dot(P->omega, P->omega)) * P->boundRadius + Sqrt(Dot(Q->omega, Q->omega)) * Q->boundRadius);
                    const double margin = kContactTol + relv * h;
                    const size_t npnt = P->points.size() / 3;
                    // P 의 점들을 월드로
                    double xw[80][3];
                    const size_t pn = npnt < 80 ? npnt : 80;
                    for (size_t k = 0; k < pn; k++) {
                        const double bp[3] = {P->points[k * 3], P->points[k * 3 + 1], P->points[k * 3 + 2]};
                        double w[3];
                        Rot(P->rot, bp, w);
                        xw[k][0] = P->pos[0] + w[0]; xw[k][1] = P->pos[1] + w[1]; xw[k][2] = P->pos[2] + w[2];
                    }
                    // 분리축: Q 의 면 중 P 가 가장 멀리 떨어진(겹침이 가장 얕은) 면. 그 면 쪽으로 가장 깊이 들어온 점도 margin 보다 멀면 닿지 않았다.
                    size_t bf = 0;
                    double bestSep = -kDoubleMax;
                    for (size_t f = 0; f < qn; f++) {
                        double lo = kDoubleMax;
                        for (size_t k = 0; k < pn; k++) {
                            const double sd = Dot(nw[f], xw[k]) - dw[f];
                            if (sd < lo) lo = sd;
                        }
                        if (lo > bestSep) { bestSep = lo; bf = f; }
                    }
                    if (bestSep > margin) continue;
                    // 기준 면 bf 에 맞서는 P 의 면(입사 면)을 찾아 그 다각형을 Q 의 옆면들로 잘라 낸다: 남은 꼭짓점이 접촉점이다
                    // (겹친 영역의 모서리 교차점까지 잡히므로 상자 위에 상자를 올려도 면 전체로 받친다)
                    double poly[200][3];
                    size_t pcount = 0;
                    {
                        const size_t np2 = P->planes.size() / 4;
                        size_t bi = 0;
                        double bd = kDoubleMax;
                        for (size_t f = 0; f < np2 && f < 160; f++) {
                            const double nb[3] = {P->planes[f * 4], P->planes[f * 4 + 1], P->planes[f * 4 + 2]};
                            double nwp[3];
                            Rot(P->rot, nb, nwp);
                            const double dd = Dot(nwp, nw[bf]);
                            if (dd < bd) { bd = dd; bi = f; }
                        }
                        const double nb[3] = {P->planes[bi * 4], P->planes[bi * 4 + 1], P->planes[bi * 4 + 2]};
                        double nwp[3];
                        Rot(P->rot, nb, nwp);
                        const double dp = P->planes[bi * 4 + 3] + Dot(nwp, P->pos);
                        // 그 면 위의 점들
                        for (size_t k = 0; k < pn && pcount < 100; k++) {
                            if (Abs(Dot(nwp, xw[k]) - dp) <= 1e-4 + 1e-6 * P->boundRadius) {
                                poly[pcount][0] = xw[k][0]; poly[pcount][1] = xw[k][1]; poly[pcount][2] = xw[k][2];
                                pcount++;
                            }
                        }
                        if (pcount >= 3) {   // 각도 순으로 정렬 (볼록 다각형)
                            double cen[3] = {0, 0, 0};
                            for (size_t k = 0; k < pcount; k++) { cen[0] += poly[k][0]; cen[1] += poly[k][1]; cen[2] += poly[k][2]; }
                            cen[0] /= pcount; cen[1] /= pcount; cen[2] /= pcount;
                            double u[3], v[3];
                            Tangents(nwp, u, v);
                            double ang[100];
                            for (size_t k = 0; k < pcount; k++) {
                                const double d[3] = {poly[k][0] - cen[0], poly[k][1] - cen[1], poly[k][2] - cen[2]};
                                ang[k] = Atan2(Dot(d, v), Dot(d, u));
                            }
                            for (size_t k = 1; k < pcount; k++) {
                                const double a0 = ang[k];
                                double q0[3] = {poly[k][0], poly[k][1], poly[k][2]};
                                size_t m = k;
                                while (m > 0 && ang[m - 1] > a0) { ang[m] = ang[m - 1]; poly[m][0] = poly[m - 1][0]; poly[m][1] = poly[m - 1][1]; poly[m][2] = poly[m - 1][2]; m--; }
                                ang[m] = a0; poly[m][0] = q0[0]; poly[m][1] = q0[1]; poly[m][2] = q0[2];
                            }
                            // Q 의 다른 면들(옆면)로 자른다 (Sutherland-Hodgman)
                            for (size_t g = 0; g < qn && pcount > 0; g++) {
                                if (g == bf) continue;
                                double out[200][3];
                                size_t oc = 0;
                                for (size_t k = 0; k < pcount; k++) {
                                    const double* cur = poly[k];
                                    const double* nxt = poly[(k + 1) % pcount];
                                    const double sc = Dot(nw[g], cur) - dw[g], sn = Dot(nw[g], nxt) - dw[g];
                                    if (sc <= 0.0) { if (oc < 199) { out[oc][0] = cur[0]; out[oc][1] = cur[1]; out[oc][2] = cur[2]; oc++; } }
                                    if ((sc <= 0.0) != (sn <= 0.0) && oc < 199) {
                                        const double t = sc / (sc - sn);
                                        out[oc][0] = cur[0] + t * (nxt[0] - cur[0]);
                                        out[oc][1] = cur[1] + t * (nxt[1] - cur[1]);
                                        out[oc][2] = cur[2] + t * (nxt[2] - cur[2]);
                                        oc++;
                                    }
                                }
                                pcount = oc;
                                for (size_t k = 0; k < oc; k++) { poly[k][0] = out[k][0]; poly[k][1] = out[k][1]; poly[k][2] = out[k][2]; }
                            }
                        } else {   // 점이나 모서리로 닿는다: 그 점들 중 Q 의 면 안쪽에 있는 것만
                            size_t keep = 0;
                            for (size_t k = 0; k < pcount; k++) {
                                bool inside = true;
                                for (size_t g = 0; g < qn && inside; g++) {
                                    if (g == bf) continue;
                                    inside = Dot(nw[g], poly[k]) - dw[g] <= 1e-6;
                                }
                                if (inside) { poly[keep][0] = poly[k][0]; poly[keep][1] = poly[k][1]; poly[keep][2] = poly[k][2]; keep++; }
                            }
                            pcount = keep;
                        }
                    }
                    for (size_t k = 0; k < pcount && cs.size() < kMaxContacts; k++) {
                        const double sf = Dot(nw[bf], poly[k]) - dw[bf];
                        if (sf > margin) continue;
                        Contact c;
                        c.a = pi;
                        c.b = qi;
                        c.ra[0] = poly[k][0] - pc[0]; c.ra[1] = poly[k][1] - pc[1]; c.ra[2] = poly[k][2] - pc[2];
                        c.rb[0] = poly[k][0] - qc[0]; c.rb[1] = poly[k][1] - qc[1]; c.rb[2] = poly[k][2] - qc[2];
                        c.n[0] = nw[bf][0]; c.n[1] = nw[bf][1]; c.n[2] = nw[bf][2];
                        c.gap = sf;                          // 양수면 떨어져 있고 음수면 겹쳐 있다
                        c.mu = P->friction < Q->friction ? P->friction : Q->friction;
                        c.e = 0.5 * (P->restitution + Q->restitution);
                        cs.push_back(c);
                    }
                }
            }
        }
        // 잠든 몸체에 다른 몸체가 세게(0.2 m/s 이상) 다가와 닿으면 먼저 깨운다 (그래야 벽처럼 튕겨 내지 않고 같이 밀려난다)
        for (size_t k = 0; k < cs.size(); k++) {
            const Contact& c = cs[k];
            if (c.b < 0) continue;
            RigidFall* A = bodies[c.a];
            RigidFall* B = bodies[c.b];
            if (!((A->asleep && !A->fixed) || (B->asleep && !B->fixed))) continue;
            double va[3], vb[3];
            Velocity(A, c.ra, va);
            Velocity(B, c.rb, vb);
            const double approach = -((va[0] - vb[0]) * c.n[0] + (va[1] - vb[1]) * c.n[1] + (va[2] - vb[2]) * c.n[2]);
            if (approach > 0.2 && c.gap <= approach * h + kContactTol) {
                if (A->asleep && !A->fixed) { A->asleep = false; A->stillSteps_ = 0; }
                if (B->asleep && !B->fixed) { B->asleep = false; B->stillSteps_ = 0; }
            }
        }
        // 3) 접촉마다 유효 질량과 하한 속도
        for (size_t k = 0; k < cs.size(); k++) {
            Contact& c = cs[k];
            RigidFall* A = bodies[c.a];
            RigidFall* B = c.b >= 0 ? bodies[c.b] : nullptr;
            Tangents(c.n, c.t1, c.t2);
            const double* dirs[3] = {c.n, c.t1, c.t2};
            double m[3];
            for (int d = 0; d < 3; d++) {
                double inv = InvEff(A, c.ra, dirs[d]);
                if (B) inv += InvEff(B, c.rb, dirs[d]);
                m[d] = inv > 0.0 ? 1.0 / inv : 0.0;
            }
            c.mn = m[0]; c.m1 = m[1]; c.m2 = m[2];
            double va[3], vb[3] = {0, 0, 0};
            Velocity(A, c.ra, va);
            if (B) Velocity(B, c.rb, vb);
            const double vn = (va[0] - vb[0]) * c.n[0] + (va[1] - vb[1]) * c.n[1] + (va[2] - vb[2]) * c.n[2];
            const double approach = -vn;
            const double rs = A->restSpeed;
            if (c.gap > 0.0 && approach * h < c.gap - kContactTol) {
                c.vmin = -(c.gap - kContactTol) / h;     // 아직 닿지 않는다: 이번 스텝에 닿을 만큼까지는 다가가도 된다
            } else {
                const double bounce = approach > rs ? c.e * approach : 0.0;
                double bias = 0.0;
                if (c.gap < -5e-4) { bias = (-c.gap - 5e-4) * 0.2 / h; if (bias > 1.0) bias = 1.0; }   // 겹침은 서서히 풀어 준다
                c.vmin = bounce > bias ? bounce : bias;
            }
            c.jn = c.j1 = c.j2 = 0.0;
        }
        // 4) 충격량을 순차적으로 푼다
        for (int it = 0; it < 14 && !cs.empty(); it++) {
            for (size_t k = 0; k < cs.size(); k++) {
                Contact& c = cs[k];
                RigidFall* A = bodies[c.a];
                RigidFall* B = c.b >= 0 ? bodies[c.b] : nullptr;
                double va[3], vb[3] = {0, 0, 0};
                Velocity(A, c.ra, va);
                if (B) Velocity(B, c.rb, vb);
                const double rel[3] = {va[0] - vb[0], va[1] - vb[1], va[2] - vb[2]};
                double dj = c.mn * (c.vmin - Dot(rel, c.n));
                double nj = c.jn + dj;
                if (nj < 0.0) nj = 0.0;
                dj = nj - c.jn;
                c.jn = nj;
                if (dj != 0.0) {
                    ApplyImpulse(A, c.ra, c.n, dj);
                    if (B) ApplyImpulse(B, c.rb, c.n, -dj);
                }
                const double* tdir[2] = {c.t1, c.t2};
                double* tj[2] = {&c.j1, &c.j2};
                const double tm[2] = {c.m1, c.m2};
                for (int a = 0; a < 2; a++) {
                    Velocity(A, c.ra, va);
                    if (B) Velocity(B, c.rb, vb);
                    const double vt = (va[0] - vb[0]) * tdir[a][0] + (va[1] - vb[1]) * tdir[a][1] + (va[2] - vb[2]) * tdir[a][2];
                    double jt = *tj[a] - tm[a] * vt;
                    const double lim = c.mu * c.jn;
                    if (jt > lim) jt = lim;
                    if (jt < -lim) jt = -lim;
                    const double d = jt - *tj[a];
                    *tj[a] = jt;
                    if (d != 0.0) {
                        ApplyImpulse(A, c.ra, tdir[a], d);
                        if (B) ApplyImpulse(B, c.rb, tdir[a], -d);
                    }
                }
            }
        }
        // 잠든 몸체를 세게 부딪히면 깨운다
        for (size_t k = 0; k < cs.size(); k++) {
            const Contact& c = cs[k];
            if (c.b < 0 || c.jn < 0.2) continue;
            RigidFall* B = bodies[c.b];
            RigidFall* A = bodies[c.a];
            if (B->asleep && !B->fixed) { B->asleep = false; B->stillSteps_ = 0; }
            if (A->asleep && !A->fixed) { A->asleep = false; A->stillSteps_ = 0; }
        }
        // 5) 이동과 회전 (질량 중심 기준), 바닥 아래로 들어간 만큼 위로 민다
        for (size_t i = 0; i < count; i++) touching[i] = 0;
        for (size_t k = 0; k < cs.size(); k++) {
            if (cs[k].gap < 1e-3 && cs[k].jn > 0.0) {
                touching[static_cast<size_t>(cs[k].a)]++;
                if (cs[k].b >= 0) touching[static_cast<size_t>(cs[k].b)]++;
            }
        }
        for (size_t i = 0; i < count; i++) {
            RigidFall* b = bodies[i];
            if (!Dynamic(b)) continue;
            double comW[3];
            b->WorldCom(comW);
            const double comNew[3] = {comW[0] + b->vel[0] * h, comW[1] + b->vel[1] * h, comW[2] + b->vel[2] * h};
            Integrate(b->rot, b->omega, h);
            double c2[3];
            Rot(b->rot, b->com, c2);
            for (int k = 0; k < 3; k++) b->pos[k] = comNew[k] - c2[k];
            const double low = b->LowestPointZ();
            if (low < b->planeZ) {
                b->pos[2] += b->planeZ - low;
                if (b->vel[2] < 0.0) b->vel[2] = 0.0;
            }
            b->moved = true;
            // 가만히 서 있는가: 접촉이 셋 이상이고 속도가 아주 작은 채로 잠깐 지속되면 잠든다
            const double speed2 = Dot(b->vel, b->vel), spin2 = Dot(b->omega, b->omega);
            if (speed2 < 4e-4 && spin2 < 2.5e-3 && touching[i] >= 3) {
                if (++b->stillSteps_ >= 48) {
                    for (int k = 0; k < 3; k++) b->vel[k] = b->omega[k] = 0.0;
                    b->asleep = true;
                }
            } else {
                b->stillSteps_ = 0;
            }
        }
    }
    for (size_t i = 0; i < count; i++) any = any || Dynamic(bodies[i]);
    return any;
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
