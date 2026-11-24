#include "CSimul/AStar.h"

#include <math.h>

namespace simul {
namespace sim {

double SegmentDistance(const Point2& a, const Point2& b, const Point2& p) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (l2 > 0.0) {
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    return hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool SegmentClear(const Point2& a, const Point2& b, const Array<Circle2>& obstacles) {
    for (size_t i = 0; i < obstacles.size(); i++)
        if (SegmentDistance(a, b, Point2(obstacles[i].x, obstacles[i].y)) < obstacles[i].r - 1e-9) return false;
    return true;
}

namespace {

struct HeapItem {
    double f;
    int idx;
    HeapItem() : f(0), idx(0) {}
    HeapItem(double pf, int pi) : f(pf), idx(pi) {}
};

// 작은 f 가 위로 가는 이진 힙 (같은 칸이 여러 번 들어갈 수 있다: 꺼낼 때 이미 처리한 칸이면 버린다)
void HeapPush(Array<HeapItem>* h, const HeapItem& it) {
    h->push_back(it);
    size_t i = h->size() - 1;
    while (i > 0) {
        const size_t p = (i - 1) / 2;
        if ((*h)[p].f <= (*h)[i].f) break;
        const HeapItem t = (*h)[p];
        (*h)[p] = (*h)[i];
        (*h)[i] = t;
        i = p;
    }
}

HeapItem HeapPop(Array<HeapItem>* h) {
    const HeapItem top = (*h)[0];
    (*h)[0] = (*h)[h->size() - 1];
    h->pop_back();
    size_t i = 0;
    const size_t n = h->size();
    for (;;) {
        size_t l = 2 * i + 1, r = l + 1, m = i;
        if (l < n && (*h)[l].f < (*h)[m].f) m = l;
        if (r < n && (*h)[r].f < (*h)[m].f) m = r;
        if (m == i) break;
        const HeapItem t = (*h)[m];
        (*h)[m] = (*h)[i];
        (*h)[i] = t;
        i = m;
    }
    return top;
}

int ClampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

bool FindPath2D(const Point2& start, const Point2& goal, const Array<Circle2>& obstacles, double cell, double margin, double rightBias,
                Array<Point2>* out) {
    out->clear();
    if (!(cell > 0.0) || !(margin >= 0.0) || !(rightBias >= 0.0)) return false;
    if (!(fabs(start.x) < 1e9 && fabs(start.y) < 1e9 && fabs(goal.x) < 1e9 && fabs(goal.y) < 1e9)) return false;
    if (hypot(goal.x - start.x, goal.y - start.y) < 1e-9) {
        out->push_back(goal);
        return true;
    }

    const double minx = (start.x < goal.x ? start.x : goal.x) - margin, maxx = (start.x > goal.x ? start.x : goal.x) + margin;
    const double miny = (start.y < goal.y ? start.y : goal.y) - margin, maxy = (start.y > goal.y ? start.y : goal.y) + margin;
    const double fnx = ceil((maxx - minx) / cell) + 1.0, fny = ceil((maxy - miny) / cell) + 1.0;
    if (fnx * fny > 40000.0) return false;
    const int nx = static_cast<int>(fnx), ny = static_cast<int>(fny);
    const int n = nx * ny;

    // 칸마다 비용 배율: 1 이 보통, 장애물 안쪽은 10 + 깊이에 따라 최대 50 까지
    Array<double> mult(static_cast<size_t>(n), 1.0);
    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            const double cx = minx + i * cell, cy = miny + j * cell;
            double m = 1.0;
            for (size_t k = 0; k < obstacles.size(); k++) {
                const Circle2& o = obstacles[k];
                const double d = hypot(cx - o.x, cy - o.y);
                if (d < o.r && o.r > 0.0) m += 10.0 + 40.0 * (o.r - d) / o.r;
            }
            mult[static_cast<size_t>(j * nx + i)] = m;
        }
    }

    const int si = ClampInt(static_cast<int>(floor((start.x - minx) / cell + 0.5)), 0, nx - 1);
    const int sj = ClampInt(static_cast<int>(floor((start.y - miny) / cell + 0.5)), 0, ny - 1);
    const int gi = ClampInt(static_cast<int>(floor((goal.x - minx) / cell + 0.5)), 0, nx - 1);
    const int gj = ClampInt(static_cast<int>(floor((goal.y - miny) / cell + 0.5)), 0, ny - 1);
    const int sIdx = sj * nx + si, gIdx = gj * nx + gi;

    const double dirx = (goal.x - start.x), diry = (goal.y - start.y);
    const double dl = hypot(dirx, diry);
    const double ux = dirx / dl, uy = diry / dl;

    const double kInf = 1e30;
    const double kDiag = 1.41421356237309504880;
    Array<double> g(static_cast<size_t>(n), kInf);
    Array<int> parent(static_cast<size_t>(n), -1);
    Array<char> closed(static_cast<size_t>(n), 0);
    Array<HeapItem> open;
    g[static_cast<size_t>(sIdx)] = 0.0;
    HeapPush(&open, HeapItem(0.0, sIdx));

    bool found = false;
    while (!open.empty()) {
        const HeapItem it = HeapPop(&open);
        if (closed[static_cast<size_t>(it.idx)]) continue;
        closed[static_cast<size_t>(it.idx)] = 1;
        if (it.idx == gIdx) { found = true; break; }
        const int ci = it.idx % nx, cj = it.idx / nx;
        for (int dj = -1; dj <= 1; dj++) {
            for (int di = -1; di <= 1; di++) {
                if (di == 0 && dj == 0) continue;
                const int ni = ci + di, nj = cj + dj;
                if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
                const int nIdx = nj * nx + ni;
                if (closed[static_cast<size_t>(nIdx)]) continue;
                const double step = (di != 0 && dj != 0) ? cell * kDiag : cell;
                double c = step * 0.5 * (mult[static_cast<size_t>(it.idx)] + mult[static_cast<size_t>(nIdx)]);
                const double lat = ux * (miny + nj * cell - start.y) - uy * (minx + ni * cell - start.x);   // 왼쪽이 +
                if (lat > 0.0) c += rightBias * lat * step;
                const double ng = g[static_cast<size_t>(it.idx)] + c;
                if (ng < g[static_cast<size_t>(nIdx)]) {
                    g[static_cast<size_t>(nIdx)] = ng;
                    parent[static_cast<size_t>(nIdx)] = it.idx;
                    const double ddx = static_cast<double>(abs(gi - ni)), ddy = static_cast<double>(abs(gj - nj));
                    const double h = cell * ((ddx > ddy ? ddx : ddy) + (kDiag - 1.0) * (ddx < ddy ? ddx : ddy));
                    HeapPush(&open, HeapItem(ng + h, nIdx));
                }
            }
        }
    }
    if (!found) return false;

    // 칸 중심들을 시작 -> 목표 순서로
    Array<Point2> raw;
    for (int c = gIdx; c >= 0; c = parent[static_cast<size_t>(c)]) {
        raw.push_back(Point2(minx + (c % nx) * cell, miny + (c / nx) * cell));
        if (c == sIdx) break;
    }
    Array<Point2> pts;
    for (size_t k = raw.size(); k > 0; k--) pts.push_back(raw[k - 1]);
    if (pts.size() == 1) pts.push_back(goal);   // 시작과 목표가 같은 칸
    pts[0] = start;
    pts[pts.size() - 1] = goal;

    // 직선으로 갈 수 있는 가장 먼 점으로 건너뛰며 줄인다
    size_t anchor = 0;
    while (anchor + 1 < pts.size()) {
        size_t j = pts.size() - 1;
        while (j > anchor + 1 && !SegmentClear(pts[anchor], pts[j], obstacles)) j--;
        out->push_back(pts[j]);
        anchor = j;
    }
    return true;
}

}  // namespace sim
}  // namespace simul
