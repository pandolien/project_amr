// CSimul A* 테스트: 장애물이 없으면 직선, 둘러 가기, 오른쪽 치우침, 장애물 안에서 시작/도착, 잘못된 인자
#include <math.h>
#include <stdio.h>

#include "CSimul/AStar.h"

using namespace simul;
using namespace simul::sim;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

double pathLength(const Point2& s, const Array<Point2>& p) {
    double len = 0;
    Point2 a = s;
    for (size_t i = 0; i < p.size(); i++) {
        len += hypot(p[i].x - a.x, p[i].y - a.y);
        a = p[i];
    }
    return len;
}

bool pathClear(const Point2& s, const Array<Point2>& p, const Array<Circle2>& obs) {
    Point2 a = s;
    for (size_t i = 0; i < p.size(); i++) {
        if (!SegmentClear(a, p[i], obs)) return false;
        a = p[i];
    }
    return true;
}

void testBasics() {
    Array<Circle2> none;
    Array<Point2> out;
    check(FindPath2D(Point2(0, 0), Point2(5, 0), none, 0.25, 2.0, 0.15, &out), "장애물 없이 찾는다");
    check(out.size() == 1 && out[0].x == 5.0 && out[0].y == 0.0, "장애물이 없으면 목표 하나로 줄어든다 (직선)");
    check(FindPath2D(Point2(1, 1), Point2(1, 1), none, 0.25, 2.0, 0.15, &out) && out.size() == 1, "시작 = 목표");
    check(!FindPath2D(Point2(0, 0), Point2(5, 0), none, 0.0, 2.0, 0.15, &out), "cell 이 0 이하면 실패");
    check(!FindPath2D(Point2(0, 0), Point2(5, 0), none, 0.25, -1.0, 0.15, &out), "margin 이 음수면 실패");
    check(!FindPath2D(Point2(0, 0), Point2(500, 0), none, 0.1, 20.0, 0.15, &out), "격자가 너무 크면 실패");
    check(out.empty(), "실패하면 결과는 비어 있다");
    check(FindPath2D(Point2(0, 0), Point2(0.05, 0.02), none, 0.25, 2.0, 0.15, &out) && out.size() == 1 && out[0].x == 0.05, "시작과 목표가 같은 칸이어도 목표가 나온다");
}

void testDetour() {
    Array<Circle2> obs;
    obs.push_back(Circle2(5.0, 0.0, 0.9));    // 가는 길 한가운데
    Array<Point2> out;
    check(FindPath2D(Point2(0, 0), Point2(10, 0), obs, 0.25, 3.0, 0.15, &out), "장애물 둘레 경로");
    check(out.size() >= 2, "꺾임점이 생긴다");
    check(pathClear(Point2(0, 0), out, obs), "경로의 모든 선분이 장애물에서 반지름 이상 떨어진다");
    check(out[out.size() - 1].x == 10.0 && out[out.size() - 1].y == 0.0, "마지막은 목표");
    const double len = pathLength(Point2(0, 0), out);
    check(len > 10.0 && len < 12.0, "돌아가도 크게 길지 않다 (10 ~ 12 m)");
    bool right = false;
    for (size_t i = 0; i + 1 < out.size(); i++) right = right || out[i].y < -0.5;
    check(right, "정면에 장애물이 있으면 가는 방향의 오른쪽(-y)으로 비킨다");

    // 반대로 가면 (오른쪽이 +y) 반대쪽으로 비킨다 -> 마주 오는 둘이 서로 엇갈린다
    check(FindPath2D(Point2(10, 0), Point2(0, 0), obs, 0.25, 3.0, 0.15, &out), "반대 방향");
    bool left = false;
    for (size_t i = 0; i + 1 < out.size(); i++) left = left || out[i].y > 0.5;
    check(left, "반대로 가면 +y 쪽으로 비킨다");

    // 장애물이 둘이면 둘 다 피한다
    obs.push_back(Circle2(5.0, -1.8, 0.9));
    check(FindPath2D(Point2(0, 0), Point2(10, 0), obs, 0.25, 3.0, 0.15, &out) && pathClear(Point2(0, 0), out, obs), "장애물 둘을 모두 피한다");
}

void testInsideObstacle() {
    Array<Circle2> obs;
    obs.push_back(Circle2(0.2, 0.0, 0.9));   // 시작이 이미 안쪽 (겹쳐 있는 상태)
    Array<Point2> out;
    check(FindPath2D(Point2(0, 0), Point2(6, 0), obs, 0.25, 3.0, 0.15, &out), "시작이 장애물 안이어도 경로가 나온다");
    check(out.size() >= 1 && out[out.size() - 1].x == 6.0, "목표에 닿는다");
    Array<Circle2> obs2;
    obs2.push_back(Circle2(6.0, 0.0, 0.9));   // 목표가 장애물 안
    check(FindPath2D(Point2(0, 0), Point2(6, 0.2), obs2, 0.25, 3.0, 0.15, &out), "목표가 장애물 안이어도 경로가 나온다");
}

void testTrace() {
    Array<Circle2> obs;
    obs.push_back(Circle2(5.0, 0.0, 0.9));
    Array<Point2> out;
    AStarTrace t;
    check(FindPath2D(Point2(0, 0), Point2(10, 0), obs, 0.25, 3.0, 0.15, &out, &t), "trace 와 함께 찾는다");
    check(t.found && t.nx > 0 && t.ny > 0 && t.cell == 0.25, "격자 정보가 남는다");
    check(t.expanded.size() > 10 && t.rawPath.size() >= 2, "탐색한 칸과 격자 경로가 남는다");
    check(t.rawPath[0] == t.expanded[0], "첫 번째로 꺼낸 칸은 시작 칸");
    check(t.rawPath[t.rawPath.size() - 1] == t.expanded[t.expanded.size() - 1], "마지막에 꺼낸 칸은 목표 칸");
    check(t.path.size() == out.size() && t.obstacles.size() == 1, "줄인 경로와 장애물이 남는다");
    check(t.start.x == 0.0 && t.goal.x == 10.0, "시작/목표가 남는다");
    check(!FindPath2D(Point2(0, 0), Point2(10, 0), obs, 0.0, 3.0, 0.15, &out, &t) && !t.found, "실패하면 found 는 false");
}

}  // namespace

int main() {
    testBasics();
    testDetour();
    testInsideObstacle();
    testTrace();
    if (g_fail) {
        printf("test_castar: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_castar: all passed\n");
    return 0;
}
