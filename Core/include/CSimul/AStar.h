// 평면(x, y) 격자 A*: 원형 장애물(다른 주민 등)을 피해 시작에서 목표까지 가는 경로를 만든다. (Qt 에 의존하지 않는다)
//
//   - 경로를 둘러싼 직사각형(margin 만큼 넓힘)만 cell 간격 격자로 나눠 8방향으로 찾는다.
//   - 장애물 안쪽 칸은 막힌 칸이 아니라 "비싼 칸"이다. 그래서 시작이나 목표가 이미 장애물 안에 있어도 경로가 나오고
//     (가장 빨리 빠져나오는 쪽으로), 둘러 갈 수 있으면 반드시 둘러 간다.
//   - rightBias > 0 이면 가는 방향의 왼쪽 칸을 조금 더 비싸게 쳐서 오른쪽으로 비킨다. 마주 오는 둘이 서로 오른쪽으로 비켜서 엇갈린다.
//   - 결과는 직선으로 이어 갈 수 있는 곳까지 줄인 꺾임점들이다 (시작 제외, 목표 포함).
#ifndef CORE_CSIMUL_ASTAR_H
#define CORE_CSIMUL_ASTAR_H

#include "CBase/Array.h"

namespace simul {
namespace sim {

struct Point2 {
    double x, y;
    Point2() : x(0), y(0) {}
    Point2(double px, double py) : x(px), y(py) {}
};

struct Circle2 {
    double x, y, r;   // 중심과 반지름 (m)
    Circle2() : x(0), y(0), r(0) {}
    Circle2(double px, double py, double pr) : x(px), y(py), r(pr) {}
};

// 탐색 과정 기록 (시각화용). FindPath2D 에 넘기면 채워 준다. 넘기지 않으면 아무 비용도 없다.
// 칸 번호는 j * nx + i, 칸 중심 = (minx + i * cell, miny + j * cell).
struct AStarTrace {
    double minx, miny, cell;
    int nx, ny;
    Point2 start, goal;
    Array<Circle2> obstacles;
    Array<int> expanded;   // 꺼내서 살펴본 칸 (꺼낸 순서)
    Array<int> rawPath;    // 격자 위 경로 (시작 -> 목표 칸 번호)
    Array<Point2> path;    // 직선으로 줄인 결과 (시작 제외, 목표 포함)
    bool found;
    AStarTrace() : minx(0), miny(0), cell(0), nx(0), ny(0), found(false) {}
};

// 점 p 에서 선분 ab 까지의 거리
double SegmentDistance(const Point2& a, const Point2& b, const Point2& p);
// 선분 ab 가 어느 장애물에도 닿지 않으면 true
bool SegmentClear(const Point2& a, const Point2& b, const Array<Circle2>& obstacles);

// cell: 격자 한 칸 (m), margin: 시작-목표 상자 둘레의 여유 (m), rightBias: 왼쪽 칸에 더하는 비용 (0 이면 치우침 없음, 0.1 ~ 0.3 권장).
// 격자가 너무 크거나(4만 칸 초과) 인자가 틀리면 false.
bool FindPath2D(const Point2& start, const Point2& goal, const Array<Circle2>& obstacles, double cell, double margin, double rightBias,
                Array<Point2>* out, AStarTrace* trace = nullptr);

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_ASTAR_H
