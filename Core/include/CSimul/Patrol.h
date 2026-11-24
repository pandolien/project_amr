// 순찰: 정해 둔 지점들을 위에서 아래로 차례로 걷는다. 끝에 닿으면 처음으로 돌아간다.
// 다른 주민이나 움직이지 않는 장애물(ParseObstacles)에 닿을 것 같으면 A* (AStar.h) 로 비켜 간다. SetAvoidance(false) 로 끄면 곧장 걷는다.
// (Qt 와 Python 에 의존하지 않는다)
//
//   Engine 의 Listener 로 붙인다. 엔진이 시작되면 이름이 맞는 오브젝트를 첫 지점에 놓고, 다음 지점으로 move_to 를 보낸다.
//   도착 이벤트(kEventArrived)가 오면 다음 지점으로 또 보낸다. 가는 방향을 바라보게 자세도 맞춘다.
//
//   지점 파일 (waypoints.txt):
//     # 주석
//     [villager_1]        <- 오브젝트 이름 (장면에서 그 이름으로 올라온 모델)
//     speed 1.3           <- m/s (생략하면 1)
//     18.9 -14.5 2.9      <- x y z (m, Z-up). 지점마다 한 줄
//     17.0 -14.1 3.2
//
//   모델의 앞쪽은 엔진 좌표에서 -Y 방향이어야 한다 (GLB 를 Z-up 으로 돌려 올리면 그렇게 된다).
#ifndef CORE_CSIMUL_PATROL_H
#define CORE_CSIMUL_PATROL_H

#include "3d/dhVector.h"
#include "CBase/Array.h"
#include "CBase/String.h"
#include "CSimul/AStar.h"
#include "CSimul/Engine.h"

namespace simul {
namespace sim {

struct PatrolRoute {
    String name;
    double speed;            // m/s
    Array<dhVector> points;  // 지점 (2개 이상)
    PatrolRoute() : speed(1.0) {}
};

class Patrol : public Listener {
public:
    Patrol() {}

    // 지점 파일 내용을 읽는다. 틀리면 false 와 이유(줄 번호 포함). 실패하면 이전 내용은 그대로 둔다.
    bool Parse(const String& text, String* error);
    int RouteCount() const { return static_cast<int>(routes_.size()); }
    const PatrolRoute& Route(int i) const { return routes_[static_cast<size_t>(i)]; }
    int Arrivals(int i) const { return state_[static_cast<size_t>(i)].reached; }   // 그 길에서 지금까지 도착한 횟수

    // 피하기: 다른 주민 둘레 radius(m) 안으로 들어가지 않는 길을 찾는다. 기본은 켜짐, 0.9 m.
    void SetAvoidance(bool on, double radius = 0.9) { avoid_ = on; radius_ = radius; }
    bool Avoidance() const { return avoid_; }
    // 서로 이 거리(m)보다 가까우면 밀어서 떼어 놓는다 (겹쳐서 출발하거나 빠르게 스쳤을 때). 0 이면 끈다. 기본 0.6 m.
    void SetSeparation(double meters) { sep_ = meters; }
    // 움직이지 않는 장애물(나무, 집 등): 한 줄에 "x y 반지름" (m, Z-up, # 주석). 주민 몸 반지름 0.4 m 를 더해서 피한다.
    bool ParseObstacles(const String& text, String* error);
    int ObstacleCount() const { return static_cast<int>(fixed_.size()); }
    const Circle2& Obstacle(int i) const { return fixed_[static_cast<size_t>(i)]; }
    // 대화 등으로 잠깐 세워 둔다: Hold 는 그 자리에 멈추고 (faceX, faceY) 쪽을 보게 하고, Release 는 가던 길을 다시 간다.
    // 세워 둔 주민은 길 확인과 밀어내기를 하지 않고, 다른 주민에게는 그냥 장애물이다.
    bool Hold(Engine& e, const String& name, double faceX, double faceY);
    bool Release(Engine& e, const String& name);
    bool Held(const String& name) const;
    bool Active(const String& name) const;   // 길이 있고 출발한 주민인가
    int Detours() const { return detours_; }   // 지금까지 길을 새로 짠 횟수
    // 시각화용: 켜 두면 A* 로 길을 새로 짤 때마다 탐색 과정을 남긴다 (꺼져 있으면 비용 없음).
    void SetTracing(bool on) { tracing_ = on; }
    int TraceSerial() const { return traceSerial_; }              // 기록이 새로 남을 때마다 1 씩 는다
    const AStarTrace& LastTrace() const { return trace_; }
    const String& LastTraceName() const { return traceName_; }    // 그 길을 짠 주민 이름

    void OnStart(Engine& e) override;
    void OnTick(Engine& e, double time, double dt) override;   // 떼어 놓기 -> 앞길 확인 -> 필요하면 A*
    void OnEvent(Engine& e, const Event& ev) override;

private:
    struct State {
        bool started;
        int next;       // 지금 향해 가는 지점 번호
        int reached;
        double yaw;     // 마지막으로 바라본 방향 (세로 이동일 때 유지)
        Array<dhVector> path;   // 다음 지점까지 가는 꺾임점 (비켜 가는 중이면 2개 이상, 마지막이 지점)
        size_t pathPos;         // path 에서 지금 향하는 곳
        double lastPlan;        // 마지막으로 길을 확인한 시각
        bool held;              // Hold 로 세워 둔 상태
        bool fresh;             // 방금 출발시켰다 (SetPose 가 다음 틱에 적용되므로 이번 틱에는 위치를 믿지 않는다)
        State() : started(false), next(0), reached(0), yaw(0.0), pathPos(0), lastPlan(-1e9), held(false), fresh(false) {}
    };
    int find(const String& name) const;
    bool begin(Engine& e, int route);                  // 첫 지점에 놓고 출발
    void go(Engine& e, Handle h, int route, int from, int to);
    void steer(Engine& e, Handle h, int route, const dhVector& from, const dhVector& to);   // from 에서 to 로 걷기 (자세 맞춤)

    Array<Circle2> fixed_;
    bool avoid_ = true;
    double sep_ = 0.6;
    double radius_ = 0.9;
    double time_ = 0.0;
    int detours_ = 0;
    bool tracing_ = false;
    int traceSerial_ = 0;
    AStarTrace trace_;
    String traceName_;

    Array<PatrolRoute> routes_;
    Array<State> state_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_PATROL_H
