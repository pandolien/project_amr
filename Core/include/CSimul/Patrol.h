// 순찰: 정해 둔 지점들을 위에서 아래로 차례로 걷는다. 끝에 닿으면 처음으로 돌아간다. 길 찾기는 없다.
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

    void OnStart(Engine& e) override;
    void OnEvent(Engine& e, const Event& ev) override;

private:
    struct State {
        bool started;
        int next;       // 지금 향해 가는 지점 번호
        int reached;
        double yaw;     // 마지막으로 바라본 방향 (세로 이동일 때 유지)
        State() : started(false), next(0), reached(0), yaw(0.0) {}
    };
    int find(const String& name) const;
    bool begin(Engine& e, int route);                  // 첫 지점에 놓고 출발
    void go(Engine& e, Handle h, int route, int from, int to);

    Array<PatrolRoute> routes_;
    Array<State> state_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_PATROL_H
