#ifndef CORE_CPROCESS_PROCESS_H
#define CORE_CPROCESS_PROCESS_H

#include <stddef.h>

#include "3d/dhVector.h"
#include "CBase/Array.h"
#include "CBase/String.h"

// 간단한 물류 공정: 물건(Item)을 A 지점에서 B 지점으로 옮긴다. 로봇은 쓰지 않는다 (물건이 직선으로 일정한 속도로 이동).
// 나중에 로봇이나 AMR 이 물건을 옮기게 되면 Item 의 이동만 그쪽에서 제어하게 바꾸면 되고, 지점과 상태/이벤트는 그대로 쓴다.
// 단위는 m, m/s, s. 3D 화면과는 독립이다: 호스트가 Step 을 부르고 물건의 position 을 화면 모델에 넣어 준다.
//
//   Process p;
//   p.AddPoint("A", dhVector(0, 0, 0));
//   p.AddPoint("B", dhVector(3, 0, 0));
//   int box = p.AddItem("box", "A");        // A 에 놓인 물건
//   p.Order(box, "B", 0.5);                 // B 로 0.5 m/s 로 보낸다
//   while (!p.AllIdle()) p.Step(0.02);      // 도착하면 AllIdle() 이 참
namespace simul {
namespace process {

struct Point {
    String name;
    dhVector position;   // 지점의 위치 (m)
};

enum ItemState {
    kItemWaiting = 0,   // 어떤 지점에 놓여 있고 보내라는 명령을 기다린다
    kItemMoving = 1,    // 이동 중
    kItemArrived = 2    // 명령받은 지점에 도착했다 (다시 Order 하면 또 보낼 수 있다)
};

struct Item {
    String name;
    dhVector position;   // 지금 위치 (m)
    ItemState state;
    int at;              // 놓여 있는 지점 번호 (이동 중이면 -1)
    int from;            // 마지막 이동의 출발 지점 (아직 안 움직였으면 -1)
    int to;              // 마지막 이동의 도착 지점 (아직 안 움직였으면 -1)
    double speed;        // 이동 속도 (m/s)
    double distance;     // 이번 이동의 전체 거리 (m)
    double travelled;    // 이번 이동에서 지금까지 간 거리 (m)
    Item() : position(), state(kItemWaiting), at(-1), from(-1), to(-1), speed(0.0), distance(0.0), travelled(0.0) {}
};

enum EventType {
    kEventStarted = 0,   // 물건이 출발했다
    kEventArrived = 1    // 물건이 도착했다
};

struct Event {
    EventType type;
    int item;
    int point;           // 출발 이벤트면 출발 지점, 도착 이벤트면 도착 지점
    double time;         // 공정 시계 (s)
    Event() : type(kEventStarted), item(-1), point(-1), time(0.0) {}
};

class Process {
public:
    Process() : time_(0.0) {}

    // 지점을 더한다. 이름이 비었거나 이미 있으면 실패(-1)
    int AddPoint(const String& name, const dhVector& position, String* error = nullptr);
    int PointCount() const { return static_cast<int>(points_.size()); }
    const Point& GetPoint(int i) const { return points_[static_cast<size_t>(i)]; }
    int FindPoint(const String& name) const;   // 없으면 -1

    // 물건을 지점 위에 놓는다. 실패하면 -1
    int AddItem(const String& name, const String& atPoint, String* error = nullptr);
    int ItemCount() const { return static_cast<int>(items_.size()); }
    const Item& GetItem(int i) const { return items_[static_cast<size_t>(i)]; }
    int FindItem(const String& name) const;    // 없으면 -1

    // 물건을 다른 지점으로 보낸다. 물건이 지점에 놓여 있어야 하고(이동 중이면 안 된다), 속도는 0 보다 커야 하고,
    // 도착 지점이 지금 놓인 지점과 달라야 한다. 실패하면 false 와 이유.
    bool Order(int item, const String& toPoint, double speed, String* error = nullptr);

    // dt 초 만큼 공정을 진행한다. dt 가 커도 도착은 정확한 시각에 기록된다.
    void Step(double dt);

    // 이동 중인 물건이 하나도 없으면 true
    bool AllIdle() const;
    // 이동 중인 물건이 도착하기까지 남은 시간 (s). 이동 중이 아니면 0
    double TimeToArrival(int item) const;
    double Time() const { return time_; }   // 공정 시계 (s)

    // 지금까지 쌓인 이벤트를 꺼내고 비운다
    Array<Event> TakeEvents();

private:
    Array<Point> points_;
    Array<Item> items_;
    Array<Event> events_;
    double time_;
};

}  // namespace process
}  // namespace simul

#endif  // CORE_CPROCESS_PROCESS_H
