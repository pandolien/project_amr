// 시뮬레이션 전체 루프: 오브젝트 정보, 명령 큐, 틱, 이벤트. (Qt 와 Python 에 의존하지 않는다)
//
//   호스트(3D 뷰, 테스트 등)가 Backend 를 구현해서 "지금 어떤 오브젝트가 있고 무슨 상태인지"를 알려 주고, 명령을 실제로 적용한다.
//   Engine 은 그 위에서 이름 -> 핸들 등록부, 명령 큐, 시간(틱), 움직임(속도로 이동), 이벤트를 맡는다.
//   Python 같은 스크립트는 Listener 로 붙어서 매 틱 불리고, 명령은 Engine 의 함수로 큐에 넣는다.
//
// 한 틱 (Step(dt)) 의 순서 [lockstep]:
//   1. Backend 와 등록부를 맞춘다 (오브젝트가 생기거나 사라졌으면 이벤트)
//   2. 큐에 쌓인 명령을 순서대로 적용한다 (이전 틱 끝, 또는 틱 사이에 들어온 것)
//   3. 움직이는 중인 것(move_to, move_joint)을 dt 만큼 진행한다. 목표에 닿으면 이벤트
//   4. 시간을 dt 만큼 올린다
//   5. 이번 틱에 생긴 이벤트마다 Listener::OnEvent, 그리고 Listener::OnTick
//   Listener 가 5 에서 넣은 명령은 다음 틱의 2 에서 적용된다. 그래서 스크립트가 느려도 결과는 같다 (결정성).
//
// 핸들은 (슬롯, 세대)다. 오브젝트가 지워졌다가 같은 이름으로 다시 생기면 세대가 달라서, 옛 핸들은 안전하게 거부된다.
// 단위: m, rad, s, 자세는 단위 쿼터니언 (dhPose).
#ifndef CORE_CSIMUL_ENGINE_H
#define CORE_CSIMUL_ENGINE_H

#include <stddef.h>

#include "3d/dhPose.h"
#include "3d/dhVector.h"
#include "CBase/Array.h"
#include "CBase/String.h"

namespace simul {
namespace sim {

struct Handle {
    int slot;
    int gen;
    Handle() : slot(-1), gen(0) {}
    Handle(int s, int g) : slot(s), gen(g) {}
    bool operator==(const Handle& o) const { return slot == o.slot && gen == o.gen; }
    bool operator!=(const Handle& o) const { return !(*this == o); }
};

enum JointKind { kJointFixedKind = 0, kJointRevoluteKind = 1, kJointPrismaticKind = 2, kJointContinuousKind = 3 };

struct JointInfo {
    String name;
    int kind;            // JointKind. 회전은 rad, 직선은 m
    double value;        // 지금 값
    double lower, upper; // hasLimit 일 때만 뜻이 있다
    double velocity;     // 최대 속도 (rad/s 또는 m/s). 0 이면 정해져 있지 않다
    bool hasLimit;
    JointInfo() : kind(0), value(0), lower(0), upper(0), velocity(0), hasLimit(false) {}
};

struct ObjectInfo {
    String name;              // 장면 안에서 고유한 이름
    String kind;              // "robot"(움직이는 관절이 있다) / "model"(그 밖)
    dhPose pose;              // 월드 자세
    Array<JointInfo> joints;  // 움직이는 관절만
};

// 지금 서로 겹치는 두 부분 (OBB 까지 겹친 것). 같은 오브젝트 안의 링크끼리면 sameObject 가 true (자기충돌).
struct ContactInfo {
    String objectA, linkA;
    String objectB, linkB;
    bool sameObject;
    ContactInfo() : sameObject(false) {}
};

// 호스트가 구현한다. 인덱스는 0 .. ObjectCount()-1 이고 Engine 은 이름으로 오브젝트를 알아본다 (인덱스는 오브젝트가 지워지면 바뀔 수 있다).
class Backend {
public:
    virtual ~Backend() {}
    virtual int ObjectCount() const = 0;
    virtual bool ReadObject(int index, ObjectInfo* out) const = 0;
    virtual bool SetPose(int index, const dhPose& pose, String* error) = 0;
    virtual bool SetJoint(int index, const String& joint, double value, String* error) = 0;
    // 지금 겹쳐 있는 충돌 쌍. 충돌 검사를 하지 않는 호스트는 비워 두면 된다
    virtual void ReadCollisions(Array<ContactInfo>* out) const { out->clear(); }
};

enum EventType {
    kEventAdded = 0,        // 오브젝트가 생겼다
    kEventRemoved = 1,      // 오브젝트가 사라졌다 (핸들은 이미 쓸 수 없다. name 에 이름이 남는다)
    kEventArrived = 2,      // move_to 가 목표에 닿았다
    kEventJointReached = 3, // move_joint 가 목표에 닿았다 (joint 에 관절 이름)
    kEventCommandFailed = 4 // 적용하려던 명령이 실패했다 (message 에 이유)
};

struct Event {
    EventType type;
    Handle object;
    String name;      // 오브젝트 이름
    String joint;     // kEventJointReached
    String message;   // kEventCommandFailed
    double time;      // 시뮬레이션 시계 (s)
    Event() : type(kEventAdded), time(0.0) {}
};

const char* EventTypeName(EventType t);   // "added" "removed" "arrived" "joint_reached" "command_failed"

class Engine;

class Listener {
public:
    virtual ~Listener() {}
    virtual void OnStart(Engine&) {}
    virtual void OnTick(Engine&, double /*time*/, double /*dt*/) {}
    virtual void OnEvent(Engine&, const Event&) {}
    virtual void OnStop(Engine&) {}
};

class Engine {
public:
    Engine();
    explicit Engine(Backend* backend);
    ~Engine();

    void SetBackend(Backend* backend);
    Backend* GetBackend() const { return backend_; }
    void SetListener(Listener* listener) { listener_ = listener; }
    Listener* GetListener() const { return listener_; }

    // ---- 시간 ----
    double Time() const { return time_; }          // 시뮬레이션 시계 (s)
    unsigned long long TickCount() const { return tick_; }
    double LastDt() const { return lastDt_; }
    bool Running() const { return running_; }
    void Start();                                  // 등록부를 맞추고 Listener::OnStart (이미 돌고 있으면 무시)
    void Stop();                                   // Listener::OnStop (돌고 있지 않으면 무시)
    void Reset();                                  // 시간 0, 명령과 움직임을 비운다 (Backend 는 그대로)
    // dt 초 진행한다 (dt > 0, 유한). 위의 순서대로. 이벤트는 Listener 에게 주고 TakeEvents 로도 꺼낼 수 있다.
    bool Step(double dt, String* error = nullptr);

    // ---- 오브젝트 정보 ----
    int Refresh();                                 // Backend 와 맞춘다. 살아 있는 오브젝트 수
    int Count() const { return static_cast<int>(order_.size()); }
    Handle HandleAt(int i) const { return order_[static_cast<size_t>(i)]; }   // Backend 순서
    Handle Find(const String& name);               // Backend 와 먼저 맞춘다. 없으면 slot -1
    bool Alive(Handle h) const;
    String Name(Handle h) const;                   // 죽은 핸들이면 빈 문자열
    void Contacts(Array<ContactInfo>* out) const { backend_->ReadCollisions(out); }   // 지금 겹친 쌍 (Backend 에서 읽는다)
    bool Info(Handle h, ObjectInfo* out);          // Backend 에서 지금 상태를 읽는다. 죽은 핸들이면 false

    // ---- 명령 (큐에 넣는다. 검사해서 틀리면 false 와 이유. 적용은 다음 틱 시작) ----
    bool SetPose(Handle h, const dhPose& pose, String* error = nullptr);                     // 즉시 자세 지정
    bool SetJoint(Handle h, const String& joint, double value, String* error = nullptr);     // 즉시 관절 값 지정 (그 관절의 움직임은 멈춘다)
    bool MoveTo(Handle h, const dhVector& position, double speed, String* error = nullptr);  // 직선으로 speed(m/s) 로 이동. 닿으면 kEventArrived
    bool MoveJoint(Handle h, const String& joint, double target, double speed, String* error = nullptr);  // speed <= 0 이면 관절의 최대 속도(없으면 1)
    bool StopMotion(Handle h, String* error = nullptr);                                       // 그 오브젝트의 move_to, move_joint 를 모두 멈춘다
    size_t PendingCommands() const { return commands_.size(); }

    bool Moving(Handle h) const;                   // move_to 나 move_joint 가 진행 중인가
    Array<Event> TakeEvents();                     // 쌓인 이벤트를 꺼내고 비운다

private:
    struct Slot {
        String name;
        int gen;
        bool alive;
        int index;   // Backend 인덱스
        bool moveActive;
        dhVector moveTarget;
        double moveSpeed;
        struct JointMotion {
            String joint;
            double target, speed;
        };
        Array<JointMotion> joints;
        Slot() : gen(1), alive(false), index(-1), moveActive(false), moveSpeed(0.0) {}
    };
    enum CmdType { kCmdPose, kCmdJoint, kCmdMoveTo, kCmdMoveJoint, kCmdStop };
    struct Command {
        CmdType type;
        Handle h;
        dhPose pose;
        dhVector pos;
        String joint;
        double value, speed;
        Command() : type(kCmdStop), value(0.0), speed(0.0) {}
    };

    Slot* slotOf(Handle h);
    const Slot* slotOf(Handle h) const;
    void push(const Event& e);
    Event makeEvent(EventType t, const Slot& s) const;
    void applyCommand(const Command& c);
    void advance(double dt);
    bool checkJoint(Handle h, const String& joint, double value, bool checkRange, String* error);

    Backend* backend_;
    Listener* listener_;
    Array<Slot> slots_;
    Array<Handle> order_;
    Array<Command> commands_;
    Array<Event> events_;
    Array<Event> toDeliver_;
    double time_, lastDt_;
    unsigned long long tick_;
    bool running_;
    bool inStep_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_ENGINE_H
