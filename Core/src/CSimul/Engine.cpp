#include "CSimul/Engine.h"

#include <math.h>
#include <stdio.h>

namespace simul {
namespace sim {

namespace {

bool Finite(double v) { return v == v && v - v == 0.0; }

String Fmt(const char* f, const char* a, const char* b = "") {
    char buf[512];
    snprintf(buf, sizeof(buf), f, a, b);
    return String(buf);
}

const size_t kMaxKeptEvents = 4096;

}  // namespace

const char* EventTypeName(EventType t) {
    switch (t) {
        case kEventAdded: return "added";
        case kEventRemoved: return "removed";
        case kEventArrived: return "arrived";
        case kEventJointReached: return "joint_reached";
        case kEventCommandFailed: return "command_failed";
    }
    return "";
}

Engine::Engine() : backend_(nullptr), listener_(nullptr), time_(0.0), lastDt_(0.0), tick_(0), running_(false), inStep_(false) {}
Engine::Engine(Backend* backend) : backend_(backend), listener_(nullptr), time_(0.0), lastDt_(0.0), tick_(0), running_(false), inStep_(false) {}
Engine::~Engine() {}

void Engine::SetBackend(Backend* backend) {
    backend_ = backend;
    slots_.clear();
    order_.clear();
    commands_.clear();
}

Engine::Slot* Engine::slotOf(Handle h) {
    if (h.slot < 0 || static_cast<size_t>(h.slot) >= slots_.size()) return nullptr;
    Slot* s = &slots_[static_cast<size_t>(h.slot)];
    return s->alive && s->gen == h.gen ? s : nullptr;
}

const Engine::Slot* Engine::slotOf(Handle h) const {
    if (h.slot < 0 || static_cast<size_t>(h.slot) >= slots_.size()) return nullptr;
    const Slot* s = &slots_[static_cast<size_t>(h.slot)];
    return s->alive && s->gen == h.gen ? s : nullptr;
}

Event Engine::makeEvent(EventType t, const Slot& s) const {
    Event e;
    e.type = t;
    e.name = s.name;
    e.time = time_;
    return e;
}

void Engine::push(const Event& e) {
    events_.push_back(e);
    if (events_.size() > kMaxKeptEvents) events_.erase(events_.begin(), events_.begin() + kMaxKeptEvents / 2);
    toDeliver_.push_back(e);
    if (toDeliver_.size() > kMaxKeptEvents) toDeliver_.erase(toDeliver_.begin(), toDeliver_.begin() + kMaxKeptEvents / 2);
}

int Engine::Refresh() {
    if (!backend_) {
        // Backend 가 없으면 모두 사라진 것으로 본다
        for (size_t i = 0; i < slots_.size(); i++) {
            if (!slots_[i].alive) continue;
            Event e = makeEvent(kEventRemoved, slots_[i]);
            e.object = Handle(static_cast<int>(i), slots_[i].gen);
            slots_[i].alive = false;
            slots_[i].gen++;
            slots_[i].moveActive = false;
            slots_[i].joints.clear();
            push(e);
        }
        order_.clear();
        return 0;
    }
    const int n = backend_->ObjectCount();
    Array<String> names;
    for (int i = 0; i < n; i++) {
        ObjectInfo info;
        backend_->ReadObject(i, &info);
        names.push_back(info.name);
    }
    Array<char> seen(slots_.size(), 0);
    // 이미 있는 것은 인덱스만 갱신한다. 같은 이름이 둘이면 앞의 것만 쓴다.
    Array<char> used(static_cast<size_t>(n), 0);
    for (size_t s = 0; s < slots_.size(); s++) {
        if (!slots_[s].alive) continue;
        int found = -1;
        for (int i = 0; i < n; i++)
            if (!used[static_cast<size_t>(i)] && names[static_cast<size_t>(i)] == slots_[s].name) {
                found = i;
                break;
            }
        if (found >= 0) {
            used[static_cast<size_t>(found)] = 1;
            slots_[s].index = found;
            seen[s] = 1;
        }
    }
    for (size_t s = 0; s < slots_.size(); s++) {
        if (!slots_[s].alive || seen[s]) continue;
        Event e = makeEvent(kEventRemoved, slots_[s]);
        e.object = Handle(static_cast<int>(s), slots_[s].gen);
        slots_[s].alive = false;
        slots_[s].gen++;
        slots_[s].index = -1;
        slots_[s].moveActive = false;
        slots_[s].joints.clear();
        push(e);
    }
    for (int i = 0; i < n; i++) {
        if (used[static_cast<size_t>(i)]) continue;
        // 이름이 같은 앞의 것이 이미 있으면 건너뛴다 (used 는 위에서 표시됨. 여기서는 새 이름이다)
        bool dup = false;
        for (int k = 0; k < i; k++)
            if (names[static_cast<size_t>(k)] == names[static_cast<size_t>(i)]) dup = true;
        if (dup) continue;
        int slot = -1;
        for (size_t s = 0; s < slots_.size(); s++)
            if (!slots_[s].alive) {
                slot = static_cast<int>(s);
                break;
            }
        if (slot < 0) {
            slots_.push_back(Slot());
            slot = static_cast<int>(slots_.size()) - 1;
        }
        Slot& sl = slots_[static_cast<size_t>(slot)];
        sl.name = names[static_cast<size_t>(i)];
        sl.alive = true;
        sl.index = i;
        sl.moveActive = false;
        sl.joints.clear();
        Event e = makeEvent(kEventAdded, sl);
        e.object = Handle(slot, sl.gen);
        push(e);
    }
    order_.clear();
    for (int i = 0; i < n; i++)
        for (size_t s = 0; s < slots_.size(); s++)
            if (slots_[s].alive && slots_[s].index == i && slots_[s].name == names[static_cast<size_t>(i)]) {
                order_.push_back(Handle(static_cast<int>(s), slots_[s].gen));
                break;
            }
    return static_cast<int>(order_.size());
}

Handle Engine::Find(const String& name) {
    Refresh();
    for (size_t i = 0; i < order_.size(); i++)
        if (slots_[static_cast<size_t>(order_[i].slot)].name == name) return order_[i];
    return Handle();
}

bool Engine::Alive(Handle h) const { return slotOf(h) != nullptr; }

String Engine::Name(Handle h) const {
    const Slot* s = slotOf(h);
    return s ? s->name : String();
}

bool Engine::Info(Handle h, ObjectInfo* out) {
    Slot* s = slotOf(h);
    if (!s || !backend_) return false;
    if (s->index < 0 || s->index >= backend_->ObjectCount() || !backend_->ReadObject(s->index, out) || !(out->name == s->name)) {
        Refresh();
        s = slotOf(h);
        if (!s || !backend_->ReadObject(s->index, out)) return false;
    }
    return true;
}

bool Engine::checkJoint(Handle h, const String& joint, double value, bool checkRange, String* error) {
    ObjectInfo info;
    if (!Info(h, &info)) {
        if (error) *error = "오브젝트가 없습니다 (사라졌거나 잘못된 핸들)";
        return false;
    }
    if (!Finite(value)) {
        if (error) *error = "값이 유한한 숫자가 아닙니다";
        return false;
    }
    for (size_t i = 0; i < info.joints.size(); i++) {
        const JointInfo& j = info.joints[i];
        if (!(j.name == joint)) continue;
        if (checkRange && j.hasLimit && j.kind != kJointContinuousKind && (value < j.lower - 1e-9 || value > j.upper + 1e-9)) {
            char buf[256];
            snprintf(buf, sizeof(buf), "관절 '%s' 의 목표 %g 가 한계 [%g, %g] 를 벗어납니다", joint.c_str(), value, j.lower, j.upper);
            if (error) *error = buf;
            return false;
        }
        return true;
    }
    if (error) *error = Fmt("오브젝트 '%s' 에 움직이는 관절 '%s' 가 없습니다", info.name.c_str(), joint.c_str());
    return false;
}

bool Engine::SetPose(Handle h, const dhPose& pose, String* error) {
    if (!slotOf(h)) {
        if (error) *error = "오브젝트가 없습니다 (사라졌거나 잘못된 핸들)";
        return false;
    }
    double a[7];
    pose.ToArray(a);
    for (int i = 0; i < 7; i++)
        if (!Finite(a[i])) {
            if (error) *error = "자세 값이 유한한 숫자가 아닙니다";
            return false;
        }
    Command c;
    c.type = kCmdPose;
    c.h = h;
    c.pose = pose.Normalized();
    commands_.push_back(c);
    return true;
}

bool Engine::SetJoint(Handle h, const String& joint, double value, String* error) {
    if (!checkJoint(h, joint, value, true, error)) return false;
    Command c;
    c.type = kCmdJoint;
    c.h = h;
    c.joint = joint;
    c.value = value;
    commands_.push_back(c);
    return true;
}

bool Engine::MoveTo(Handle h, const dhVector& position, double speed, String* error) {
    if (!slotOf(h)) {
        if (error) *error = "오브젝트가 없습니다 (사라졌거나 잘못된 핸들)";
        return false;
    }
    if (!Finite(position.x) || !Finite(position.y) || !Finite(position.z) || !Finite(speed) || !(speed > 0.0)) {
        if (error) *error = "목표 위치는 유한한 숫자이고 속도는 0 보다 커야 합니다";
        return false;
    }
    Command c;
    c.type = kCmdMoveTo;
    c.h = h;
    c.pos = position;
    c.speed = speed;
    commands_.push_back(c);
    return true;
}

bool Engine::MoveJoint(Handle h, const String& joint, double target, double speed, String* error) {
    if (!checkJoint(h, joint, target, true, error)) return false;
    if (!Finite(speed)) {
        if (error) *error = "속도가 유한한 숫자가 아닙니다";
        return false;
    }
    Command c;
    c.type = kCmdMoveJoint;
    c.h = h;
    c.joint = joint;
    c.value = target;
    c.speed = speed;
    commands_.push_back(c);
    return true;
}

bool Engine::StopMotion(Handle h, String* error) {
    if (!slotOf(h)) {
        if (error) *error = "오브젝트가 없습니다 (사라졌거나 잘못된 핸들)";
        return false;
    }
    Command c;
    c.type = kCmdStop;
    c.h = h;
    commands_.push_back(c);
    return true;
}

bool Engine::Moving(Handle h) const {
    const Slot* s = slotOf(h);
    return s && (s->moveActive || !s->joints.empty());
}

void Engine::applyCommand(const Command& c) {
    Slot* s = slotOf(c.h);
    if (!s) {
        Event e;
        e.type = kEventCommandFailed;
        e.object = c.h;
        e.time = time_;
        e.message = "명령을 적용하기 전에 오브젝트가 사라졌습니다";
        push(e);
        return;
    }
    String err;
    bool ok = true;
    switch (c.type) {
        case kCmdPose:
            ok = backend_ && backend_->SetPose(s->index, c.pose, &err);
            if (ok) s->moveActive = false;
            break;
        case kCmdJoint:
            ok = backend_ && backend_->SetJoint(s->index, c.joint, c.value, &err);
            if (ok) {
                for (size_t k = s->joints.size(); k-- > 0;)
                    if (s->joints[k].joint == c.joint) s->joints.erase(s->joints.begin() + k);
            }
            break;
        case kCmdMoveTo:
            s->moveActive = true;
            s->moveTarget = c.pos;
            s->moveSpeed = c.speed;
            break;
        case kCmdMoveJoint: {
            double speed = c.speed;
            if (!(speed > 0.0)) {
                ObjectInfo info;
                speed = 1.0;
                if (backend_ && backend_->ReadObject(s->index, &info))
                    for (size_t k = 0; k < info.joints.size(); k++)
                        if (info.joints[k].name == c.joint && info.joints[k].velocity > 0.0) speed = info.joints[k].velocity;
            }
            bool found = false;
            for (size_t k = 0; k < s->joints.size(); k++)
                if (s->joints[k].joint == c.joint) {
                    s->joints[k].target = c.value;
                    s->joints[k].speed = speed;
                    found = true;
                }
            if (!found) {
                Slot::JointMotion m;
                m.joint = c.joint;
                m.target = c.value;
                m.speed = speed;
                s->joints.push_back(m);
            }
            break;
        }
        case kCmdStop:
            s->moveActive = false;
            s->joints.clear();
            break;
    }
    if (!ok) {
        Event e = makeEvent(kEventCommandFailed, *s);
        e.object = c.h;
        e.message = err.empty() ? String("명령이 적용되지 않았습니다") : err;
        push(e);
    }
}

void Engine::advance(double dt) {
    if (!backend_) return;
    for (size_t si = 0; si < slots_.size(); si++) {
        Slot& s = slots_[si];
        if (!s.alive || (!s.moveActive && s.joints.empty())) continue;
        ObjectInfo info;
        if (s.index < 0 || !backend_->ReadObject(s.index, &info)) continue;
        const Handle h(static_cast<int>(si), s.gen);
        if (s.moveActive) {
            const double dx = s.moveTarget.x - info.pose.position.x, dy = s.moveTarget.y - info.pose.position.y, dz = s.moveTarget.z - info.pose.position.z;
            const double dist = sqrt(dx * dx + dy * dy + dz * dz), step = s.moveSpeed * dt;
            dhPose p = info.pose;
            bool done = false;
            if (dist <= step || dist < 1e-9) {
                p.position = s.moveTarget;
                done = true;
            } else {
                const double k = step / dist;
                p.position = dhVector(static_cast<float>(info.pose.position.x + dx * k), static_cast<float>(info.pose.position.y + dy * k), static_cast<float>(info.pose.position.z + dz * k));
            }
            String err;
            if (!backend_->SetPose(s.index, p, &err)) {
                Event e = makeEvent(kEventCommandFailed, s);
                e.object = h;
                e.message = err;
                push(e);
                s.moveActive = false;
            } else if (done) {
                s.moveActive = false;
                Event e = makeEvent(kEventArrived, s);
                e.object = h;
                push(e);
            }
        }
        for (size_t k = s.joints.size(); k-- > 0;) {
            const Slot::JointMotion m = s.joints[k];
            double cur = 0.0;
            bool found = false;
            for (size_t q = 0; q < info.joints.size(); q++)
                if (info.joints[q].name == m.joint) {
                    cur = info.joints[q].value;
                    found = true;
                }
            if (!found) {
                s.joints.erase(s.joints.begin() + k);
                continue;
            }
            const double diff = m.target - cur, step = m.speed * dt;
            const bool done = fabs(diff) <= step;
            const double next = done ? m.target : cur + (diff > 0 ? step : -step);
            String err;
            if (!backend_->SetJoint(s.index, m.joint, next, &err)) {
                Event e = makeEvent(kEventCommandFailed, s);
                e.object = h;
                e.joint = m.joint;
                e.message = err;
                push(e);
                s.joints.erase(s.joints.begin() + k);
            } else if (done) {
                s.joints.erase(s.joints.begin() + k);
                Event e = makeEvent(kEventJointReached, s);
                e.object = h;
                e.joint = m.joint;
                push(e);
            }
        }
    }
}

void Engine::Start() {
    if (running_) return;
    running_ = true;
    Refresh();
    if (listener_) listener_->OnStart(*this);
}

void Engine::Stop() {
    if (!running_) return;
    running_ = false;
    if (listener_) listener_->OnStop(*this);
}

void Engine::Reset() {
    time_ = 0.0;
    lastDt_ = 0.0;
    tick_ = 0;
    commands_.clear();
    events_.clear();
    toDeliver_.clear();
    for (size_t i = 0; i < slots_.size(); i++) {
        slots_[i].moveActive = false;
        slots_[i].joints.clear();
    }
}

bool Engine::Step(double dt, String* error) {
    if (inStep_) {
        if (error) *error = "Step 안에서 다시 Step 을 부를 수 없습니다";
        return false;
    }
    if (!Finite(dt) || !(dt > 0.0)) {
        if (error) *error = "dt 는 0 보다 큰 유한한 숫자여야 합니다";
        return false;
    }
    inStep_ = true;
    Refresh();
    Array<Command> cmds;
    cmds.swap(commands_);
    for (size_t i = 0; i < cmds.size(); i++) applyCommand(cmds[i]);
    advance(dt);
    time_ += dt;
    lastDt_ = dt;
    tick_++;
    Array<Event> deliver;
    deliver.swap(toDeliver_);
    if (listener_) {
        for (size_t i = 0; i < deliver.size(); i++) listener_->OnEvent(*this, deliver[i]);
        listener_->OnTick(*this, time_, dt);
    }
    inStep_ = false;
    return true;
}

Array<Event> Engine::TakeEvents() {
    Array<Event> out;
    out.swap(events_);
    return out;
}

}  // namespace sim
}  // namespace simul
