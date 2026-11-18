#include "CProcess/Process.h"

#include "CBase/Math.h"

namespace simul {
namespace process {

namespace {
void SetError(String* error, const String& msg) {
    if (error) *error = msg;
}
}  // namespace

int Process::FindPoint(const String& name) const {
    for (size_t i = 0; i < points_.size(); i++)
        if (points_[i].name == name) return static_cast<int>(i);
    return -1;
}

int Process::FindItem(const String& name) const {
    for (size_t i = 0; i < items_.size(); i++)
        if (items_[i].name == name) return static_cast<int>(i);
    return -1;
}

int Process::AddPoint(const String& name, const dhVector& position, String* error) {
    if (name.empty()) {
        SetError(error, "지점 이름이 비어 있습니다");
        return -1;
    }
    if (FindPoint(name) >= 0) {
        SetError(error, "이미 있는 지점 이름입니다: " + name);
        return -1;
    }
    Point p;
    p.name = name;
    p.position = position;
    points_.push_back(p);
    return static_cast<int>(points_.size()) - 1;
}

int Process::AddItem(const String& name, const String& atPoint, String* error) {
    if (name.empty()) {
        SetError(error, "물건 이름이 비어 있습니다");
        return -1;
    }
    if (FindItem(name) >= 0) {
        SetError(error, "이미 있는 물건 이름입니다: " + name);
        return -1;
    }
    const int at = FindPoint(atPoint);
    if (at < 0) {
        SetError(error, "없는 지점입니다: " + atPoint);
        return -1;
    }
    Item it;
    it.name = name;
    it.position = points_[static_cast<size_t>(at)].position;
    it.at = at;
    items_.push_back(it);
    return static_cast<int>(items_.size()) - 1;
}

bool Process::Order(int item, const String& toPoint, double speed, String* error) {
    if (item < 0 || static_cast<size_t>(item) >= items_.size()) {
        SetError(error, "없는 물건 번호입니다");
        return false;
    }
    Item& it = items_[static_cast<size_t>(item)];
    if (it.state == kItemMoving) {
        SetError(error, "이미 이동 중인 물건입니다: " + it.name);
        return false;
    }
    const int to = FindPoint(toPoint);
    if (to < 0) {
        SetError(error, "없는 지점입니다: " + toPoint);
        return false;
    }
    if (!(speed > 0.0)) {
        SetError(error, "속도는 0 보다 커야 합니다");
        return false;
    }
    if (to == it.at) {
        SetError(error, "물건이 이미 그 지점에 있습니다: " + toPoint);
        return false;
    }
    it.from = it.at;
    it.to = to;
    it.speed = speed;
    it.distance = static_cast<double>(points_[static_cast<size_t>(it.from)].position.Norm(points_[static_cast<size_t>(to)].position));
    it.travelled = 0.0;
    it.state = kItemMoving;
    it.at = -1;
    Event e;
    e.type = kEventStarted;
    e.item = item;
    e.point = it.from;
    e.time = time_;
    events_.push_back(e);
    return true;
}

void Process::Step(double dt) {
    if (!(dt > 0.0)) return;
    const double t0 = time_;
    time_ += dt;
    for (size_t i = 0; i < items_.size(); i++) {
        Item& it = items_[i];
        if (it.state != kItemMoving) continue;
        const double remaining = it.distance - it.travelled;
        const double step = it.speed * dt;
        const dhVector a = points_[static_cast<size_t>(it.from)].position;
        const dhVector b = points_[static_cast<size_t>(it.to)].position;
        if (step >= remaining) {   // 이번 틱 안에 도착한다: 도착 시각은 틱의 정확한 순간
            it.travelled = it.distance;
            it.position = b;
            it.state = kItemArrived;
            it.at = it.to;
            Event e;
            e.type = kEventArrived;
            e.item = static_cast<int>(i);
            e.point = it.to;
            e.time = t0 + remaining / it.speed;
            events_.push_back(e);
        } else {
            it.travelled += step;
            const float f = static_cast<float>(it.travelled / it.distance);
            it.position = a + (b - a) * f;
        }
    }
}

bool Process::AllIdle() const {
    for (size_t i = 0; i < items_.size(); i++)
        if (items_[i].state == kItemMoving) return false;
    return true;
}

double Process::TimeToArrival(int item) const {
    if (item < 0 || static_cast<size_t>(item) >= items_.size()) return 0.0;
    const Item& it = items_[static_cast<size_t>(item)];
    if (it.state != kItemMoving) return 0.0;
    return (it.distance - it.travelled) / it.speed;
}

Array<Event> Process::TakeEvents() {
    Array<Event> out = events_;
    events_.clear();
    return out;
}

}  // namespace process
}  // namespace simul
