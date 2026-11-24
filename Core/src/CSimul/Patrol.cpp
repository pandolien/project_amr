#include "CSimul/Patrol.h"

#include <math.h>

#include "3d/dhPose.h"
#include "3d/dhQuat.h"
#include "CBase/Text.h"

namespace simul {
namespace sim {

bool Patrol::Parse(const String& text, String* error) {
    Array<PatrolRoute> routes;
    Lines lines(text);
    String raw;
    int no = 0;
    while (lines.Next(&raw)) {
        no++;
        const String line = Trim(raw);
        if (line.empty() || line[0] == '#') continue;
        const String where = "지점 파일 " + ToString(no) + "번째 줄: ";
        if (line[0] == '[') {
            const size_t close = line.find(']');
            if (close == String::npos || close < 2) {
                if (error) *error = where + "[이름] 형식이 아닙니다";
                return false;
            }
            PatrolRoute r;
            r.name = Trim(line.substr(1, close - 1));
            routes.push_back(r);
            continue;
        }
        if (routes.empty()) {
            if (error) *error = where + "[이름] 보다 먼저 지점이 나왔습니다";
            return false;
        }
        PatrolRoute& r = routes[routes.size() - 1];
        if (StartsWith(line, "speed")) {
            double v = 0;
            if (!ToDouble(line.substr(5), &v) || !(v > 0) || !(v < 1e6)) {
                if (error) *error = where + "speed 는 0 보다 큰 숫자여야 합니다";
                return false;
            }
            r.speed = v;
            continue;
        }
        double xyz[3] = {0, 0, 0};
        const char* p = line.c_str();
        for (int k = 0; k < 3; k++) {
            const char* end = p;
            if (!ParseDouble(p, &xyz[k], &end) || !(fabs(xyz[k]) < 1e9)) {
                if (error) *error = where + "지점은 x y z 숫자 세 개여야 합니다";
                return false;
            }
            p = end;
        }
        double extra = 0;
        const char* end = p;
        if (ParseDouble(p, &extra, &end)) {
            if (error) *error = where + "지점은 x y z 숫자 세 개여야 합니다";
            return false;
        }
        r.points.push_back(dhVector(static_cast<float>(xyz[0]), static_cast<float>(xyz[1]), static_cast<float>(xyz[2])));
    }
    for (size_t i = 0; i < routes.size(); i++) {
        if (routes[i].name.empty()) {
            if (error) *error = "이름이 빈 [ ] 가 있습니다";
            return false;
        }
        if (routes[i].points.size() < 2) {
            if (error) *error = "[" + routes[i].name + "] 은 지점이 2개 이상 필요합니다";
            return false;
        }
    }
    routes_ = routes;
    state_.clear();
    for (size_t i = 0; i < routes_.size(); i++) state_.push_back(State());
    return true;
}

int Patrol::find(const String& name) const {
    for (size_t i = 0; i < routes_.size(); i++)
        if (routes_[i].name == name) return static_cast<int>(i);
    return -1;
}

// from 지점에 놓고(자세는 to 쪽을 바라봄) to 로 직선 이동을 건다.
void Patrol::go(Engine& e, Handle h, int route, int from, int to) {
    const PatrolRoute& r = routes_[static_cast<size_t>(route)];
    State& s = state_[static_cast<size_t>(route)];
    const dhVector a = r.points[static_cast<size_t>(from)];
    const dhVector b = r.points[static_cast<size_t>(to)];
    const double dx = static_cast<double>(b.x) - static_cast<double>(a.x);
    const double dy = static_cast<double>(b.y) - static_cast<double>(a.y);
    if (dx * dx + dy * dy > 1e-12) s.yaw = atan2(dx, -dy);   // 앞쪽(-Y)이 가는 방향을 본다
    e.SetPose(h, dhPose(a, dhQuat::FromAxisAngle(static_cast<float>(s.yaw), dhVector(0, 0, 1))));
    e.MoveTo(h, b, r.speed);
    s.next = to;
}

bool Patrol::begin(Engine& e, int route) {
    const Handle h = e.Find(routes_[static_cast<size_t>(route)].name);
    if (h.slot < 0) return false;
    State& s = state_[static_cast<size_t>(route)];
    s.started = true;
    go(e, h, route, 0, 1);
    return true;
}

void Patrol::OnStart(Engine& e) {
    for (size_t i = 0; i < routes_.size(); i++) {
        state_[i] = State();
        begin(e, static_cast<int>(i));
    }
}

void Patrol::OnEvent(Engine& e, const Event& ev) {
    const int r = find(ev.name);
    if (r < 0) return;
    State& s = state_[static_cast<size_t>(r)];
    if (ev.type == kEventAdded) {                 // 늦게 올라온 모델도 걷기 시작한다
        if (!s.started && e.Running()) begin(e, r);
    } else if (ev.type == kEventRemoved) {
        s.started = false;
    } else if (ev.type == kEventArrived && s.started) {
        s.reached++;
        const int here = s.next;
        const int next = (here + 1) % static_cast<int>(routes_[static_cast<size_t>(r)].points.size());
        go(e, ev.object, r, here, next);
    }
}

}  // namespace sim
}  // namespace simul
