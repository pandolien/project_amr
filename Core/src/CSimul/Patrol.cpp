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

bool Patrol::ParseObstacles(const String& text, String* error) {
    Array<Circle2> list;
    Lines lines(text);
    String raw;
    int no = 0;
    while (lines.Next(&raw)) {
        no++;
        const String line = Trim(raw);
        if (line.empty() || line[0] == '#') continue;
        double v[3] = {0, 0, 0};
        const char* p = line.c_str();
        for (int k = 0; k < 3; k++) {
            const char* end = p;
            if (!ParseDouble(p, &v[k], &end) || !(fabs(v[k]) < 1e9)) {
                if (error) *error = "장애물 파일 " + ToString(no) + "번째 줄: x y 반지름 숫자 세 개여야 합니다";
                return false;
            }
            p = end;
        }
        double extra = 0;
        const char* end = p;
        if (ParseDouble(p, &extra, &end) || !(v[2] > 0)) {
            if (error) *error = "장애물 파일 " + ToString(no) + "번째 줄: x y 반지름 (반지름은 0 보다 커야 함) 세 개여야 합니다";
            return false;
        }
        list.push_back(Circle2(v[0], v[1], v[2]));
    }
    fixed_ = list;
    return true;
}

bool Patrol::Active(const String& name) const {
    const int r = find(name);
    return r >= 0 && state_[static_cast<size_t>(r)].started;
}

bool Patrol::Held(const String& name) const {
    const int r = find(name);
    return r >= 0 && state_[static_cast<size_t>(r)].held;
}

bool Patrol::Hold(Engine& e, const String& name, double fx, double fy) {
    const int r = find(name);
    if (r < 0 || !state_[static_cast<size_t>(r)].started) return false;
    const Handle h = e.Find(name);
    ObjectInfo info;
    if (h.slot < 0 || !e.Info(h, &info)) return false;
    State& s = state_[static_cast<size_t>(r)];
    s.held = true;
    e.StopMotion(h);
    const double dx = fx - static_cast<double>(info.pose.position.x), dy = fy - static_cast<double>(info.pose.position.y);
    if (dx * dx + dy * dy > 1e-12) s.yaw = atan2(dx, -dy);
    e.SetPose(h, dhPose(info.pose.position, dhQuat::FromAxisAngle(static_cast<float>(s.yaw), dhVector(0, 0, 1))));
    return true;
}

bool Patrol::Release(Engine& e, const String& name) {
    const int r = find(name);
    if (r < 0 || !state_[static_cast<size_t>(r)].held) return false;
    State& s = state_[static_cast<size_t>(r)];
    s.held = false;
    const Handle h = e.Find(name);
    ObjectInfo info;
    if (h.slot < 0 || !e.Info(h, &info) || s.path.empty()) return true;
    s.lastPlan = time_;
    steer(e, h, r, info.pose.position, s.path[s.pathPos]);   // 가던 곳으로 다시
    return true;
}

int Patrol::find(const String& name) const {
    for (size_t i = 0; i < routes_.size(); i++)
        if (routes_[i].name == name) return static_cast<int>(i);
    return -1;
}

void Patrol::steer(Engine& e, Handle h, int route, const dhVector& a, const dhVector& b) {
    State& s = state_[static_cast<size_t>(route)];
    const double dx = static_cast<double>(b.x) - static_cast<double>(a.x);
    const double dy = static_cast<double>(b.y) - static_cast<double>(a.y);
    if (dx * dx + dy * dy > 1e-12) s.yaw = atan2(dx, -dy);   // 앞쪽(-Y)이 가는 방향을 본다
    e.SetPose(h, dhPose(a, dhQuat::FromAxisAngle(static_cast<float>(s.yaw), dhVector(0, 0, 1))));
    e.MoveTo(h, b, routes_[static_cast<size_t>(route)].speed);
}

// from 지점에 놓고(자세는 to 쪽을 바라봄) to 로 직선 이동을 건다.
void Patrol::go(Engine& e, Handle h, int route, int from, int to) {
    const PatrolRoute& r = routes_[static_cast<size_t>(route)];
    State& s = state_[static_cast<size_t>(route)];
    s.next = to;
    s.lastPlan = time_;   // SetPose 는 다음 틱에 적용되니, 그동안은 옛 위치로 길을 다시 짜지 않는다
    s.path.clear();
    s.path.push_back(r.points[static_cast<size_t>(to)]);
    s.pathPos = 0;
    steer(e, h, route, r.points[static_cast<size_t>(from)], r.points[static_cast<size_t>(to)]);
}

bool Patrol::begin(Engine& e, int route) {
    const Handle h = e.Find(routes_[static_cast<size_t>(route)].name);
    if (h.slot < 0) return false;
    State& s = state_[static_cast<size_t>(route)];
    s.started = true;
    s.fresh = true;
    go(e, h, route, 0, 1);
    return true;
}

void Patrol::OnStart(Engine& e) {
    for (size_t i = 0; i < routes_.size(); i++) {
        state_[i] = State();
        begin(e, static_cast<int>(i));
    }
}

// 틱마다 (0.25 초에 한 번) 앞길이 다른 주민에게 막히는지 보고, 막히면 A* 로 비켜 가는 길을 다시 짠다.
void Patrol::OnTick(Engine& e, double time, double dt) {
    time_ = time;
    if (!avoid_) return;
    const size_t n = routes_.size();
    Array<dhVector> pos(n);
    Array<char> alive(n, 0);
    Array<Handle> hs(n);
    for (size_t i = 0; i < n; i++) {
        if (!state_[i].started) continue;
        hs[i] = e.Find(routes_[i].name);
        ObjectInfo info;
        if (state_[i].fresh) {   // 방금 출발시킨 주민은 이번 틱에는 건너뛴다
            state_[i].fresh = false;
            continue;
        }
        if (hs[i].slot >= 0 && e.Info(hs[i], &info)) {
            pos[i] = info.pose.position;
            alive[i] = 1;
        }
    }
    // 1) 떼어 놓기: 서로 sep_ 보다 가까우면 번호가 작은 쪽은 한쪽으로, 큰 쪽은 반대쪽으로 민다.
    //    둘이 똑같은 자리면 방향을 못 정하므로, 번호가 작은 쪽의 가는 방향에 수직으로 가른다 (같은 입력이면 같은 결과가 되는 대칭을 깬다).
    if (sep_ > 0.0) {
        Array<char> moved(n, 0);
        const double step = 1.5 * dt;   // 한 틱에 한 명이 움직일 최대 거리
        for (size_t i = 0; i < n; i++) {
            for (size_t j = i + 1; j < n; j++) {
                if (!alive[i] || !alive[j] || state_[i].held || state_[j].held) continue;
                double dx = static_cast<double>(pos[j].x) - static_cast<double>(pos[i].x);
                double dy = static_cast<double>(pos[j].y) - static_cast<double>(pos[i].y);
                double d = hypot(dx, dy);
                if (d >= sep_) continue;
                if (d < 1e-6) {
                    const double yaw = state_[i].yaw;
                    dx = cos(yaw);
                    dy = sin(yaw);
                    d = 1.0;
                }
                double push = (sep_ - hypot(static_cast<double>(pos[j].x) - static_cast<double>(pos[i].x), static_cast<double>(pos[j].y) - static_cast<double>(pos[i].y))) * 0.5;
                if (push > step) push = step;
                const double ux = dx / d, uy = dy / d;
                pos[i].x = static_cast<float>(static_cast<double>(pos[i].x) - ux * push);
                pos[i].y = static_cast<float>(static_cast<double>(pos[i].y) - uy * push);
                pos[j].x = static_cast<float>(static_cast<double>(pos[j].x) + ux * push);
                pos[j].y = static_cast<float>(static_cast<double>(pos[j].y) + uy * push);
                moved[i] = moved[j] = 1;
            }
        }
        for (size_t i = 0; i < n; i++) {
            if (!moved[i] || state_[i].path.empty()) continue;
            steer(e, hs[i], static_cast<int>(i), pos[i], state_[i].path[state_[i].pathPos]);   // 밀린 자리에서 같은 곳으로 다시 걷기
        }
    }
    // 2) 앞길 확인: 0.25 초마다 (누가 sep 안쪽으로 붙어 있으면 0.05 초마다)
    for (size_t i = 0; i < n; i++) {
        State& s = state_[i];
        if (!alive[i] || s.path.empty() || s.held) continue;
        double nearest = 1e9;
        for (size_t k = 0; k < n; k++)
            if (k != i && alive[k]) {
                const double d = hypot(static_cast<double>(pos[k].x) - static_cast<double>(pos[i].x), static_cast<double>(pos[k].y) - static_cast<double>(pos[i].y));
                if (d < nearest) nearest = d;
            }
        if (time - s.lastPlan < (nearest < radius_ ? 0.05 : 0.25)) continue;
        s.lastPlan = time;
        const Point2 here(static_cast<double>(pos[i].x), static_cast<double>(pos[i].y));
        Array<Circle2> obs;
        for (size_t k = 0; k < n; k++) {
            if (k == i || !alive[k]) continue;
            const double ox = static_cast<double>(pos[k].x), oy = static_cast<double>(pos[k].y);
            if (hypot(ox - here.x, oy - here.y) < 8.0) obs.push_back(Circle2(ox, oy, radius_));
        }
        for (size_t k = 0; k < fixed_.size(); k++) {   // 움직이지 않는 장애물: 몸 반지름 0.4 m 를 더한다
            const Circle2& c = fixed_[k];
            if (hypot(c.x - here.x, c.y - here.y) < 8.0 + c.r) obs.push_back(Circle2(c.x, c.y, c.r + 0.4));
        }
        const dhVector goal3 = routes_[i].points[static_cast<size_t>(s.next)];
        const Point2 goal(static_cast<double>(goal3.x), static_cast<double>(goal3.y));

        // 지금 가는 길(남은 꺾임점들)이 아직 비어 있으면 그대로 간다
        bool clear = true;
        Point2 a = here;
        for (size_t k = s.pathPos; k < s.path.size() && clear; k++) {
            const Point2 b(static_cast<double>(s.path[k].x), static_cast<double>(s.path[k].y));
            clear = SegmentClear(a, b, obs);
            a = b;
        }
        const bool detouring = s.path.size() > 1;
        if (clear && !detouring) continue;
        Array<Point2> found;
        if (SegmentClear(here, goal, obs)) {
            if (!detouring) continue;
            found.push_back(goal);      // 비키던 중인데 곧장 가도 되게 됐다
        } else if (clear) {
            continue;                    // 비켜 가는 길이 아직 비어 있다
        } else {
            const bool ok = FindPath2D(here, goal, obs, 0.25, 3.0, 0.15, &found, tracing_ ? &trace_ : nullptr);
            if (tracing_) {
                traceName_ = routes_[i].name;
                traceSerial_++;
            }
            if (!ok) continue;           // 못 찾으면 가던 대로
        }
        // z 는 현재 높이에서 지점 높이까지 거리에 비례해 잇는다
        const double total = hypot(goal.x - here.x, goal.y - here.y);
        Array<dhVector> np;
        Point2 prev = here;
        double walked = 0.0;
        for (size_t k = 0; k < found.size(); k++) {
            walked += hypot(found[k].x - prev.x, found[k].y - prev.y);
            prev = found[k];
            const double t = total > 1e-9 ? (walked > total ? 1.0 : walked / total) : 1.0;
            const float z = static_cast<float>(static_cast<double>(pos[i].z) + (static_cast<double>(goal3.z) - static_cast<double>(pos[i].z)) * t);
            np.push_back(k + 1 == found.size() ? goal3 : dhVector(static_cast<float>(found[k].x), static_cast<float>(found[k].y), z));
        }
        s.path = np;
        s.pathPos = 0;
        detours_++;
        steer(e, hs[i], static_cast<int>(i), pos[i], s.path[0]);
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
        s.held = false;
    } else if (ev.type == kEventArrived && s.started && !s.held) {
        if (s.pathPos + 1 < s.path.size()) {     // 비켜 가는 중간 지점: 다음 꺾임점으로
            const dhVector from = s.path[s.pathPos];
            s.pathPos++;
            steer(e, ev.object, r, from, s.path[s.pathPos]);
            return;
        }
        s.reached++;
        const int here = s.next;
        const int next = (here + 1) % static_cast<int>(routes_[static_cast<size_t>(r)].points.size());
        go(e, ev.object, r, here, next);
    }
}

}  // namespace sim
}  // namespace simul
