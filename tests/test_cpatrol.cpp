// CSimul 순찰 테스트: 지점 파일 읽기와 오류, 차례로 걷기, 끝에서 처음으로 돌아오기, 바라보는 방향, 늦게 올라온 모델
#include <math.h>
#include <stdio.h>

#include "CSimul/Patrol.h"
#include "FakeSimBackend.h"
#include "TestUtil.h"

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
bool near(double a, double b, double tol = 1e-3) { return fabs(a - b) <= tol; }

const char* kRoutes =
    "# 주석\n"
    "[walker]\n"
    "speed 1\n"
    "0 0 0\n"
    "2 0 0\n"
    "2 2 0\n"
    "\n"
    "[late]\n"
    "speed 2\n"
    "5 5 1\n"
    "5 5 3\r\n";

dhVector posOf(Engine& e, const char* name) {
    ObjectInfo info;
    e.Info(e.Find(name), &info);
    return info.pose.position;
}

void stepFor(Engine& e, double seconds) {
    const double dt = 0.01;
    for (double t = 0; t < seconds - 1e-9; t += dt) e.Step(dt);
}

void testParse() {
    Patrol p;
    String err;
    check(p.Parse(kRoutes, &err), "정상 파일을 읽는다");
    check(p.RouteCount() == 2 && p.Route(0).name == "walker" && p.Route(0).points.size() == 3, "길 2개, 첫 길의 지점 3개");
    check(near(p.Route(1).speed, 2.0) && p.Route(1).points.size() == 2, "speed 와 지점 수");
    check(near(p.Route(0).points[2].y, 2.0), "지점 값");

    Patrol bad;
    check(!bad.Parse("1 2 3\n", &err) && err.find("보다 먼저") != String::npos, "[이름] 없이 지점이 나오면 오류");
    check(!bad.Parse("[a]\n0 0 0\n", &err) && err.find("2개 이상") != String::npos, "지점이 1개면 오류");
    check(!bad.Parse("[a]\n0 0\n1 1 1\n", &err) && err.find("2번째 줄") != String::npos, "숫자가 모자라면 줄 번호와 함께 오류");
    check(!bad.Parse("[a]\n0 0 0 9\n1 1 1\n", &err), "숫자가 넘치면 오류");
    check(!bad.Parse("[a]\nspeed -1\n0 0 0\n1 1 1\n", &err), "speed 가 0 이하면 오류");
    check(!bad.Parse("[]\n0 0 0\n1 1 1\n", &err), "이름이 비면 오류");
    check(bad.RouteCount() == 0, "실패하면 아무것도 남기지 않는다");
    check(p.Parse(kRoutes, &err) && !p.Parse("[a]\n", &err) && p.RouteCount() == 2, "실패하면 이전 내용을 유지");
}

void testWalk() {
    FakeBackend b;
    b.add("walker", false);
    b.add("other", false);   // 길이 없는 오브젝트는 건드리지 않는다
    Patrol p;
    String err;
    check(p.Parse(kRoutes, &err), "parse");
    Engine e(&b);
    e.SetListener(&p);
    e.Start();

    stepFor(e, 1.0);
    dhVector v = posOf(e, "walker");
    check(near(v.x, 1.0, 0.05) && near(v.y, 0.0, 0.05), "1초 후: 첫 구간의 절반 (1, 0)");
    ObjectInfo info;
    e.Info(e.Find("walker"), &info);
    check(near(info.pose.orientation.z, sin(tu::kPi / 4), 0.01) && near(info.pose.orientation.w, cos(tu::kPi / 4), 0.01), "+X 로 가면 앞쪽(-Y)이 +X 를 본다 (yaw 90도)");

    stepFor(e, 2.0);   // 3초: (2, 0) 을 지나 (2, 1)
    v = posOf(e, "walker");
    check(near(v.x, 2.0, 0.05) && near(v.y, 1.0, 0.05), "3초 후: 두 번째 구간 (2, 1)");
    e.Info(e.Find("walker"), &info);
    check(near(info.pose.orientation.z, 1.0, 0.01) && near(info.pose.orientation.w, 0.0, 0.01), "+Y 로 가면 yaw 180도");

    stepFor(e, 3.9);   // 6.9초: 마지막 지점 (2,2) 에서 처음 (0,0) 으로 돌아가는 길 (2.83 m) 을 끝낸다
    v = posOf(e, "walker");
    check(near(v.x, 0.0, 0.06) && near(v.y, 0.0, 0.06), "끝에 닿으면 처음으로 돌아간다");
    check(p.Arrivals(0) == 3, "세 구간 도착");

    stepFor(e, 1.0);
    v = posOf(e, "walker");
    check(near(v.x, 1.0, 0.06), "돌아온 뒤 다시 첫 구간으로 간다 (순환)");
    check(near(posOf(e, "other").x, 0.0) && near(posOf(e, "other").y, 0.0), "길이 없는 오브젝트는 그대로");
}

void testLateAndRemoved() {
    FakeBackend b;
    b.add("walker", false);
    Patrol p;
    String err;
    check(p.Parse(kRoutes, &err), "parse");
    Engine e(&b);
    e.SetListener(&p);
    e.Start();
    stepFor(e, 0.5);
    b.add("late", false);           // 시작한 뒤에 올라온 모델
    stepFor(e, 1.0);
    dhVector v = posOf(e, "late");
    check(near(v.x, 5.0) && near(v.y, 5.0) && v.z > 1.5 && v.z < 3.0, "늦게 올라온 모델도 첫 지점에서 출발해 위로 걷는다 (z 1~3)");
    check(near(posOf(e, "walker").y, 0.0, 0.6) || posOf(e, "walker").x > 1.0, "원래 있던 모델은 계속 걷는다");
}


// 마주 오는 둘, 앞질러 가는 둘: 피하기를 켜면 가까워져도 0.75 m 이상 떨어지고, 끄면 겹친다
double minGap(bool avoid, const char* routes, double seconds, int* detours) {
    FakeBackend b;
    b.add("a", false);
    b.add("b", false);
    Patrol p;
    String err;
    if (!p.Parse(routes, &err)) return -1;
    p.SetAvoidance(avoid);
    Engine e(&b);
    e.SetListener(&p);
    e.Start();
    double best = 1e9;
    for (double t = 0; t < seconds; t += 0.01) {
        e.Step(0.01);
        const dhVector pa = posOf(e, "a"), pb = posOf(e, "b");
        const double d = hypot(static_cast<double>(pa.x - pb.x), static_cast<double>(pa.y - pb.y));
        if (d < best) best = d;
    }
    if (detours) *detours = p.Detours();
    return best;
}

void testAvoid() {
    const char* headOn = "[a]\nspeed 1\n0 0 0\n12 0 0\n[b]\nspeed 1\n12 0 0\n0 0 0\n";
    int d = 0;
    const double off = minGap(false, headOn, 10.0, &d);
    check(off < 0.2 && d == 0, "끄면 마주 오는 둘이 겹친다 (길을 새로 짜지도 않는다)");
    const double on = minGap(true, headOn, 14.0, &d);
    check(on > 0.75, "켜면 마주 오는 둘이 0.75 m 이상 떨어져 엇갈린다");
    check(d > 0, "길을 새로 짠 기록이 있다");

    const char* overtake = "[a]\nspeed 2\n0 0 0\n20 0 0\n[b]\nspeed 0.5\n3 0 0\n20 0 0\n";
    const double over = minGap(true, overtake, 12.0, &d);
    check(over > 0.75, "빠른 쪽이 느린 쪽을 앞지를 때도 0.75 m 이상");

    // 끝까지 닿는다: 비키고 나서 원래 길로 돌아와 목표에 도착
    FakeBackend b;
    b.add("a", false);
    b.add("b", false);
    Patrol p;
    String err;
    p.Parse(headOn, &err);
    Engine e(&b);
    e.SetListener(&p);
    e.Start();
    stepFor(e, 16.0);
    check(p.Arrivals(0) >= 1 && p.Arrivals(1) >= 1, "비켜 간 뒤에도 둘 다 목표에 도착한다");
}

void testObstacles() {
    Patrol p;
    String err;
    check(p.ParseObstacles("# 주석\n5 0 1\n\n7.5 -3 0.5\r\n", &err) && p.ObstacleCount() == 2 && near(p.Obstacle(1).r, 0.5), "장애물 파일을 읽는다");
    check(!p.ParseObstacles("5 0\n", &err) && err.find("1번째 줄") != String::npos, "숫자가 모자라면 줄 번호와 함께 오류");
    check(!p.ParseObstacles("5 0 0\n", &err), "반지름이 0 이면 오류");
    check(!p.ParseObstacles("5 0 1 9\n", &err), "숫자가 넘치면 오류");
    check(p.ObstacleCount() == 2, "실패하면 이전 내용을 유지");

    // 길 한가운데 나무: 곧장 가면 뚫고 지나가고, 장애물을 주면 나무 둘레 (반지름 + 0.4) 밖으로 돌아간다
    FakeBackend b;
    b.add("a", false);
    Patrol q;
    q.Parse("[a]\nspeed 1\n0 0 0\n12 0 0\n", &err);
    q.ParseObstacles("6 0 1\n", &err);
    Engine e(&b);
    e.SetListener(&q);
    e.Start();
    double best = 1e9;
    for (double t = 0; t < 14.0; t += 0.01) {
        e.Step(0.01);
        const dhVector v = posOf(e, "a");
        const double d = hypot(static_cast<double>(v.x) - 6.0, static_cast<double>(v.y));
        if (d < best) best = d;
    }
    check(best > 1.3, "나무 둘레(1.4 m) 안으로 거의 들어가지 않는다");
    check(q.Arrivals(0) >= 1, "돌아가서도 목표에 도착한다");
}

// 겹치거나 붙어서 출발한 둘: 0.2 초 안에 0.6 m 이상 벌어지고, 그 뒤로는 그 밑으로 다시 좁혀지지 않는다
void gapAfterSeparation(const char* routes, double* tSep, double* minAfter, int* arrivedA, int* arrivedB) {
    FakeBackend b;
    b.add("a", false);
    b.add("b", false);
    Patrol p;
    String err;
    p.Parse(routes, &err);
    Engine e(&b);
    e.SetListener(&p);
    e.Start();
    *tSep = -1;
    *minAfter = 1e9;
    for (double t = 0; t < 14.0; t += 0.01) {
        e.Step(0.01);
        const dhVector pa = posOf(e, "a"), pb = posOf(e, "b");
        const double d = hypot(static_cast<double>(pa.x - pb.x), static_cast<double>(pa.y - pb.y));
        if (*tSep < 0 && d >= 0.6) *tSep = t;
        if (*tSep >= 0 && d < *minAfter) *minAfter = d;
    }
    *arrivedA = p.Arrivals(0);
    *arrivedB = p.Arrivals(1);
}

void testSeparation() {
    double t, m;
    int a, b;
    gapAfterSeparation("[a]\nspeed 1\n0 0 0\n10 0 0\n[b]\nspeed 1\n0 0 0\n10 0 0\n", &t, &m, &a, &b);
    check(t >= 0 && t < 0.3 && m > 0.55 && a >= 1 && b >= 1, "정확히 같은 자리에서 출발해도 갈라져서 둘 다 도착한다");
    gapAfterSeparation("[a]\nspeed 1\n0 0 0\n10 0 0\n[b]\nspeed 1\n0.3 0 0\n-10 0 0\n", &t, &m, &a, &b);
    check(t >= 0 && t < 0.3 && m > 0.55 && a >= 1 && b >= 1, "붙어서 마주 보고 출발해도 통과하지 않고 엇갈린다");
    gapAfterSeparation("[a]\nspeed 1\n0 0 0\n10 0 0\n[b]\nspeed 1\n0.3 0.1 0\n10 0 0\n", &t, &m, &a, &b);
    check(t >= 0 && t < 0.3 && m > 0.55 && a >= 1 && b >= 1, "붙어서 같은 목표로 가도 갈라진다");

    FakeBackend fb;   // 끄면 그대로 겹쳐 걷는다
    fb.add("a", false);
    fb.add("b", false);
    Patrol off;
    String err;
    off.Parse("[a]\nspeed 1\n0 0 0\n10 0 0\n[b]\nspeed 1\n0 0 0\n10 0 0\n", &err);
    off.SetSeparation(0.0);
    off.SetAvoidance(false);
    Engine e(&fb);
    e.SetListener(&off);
    e.Start();
    stepFor(e, 3.0);
    const dhVector pa = posOf(e, "a"), pb = posOf(e, "b");
    check(hypot(static_cast<double>(pa.x - pb.x), static_cast<double>(pa.y - pb.y)) < 0.01, "피하기를 끄면 겹친 채로 걷는다");
}

}  // namespace

int main() {
    testParse();
    testAvoid();
    testObstacles();
    testSeparation();
    testWalk();
    testLateAndRemoved();
    if (g_fail) {
        printf("test_cpatrol: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_cpatrol: all passed\n");
    return 0;
}
