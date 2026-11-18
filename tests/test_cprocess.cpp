// 물류 공정(A 지점 -> B 지점) 테스트
#include <math.h>
#include <stdio.h>

#include "CProcess/Process.h"

using namespace simul;
using namespace simul::process;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol = 1e-5) { return fabs(a - b) <= tol; }

void testMoveAToB() {
    Process p;
    String err;
    check(p.AddPoint("A", dhVector(0, 0, 0)) == 0 && p.AddPoint("B", dhVector(3, 4, 0)) == 1, "two points added");
    const int box = p.AddItem("box", "A");
    check(box == 0 && p.GetItem(box).state == kItemWaiting && p.GetItem(box).at == 0, "item waits at A");
    check(p.AllIdle(), "nothing moving yet");
    check(p.Order(box, "B", 0.5, &err), "order A -> B");   // 거리 5 m, 0.5 m/s -> 10 s
    check(p.GetItem(box).state == kItemMoving && p.GetItem(box).at == -1 && near(p.GetItem(box).distance, 5.0), "item moving, 5 m to go");
    check(near(p.TimeToArrival(box), 10.0), "arrives in 10 s");
    check(!p.AllIdle(), "something is moving");
    p.Step(4.0);
    const Item& it = p.GetItem(box);
    check(near(it.position.x, 1.2) && near(it.position.y, 1.6) && near(it.travelled, 2.0), "at 40% of the way after 4 s");
    check(near(p.TimeToArrival(box), 6.0), "6 s left");
    p.Step(6.5);   // 도착을 넘겨서 진행해도 B 에서 멈춘다
    check(p.GetItem(box).state == kItemArrived && p.GetItem(box).at == 1, "arrived at B");
    check(near(p.GetItem(box).position.x, 3.0) && near(p.GetItem(box).position.y, 4.0), "item sits exactly on B");
    check(p.AllIdle() && near(p.TimeToArrival(box), 0.0), "idle after arrival");
    p.Step(1.0);
    check(near(p.GetItem(box).position.x, 3.0), "item stays at B");

    Array<Event> ev = p.TakeEvents();
    check(ev.size() == 2 && ev[0].type == kEventStarted && ev[0].point == 0 && near(ev[0].time, 0.0), "start event at A");
    check(ev[1].type == kEventArrived && ev[1].point == 1 && near(ev[1].time, 10.0), "arrival event at B at exactly t = 10 s");
    check(p.TakeEvents().empty(), "events are cleared after taking");

    check(p.Order(box, "A", 1.0, &err) && p.GetItem(box).from == 1 && p.GetItem(box).to == 0, "item can be sent back");
    for (int i = 0; i < 1000 && !p.AllIdle(); i++) p.Step(0.02);
    check(p.GetItem(box).state == kItemArrived && near(p.GetItem(box).position.x, 0.0), "returned to A with small steps");
}

void testErrors() {
    Process p;
    String err;
    check(p.AddPoint("", dhVector()) < 0, "empty point name rejected");
    check(p.AddPoint("A", dhVector()) >= 0 && p.AddPoint("A", dhVector(1, 0, 0), &err) < 0 && !err.empty(), "duplicate point rejected");
    p.AddPoint("B", dhVector(1, 0, 0));
    check(p.AddItem("x", "C", &err) < 0 && !err.empty(), "item at unknown point rejected");
    const int a = p.AddItem("x", "A");
    check(p.AddItem("x", "B", &err) < 0, "duplicate item rejected");
    check(!p.Order(7, "B", 1.0, &err) && !err.empty(), "unknown item rejected");
    check(!p.Order(a, "C", 1.0, &err), "unknown destination rejected");
    check(!p.Order(a, "B", 0.0, &err) && !p.Order(a, "B", -1.0, &err), "non-positive speed rejected");
    check(!p.Order(a, "A", 1.0, &err), "sending to the same point rejected");
    check(p.Order(a, "B", 1.0) && !p.Order(a, "B", 1.0, &err), "cannot reorder while moving");
    p.Step(0.0);
    p.Step(-1.0);
    check(near(p.Time(), 0.0), "non-positive dt is ignored");
    check(p.FindPoint("B") == 1 && p.FindItem("x") == 0 && p.FindItem("nope") < 0, "lookups");
}

void testTwoItems() {
    Process p;
    p.AddPoint("A", dhVector(0, 0, 0));
    p.AddPoint("B", dhVector(2, 0, 0));
    const int fast = p.AddItem("fast", "A");
    const int slow = p.AddItem("slow", "A");
    p.Order(fast, "B", 2.0);
    p.Order(slow, "B", 1.0);
    p.Step(1.0);
    check(p.GetItem(fast).state == kItemArrived && p.GetItem(slow).state == kItemMoving, "fast arrives first");
    check(!p.AllIdle(), "not idle while the slow one moves");
    p.Step(1.0);
    check(p.AllIdle(), "both arrived");
    Array<Event> ev = p.TakeEvents();
    int arrived = 0;
    for (size_t i = 0; i < ev.size(); i++) if (ev[i].type == kEventArrived) arrived++;
    check(arrived == 2 && ev.size() == 4, "two starts and two arrivals recorded");
}
}  // namespace

int main() {
    testMoveAToB();
    testErrors();
    testTwoItems();
    if (g_fail == 0) printf("test_cprocess: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
