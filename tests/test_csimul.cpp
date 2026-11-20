// CSimul 엔진 테스트: 등록부(핸들 세대), 명령 검증, 큐 적용 순서(lockstep), 이동/관절 움직임, 이벤트
#include <math.h>
#include <stdio.h>

#include "CSimul/Engine.h"
#include "FakeSimBackend.h"

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
bool near(double a, double b, double tol = 1e-5) { return fabs(a - b) <= tol; }

struct Rec : Listener {
    int starts, stops, ticks;
    Array<Event> events;
    double lastTime;
    Rec() : starts(0), stops(0), ticks(0), lastTime(0) {}
    void OnStart(Engine&) override { starts++; }
    void OnStop(Engine&) override { stops++; }
    void OnTick(Engine&, double t, double) override { ticks++; lastTime = t; }
    void OnEvent(Engine&, const Event& e) override { events.push_back(e); }
};

void testRegistry() {
    FakeBackend b;
    b.add("arm", true);
    b.add("box", false);
    Engine e(&b);
    check(e.Refresh() == 2 && e.Count() == 2, "two objects");
    const Handle arm = e.Find("arm"), box = e.Find("box");
    check(arm.slot >= 0 && box.slot >= 0 && arm != box, "handles found by name");
    check(e.Find("nothing").slot < 0, "unknown name");
    check(e.Name(arm) == "arm", "name of handle");
    ObjectInfo info;
    check(e.Info(arm, &info) && info.kind == "robot" && info.joints.size() == 1, "info of the robot");
    // 지우면 핸들이 죽고, 같은 이름으로 다시 만들어도 옛 핸들은 거부된다
    b.objs.erase(b.objs.begin());   // arm 삭제 (box 가 인덱스 0 이 된다)
    e.Refresh();
    check(!e.Alive(arm) && e.Alive(e.Find("box")), "removed object dies, the other one survives its index change");
    b.add("arm", true);
    const Handle arm2 = e.Find("arm");
    check(arm2.slot >= 0 && arm2 != arm && !e.Alive(arm), "re-added object gets a new generation");
    String err;
    check(!e.SetPose(arm, dhPose(), &err) && !err.empty(), "command on a stale handle is rejected");
    Array<Event> ev = e.TakeEvents();
    int added = 0, removed = 0;
    for (size_t i = 0; i < ev.size(); i++) {
        added += ev[i].type == kEventAdded;
        removed += ev[i].type == kEventRemoved;
    }
    check(added == 3 && removed == 1, "events: 3 added (arm, box, arm again), 1 removed");
}

void testCommands() {
    FakeBackend b;
    b.add("arm", true);
    Engine e(&b);
    Rec r;
    e.SetListener(&r);
    e.Start();
    check(r.starts == 1, "OnStart once");
    e.Start();
    check(r.starts == 1, "Start twice is ignored");
    const Handle arm = e.Find("arm");
    String err;
    check(!e.SetJoint(arm, "nope", 0.0, &err) && !err.empty(), "unknown joint rejected");
    check(!e.SetJoint(arm, "j1", 5.0, &err), "value above the limit rejected");
    check(!e.MoveTo(arm, dhVector(1, 0, 0), 0.0, &err), "zero speed rejected");
    check(!e.SetPose(arm, dhPose(dhVector(NAN, 0, 0), dhQuat()), &err), "NaN rejected");
    check(e.PendingCommands() == 0, "rejected commands are not queued");

    // lockstep: 큐의 명령은 다음 Step 에서 적용된다
    check(e.SetJoint(arm, "j1", 0.5, &err) && e.PendingCommands() == 1, "command queued");
    check(near(b.objs[0].joints[0].value, 0.0), "not applied before the step");
    check(e.Step(0.1) && near(b.objs[0].joints[0].value, 0.5) && e.PendingCommands() == 0, "applied at the next step");
    check(r.ticks == 1 && near(r.lastTime, 0.1) && e.TickCount() == 1, "tick delivered with time");
    check(!e.Step(0.0) && !e.Step(NAN), "bad dt rejected");

    // 같은 틱 안에서 명령은 넣은 순서대로
    e.SetJoint(arm, "j1", 0.1, &err);
    e.SetJoint(arm, "j1", -0.2, &err);
    e.Step(0.1);
    check(near(b.objs[0].joints[0].value, -0.2), "commands apply in order");
}

void testMotion() {
    FakeBackend b;
    b.add("box", false);
    b.add("arm", true);
    Engine e(&b);
    Rec r;
    e.SetListener(&r);
    e.Start();
    const Handle box = e.Find("box"), arm = e.Find("arm");
    String err;
    check(e.MoveTo(box, dhVector(1, 0, 0), 0.5, &err), "move_to queued");
    e.Step(0.5);   // 0.25 m
    check(near(b.objs[0].pose.position.x, 0.25) && e.Moving(box), "moves at speed * dt");
    e.Step(1.0);   // 남은 0.75 m 중 0.5 m
    check(near(b.objs[0].pose.position.x, 0.75), "continues");
    e.Step(1.0);   // 도착
    check(near(b.objs[0].pose.position.x, 1.0) && !e.Moving(box), "arrives exactly at the target");
    int arrived = 0;
    for (size_t i = 0; i < r.events.size(); i++) arrived += r.events[i].type == kEventArrived && r.events[i].name == "box";
    check(arrived == 1, "one arrived event");

    // 관절: 속도를 안 주면 관절의 최대 속도(2 rad/s)
    check(e.MoveJoint(arm, "j1", 1.0, 0.0, &err), "move_joint queued");
    e.Step(0.25);
    check(near(b.objs[1].joints[0].value, 0.5), "default speed is the joint velocity");
    e.StopMotion(arm);
    e.Step(0.25);
    check(near(b.objs[1].joints[0].value, 0.5) && !e.Moving(arm), "stop halts the motion");
    e.MoveJoint(arm, "j1", -1.0, 1.0);
    e.Step(1.0);
    check(near(b.objs[1].joints[0].value, -0.5), "slower explicit speed (1 rad/s)");
    e.Step(1.0);
    int reached = 0;
    for (size_t i = 0; i < r.events.size(); i++) reached += r.events[i].type == kEventJointReached && r.events[i].joint == "j1";
    check(near(b.objs[1].joints[0].value, -1.0) && reached == 1, "joint reached event");
    // set_joint 는 그 관절의 움직임을 멈춘다
    e.MoveJoint(arm, "j1", 1.0, 0.1);
    e.Step(0.1);
    e.SetJoint(arm, "j1", 0.0);
    e.Step(0.1);
    check(near(b.objs[1].joints[0].value, 0.0) && !e.Moving(arm), "set_joint cancels the motion");
    // 움직이는 중에 사라지면 정리된다
    e.MoveTo(box, dhVector(9, 9, 9), 1.0);
    e.Step(0.1);
    b.objs.erase(b.objs.begin());
    e.Step(0.1);
    check(!e.Alive(box), "removed while moving");
    e.Stop();
    check(r.stops == 1 && !e.Running(), "OnStop once");
}

void testFailure() {
    FakeBackend b;
    b.add("arm", true);
    Engine e(&b);
    Rec r;
    e.SetListener(&r);
    const Handle arm = e.Find("arm");
    e.SetJoint(arm, "j1", 0.2);
    b.objs.erase(b.objs.begin());   // 적용 전에 사라진다
    e.Step(0.1);
    int failed = 0;
    for (size_t i = 0; i < r.events.size(); i++) failed += r.events[i].type == kEventCommandFailed;
    check(failed == 1, "command on an object that vanished before the step reports command_failed");
    check(r.ticks == 1, "tick still runs");
}

}  // namespace

int main() {
    testRegistry();
    testCommands();
    testMotion();
    testFailure();
    if (g_fail) {
        printf("test_csimul: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_csimul: all passed\n");
    return 0;
}
