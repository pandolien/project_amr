// CorePy 테스트: 내장 Python 에서 Main.py 를 올리고, 등록된 클래스가 시뮬레이션과 함께 실행되는지,
// 오브젝트별 명령 권한, 예외 격리, 읽기 전용 보기를 확인한다.
#include <math.h>
#include <stdio.h>

#include "CBase/File.h"
#include "CorePy/PyRuntime.h"
#include "FakeSimBackend.h"

using namespace simul;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

Array<String> g_out;
void Out(bool isError, const char* line, void*) {
    g_out.push_back(String(isError ? "E:" : "O:") + line);
}
bool Has(const char* text) {
    for (size_t i = 0; i < g_out.size(); i++)
        if (g_out[i].find(String(text)) != String::npos) return true;
    return false;
}

const char* kUser =
    "from Main import Behavior, register\n"
    "\n"
    "class Mover(Behavior):\n"
    "    objects = ['arm']\n"
    "    def on_start(self):\n"
    "        print('MOVER start', sorted(self.objects), sorted(self.world))\n"
    "        self.arm = self.objects['arm']\n"
    "        self.arm.move_joint('j1', 0.8)\n"
    "        try:\n"
    "            self.world['box'].move_to(1, 0, 0, 1)\n"
    "        except PermissionError as e:\n"
    "            print('PERM denied box')\n"
    "        try:\n"
    "            self.objects['arm'].set_joint('j1', 99.0)\n"
    "        except Exception as e:\n"
    "            print('LIMIT rejected', type(e).__name__)\n"
    "        self.ticks = 0\n"
    "    def on_tick(self, t, dt):\n"
    "        self.ticks += 1\n"
    "    def on_event(self, ev):\n"
    "        if ev.type == 'joint_reached':\n"
    "            print('MOVER reached', ev.name, ev.joint, round(self.world['arm'].joints['j1'], 3))\n"
    "            self.objects['arm'].stop()\n"
    "    def on_stop(self):\n"
    "        print('MOVER stop ticks', self.ticks)\n"
    "\n"
    "class Bad(Behavior):\n"
    "    def on_tick(self, t, dt):\n"
    "        raise ValueError('boom')\n"
    "\n"
    "class Carrier(Behavior):\n"
    "    def on_start(self):\n"
    "        self.objects['box'].move_to(0.3, 0, 0, 1.0)\n"
    "    def on_event(self, ev):\n"
    "        if ev.type == 'arrived':\n"
    "            print('CARRIER arrived', ev.name)\n"
    "\n"
    "register(Mover)\n"
    "register(Bad, objects=[])\n"
    "register(Carrier, objects=['box'])\n";

}  // namespace

int main() {
    FakeBackend b;
    b.add("arm", true);
    b.add("box", false);
    sim::Engine eng(&b);
    py::Runtime rt(&eng);
    rt.SetOutput(Out, 0);
    String err;
    check(rt.Initialize(&err), "initialize");
    const String data = SIMUL_TEST_DATA_DIR;
    check(rt.RunFile(data + "/Python/Main.py", &err), "Main.py loads");
    check(WriteFileText("t_py_user.py", kUser), "write user script");
    check(rt.RunSnippet("import Main; Main.register_file('t_py_user.py')", &err), "register_file");
    check(!Has("E:"), "no errors so far");

    eng.Start();
    check(Has("O:MOVER start ['arm'] ['arm', 'box']"), "registered class starts with its granted objects and a world view of all");
    check(Has("O:PERM denied box"), "no permission for objects that are not granted");
    check(Has("O:LIMIT rejected CommandError"), "joint limit rejected as CommandError");
    for (int i = 0; i < 40; i++) eng.Step(0.05);
    check(Has("O:MOVER reached arm j1 0.8"), "joint_reached event reaches the class that owns the object");
    check(Has("O:CARRIER arrived box"), "second class moved its own object");
    check(Has("E:") && Has("boom"), "exception in a class is reported");
    check(Has("Bad"), "message names the class");
    size_t bad = 0;
    for (size_t i = 0; i < g_out.size(); i++) bad += g_out[i] == "E:ValueError: boom";
    check(bad == 1, "failed class is stopped (traceback printed once)");
    check(fabs(b.objs[0].joints[0].value - 0.8) < 1e-6 && fabs(b.objs[1].pose.position.x - 0.3) < 1e-6, "the simulation state was changed by the scripts");
    eng.Stop();
    check(Has("O:MOVER stop ticks 40"), "on_stop runs, other classes kept ticking after the failure");

    // 저장 후 다시 읽기: 이전 클래스는 내려가고(on_stop) 새 클래스가 시작된다
    g_out.clear();
    check(eng.Running() == false, "stopped before reload test");
    eng.Start();
    g_out.clear();
    const char* v2 =
        "from Main import Behavior, register\n"
        "class Mover2(Behavior):\n"
        "    objects = ['arm']\n"
        "    def on_start(self): print('V2 start')\n"
        "    def on_stop(self): print('V2 stop')\n"
        "register(Mover2)\n";
    check(WriteFileText("t_py_user.py", v2), "rewrite user script");
    check(rt.RunSnippet("import Main; _r = Main.reload_file('t_py_user.py')", &err), "reload_file");
    check(Has("O:V2 start"), "reloaded class starts at once while the simulation is running");
    check(Has("O:MOVER stop ticks"), "old classes from that file were stopped");
    eng.Step(0.05);
    eng.Stop();
    check(Has("O:V2 stop"), "reloaded class stops with the simulation");

    // 대화형 한 줄
    g_out.clear();
    check(rt.RunSnippet("print(len(simul.objects()), simul.find('arm').joints)", &err), "snippet");
    check(Has("O:2 {'j1': 0.8"), "snippet output");
    check(!rt.RunSnippet("1/0", &err) && err.find(String("ZeroDivisionError")) != String::npos, "snippet error is reported");
    check(rt.RunSnippet("raise SystemExit(0)", &err) && rt.QuitRequested(), "SystemExit becomes a quit request (not an error)");
    RemoveFile("t_py_user.py");
    if (g_fail) {
        printf("test_cpy: %d failure(s)\n", g_fail);
        for (size_t i = 0; i < g_out.size(); i++) printf("  %s\n", g_out[i].c_str());
        return 1;
    }
    printf("test_cpy: all passed (Python %s)\n", py::Runtime::PythonVersion().c_str());
    return 0;
}
