#include "CorePy/PyRuntime.h"

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <math.h>
#include <string.h>

#include "CBase/File.h"

namespace simul {
namespace py {

namespace {

const int kStart = 0, kTick = 1, kEvent = 2, kStop = 3;

}  // namespace

struct Runtime::Impl {
    String lineBuf[2];
};

namespace {

Runtime* g_rt = nullptr;
PyTypeObject* g_objType = nullptr;
Array<PyObject*> g_cb[4];   // 콜백: start, tick, event, stop
PyObject* g_errType = nullptr;

struct ObjectProxy {
    PyObject_HEAD
    int slot;
    int gen;
};

sim::Handle HandleOf(PyObject* o) {
    ObjectProxy* p = reinterpret_cast<ObjectProxy*>(o);
    return sim::Handle(p->slot, p->gen);
}

PyObject* Str(const String& s) { return PyUnicode_DecodeUTF8(s.c_str(), static_cast<Py_ssize_t>(s.size()), "replace"); }

PyObject* NewObject(sim::Handle h) {
    ObjectProxy* p = PyObject_New(ObjectProxy, g_objType);
    if (!p) return nullptr;
    p->slot = h.slot;
    p->gen = h.gen;
    return reinterpret_cast<PyObject*>(p);
}

PyObject* Fail(const String& why) {
    PyErr_SetString(g_errType, why.c_str());
    return nullptr;
}

sim::Engine* Eng() { return g_rt->engine(); }

bool GetInfo(PyObject* self, sim::ObjectInfo* info) {
    if (!Eng()->Info(HandleOf(self), info)) {
        PyErr_SetString(g_errType, "오브젝트가 없습니다 (사라졌거나 잘못된 핸들)");
        return false;
    }
    return true;
}

PyObject* Tuple3(double a, double b, double c) { return Py_BuildValue("(ddd)", a, b, c); }

// ---- Object 형식 ----

PyObject* Obj_name(PyObject* self, void*) {
    String n = Eng()->Name(HandleOf(self));
    return Str(n);
}
PyObject* Obj_alive(PyObject* self, void*) { return PyBool_FromLong(Eng()->Alive(HandleOf(self)) ? 1 : 0); }
PyObject* Obj_kind(PyObject* self, void*) {
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    return Str(i.kind);
}
PyObject* Obj_position(PyObject* self, void*) {
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    return Tuple3(i.pose.position.x, i.pose.position.y, i.pose.position.z);
}
PyObject* Obj_quaternion(PyObject* self, void*) {
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    return Py_BuildValue("(dddd)", static_cast<double>(i.pose.orientation.x), static_cast<double>(i.pose.orientation.y), static_cast<double>(i.pose.orientation.z), static_cast<double>(i.pose.orientation.w));
}
PyObject* Obj_joints(PyObject* self, void*) {
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    PyObject* d = PyDict_New();
    for (size_t k = 0; k < i.joints.size(); k++) {
        PyObject* key = Str(i.joints[k].name);
        PyObject* v = PyFloat_FromDouble(i.joints[k].value);
        PyDict_SetItem(d, key, v);
        Py_DECREF(key);
        Py_DECREF(v);
    }
    return d;
}
PyObject* Obj_moving(PyObject* self, void*) { return PyBool_FromLong(Eng()->Moving(HandleOf(self)) ? 1 : 0); }

PyObject* Obj_info(PyObject* self, PyObject*) {
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    PyObject* d = PyDict_New();
    PyObject* v;
    v = Str(i.name); PyDict_SetItemString(d, "name", v); Py_DECREF(v);
    v = Str(i.kind); PyDict_SetItemString(d, "kind", v); Py_DECREF(v);
    v = Tuple3(i.pose.position.x, i.pose.position.y, i.pose.position.z); PyDict_SetItemString(d, "position", v); Py_DECREF(v);
    v = Py_BuildValue("(dddd)", static_cast<double>(i.pose.orientation.x), static_cast<double>(i.pose.orientation.y), static_cast<double>(i.pose.orientation.z), static_cast<double>(i.pose.orientation.w));
    PyDict_SetItemString(d, "quaternion", v); Py_DECREF(v);
    PyObject* joints = PyDict_New();
    for (size_t k = 0; k < i.joints.size(); k++) {
        const sim::JointInfo& j = i.joints[k];
        const char* kind = j.kind == sim::kJointPrismaticKind ? "prismatic" : j.kind == sim::kJointContinuousKind ? "continuous" : j.kind == sim::kJointRevoluteKind ? "revolute" : "fixed";
        PyObject* jd = Py_BuildValue("{s:d,s:s,s:d,s:d,s:d,s:O}", "value", j.value, "kind", kind, "lower", j.lower, "upper", j.upper, "velocity", j.velocity, "has_limit", j.hasLimit ? Py_True : Py_False);
        PyObject* key = Str(j.name);
        PyDict_SetItem(joints, key, jd);
        Py_DECREF(key);
        Py_DECREF(jd);
    }
    PyDict_SetItemString(d, "joints", joints);
    Py_DECREF(joints);
    return d;
}

PyObject* Obj_joint(PyObject* self, PyObject* args) {
    const char* name;
    if (!PyArg_ParseTuple(args, "s", &name)) return nullptr;
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    for (size_t k = 0; k < i.joints.size(); k++)
        if (i.joints[k].name == name) return PyFloat_FromDouble(i.joints[k].value);
    PyErr_SetString(PyExc_KeyError, name);
    return nullptr;
}

PyObject* Done(bool ok, const String& err) {
    if (!ok) return Fail(err);
    Py_RETURN_NONE;
}

PyObject* Obj_set_pose(PyObject* self, PyObject* args) {
    double v[7];
    if (!PyArg_ParseTuple(args, "ddddddd", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6])) return nullptr;
    dhPose p;
    if (!dhPose::FromArray(v, &p)) return Fail("자세 값이 올바르지 않습니다 (쿼터니언이 0 이거나 유한하지 않음)");
    String err;
    return Done(Eng()->SetPose(HandleOf(self), p, &err), err);
}

PyObject* Obj_set_position(PyObject* self, PyObject* args) {
    double x, y, z;
    if (!PyArg_ParseTuple(args, "ddd", &x, &y, &z)) return nullptr;
    sim::ObjectInfo i;
    if (!GetInfo(self, &i)) return nullptr;
    dhPose p = i.pose;
    p.position = dhVector(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    String err;
    return Done(Eng()->SetPose(HandleOf(self), p, &err), err);
}

PyObject* Obj_set_joint(PyObject* self, PyObject* args) {
    const char* name;
    double v;
    if (!PyArg_ParseTuple(args, "sd", &name, &v)) return nullptr;
    String err;
    return Done(Eng()->SetJoint(HandleOf(self), name, v, &err), err);
}

PyObject* Obj_move_to(PyObject* self, PyObject* args) {
    double x, y, z, speed;
    if (!PyArg_ParseTuple(args, "dddd", &x, &y, &z, &speed)) return nullptr;
    String err;
    return Done(Eng()->MoveTo(HandleOf(self), dhVector(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)), speed, &err), err);
}

PyObject* Obj_move_joint(PyObject* self, PyObject* args) {
    const char* name;
    double target, speed = 0.0;
    if (!PyArg_ParseTuple(args, "sd|d", &name, &target, &speed)) return nullptr;
    String err;
    return Done(Eng()->MoveJoint(HandleOf(self), name, target, speed, &err), err);
}

PyObject* Obj_stop(PyObject* self, PyObject*) {
    String err;
    return Done(Eng()->StopMotion(HandleOf(self), &err), err);
}

PyObject* Obj_command(PyObject* self, PyObject* args) {
    const Py_ssize_t n = PyTuple_GET_SIZE(args);
    if (n < 1) {
        PyErr_SetString(PyExc_TypeError, "command(이름, 인자...) 가 필요합니다");
        return nullptr;
    }
    PyObject* name = PyTuple_GET_ITEM(args, 0);
    const char* s = PyUnicode_Check(name) ? PyUnicode_AsUTF8(name) : nullptr;
    if (!s) {
        PyErr_SetString(PyExc_TypeError, "명령 이름은 문자열이어야 합니다");
        return nullptr;
    }
    static const char* const allowed[] = {"set_pose", "set_position", "set_joint", "move_to", "move_joint", "stop"};
    bool ok = false;
    for (size_t k = 0; k < sizeof(allowed) / sizeof(allowed[0]); k++) ok = ok || strcmp(s, allowed[k]) == 0;
    if (!ok) return Fail(String("알 수 없는 명령입니다: ") + s + " (set_pose, set_position, set_joint, move_to, move_joint, stop)");
    PyObject* fn = PyObject_GetAttrString(self, s);
    if (!fn) return nullptr;
    PyObject* rest = PyTuple_GetSlice(args, 1, n);
    PyObject* r = PyObject_CallObject(fn, rest);
    Py_DECREF(fn);
    Py_DECREF(rest);
    return r;
}

PyObject* Obj_repr(PyObject* self) {
    const sim::Handle h = HandleOf(self);
    String n = Eng()->Name(h);
    String s = Eng()->Alive(h) ? String("<simul.Object '") + n + "'>" : String("<simul.Object (사라짐)>");
    return Str(s);
}

PyObject* Obj_richcompare(PyObject* a, PyObject* b, int op) {
    if ((op != Py_EQ && op != Py_NE) || !PyObject_TypeCheck(b, g_objType)) Py_RETURN_NOTIMPLEMENTED;
    const bool eq = HandleOf(a) == HandleOf(b);
    return PyBool_FromLong((op == Py_EQ) == eq ? 1 : 0);
}

Py_hash_t Obj_hash(PyObject* self) {
    const sim::Handle h = HandleOf(self);
    return static_cast<Py_hash_t>(h.slot * 1000003 + h.gen);
}

PyObject* Obj_new(PyTypeObject*, PyObject*, PyObject*) {
    PyErr_SetString(PyExc_TypeError, "simul.Object 는 직접 만들 수 없습니다 (simul.find / simul.objects 를 쓰세요)");
    return nullptr;
}

void Obj_dealloc(PyObject* self) {
    PyTypeObject* tp = Py_TYPE(self);
    tp->tp_free(self);
    Py_DECREF(tp);
}

PyGetSetDef g_getset[] = {
    {"name", Obj_name, nullptr, "오브젝트 이름", nullptr},
    {"alive", Obj_alive, nullptr, "아직 있는가 (지워지면 False)", nullptr},
    {"kind", Obj_kind, nullptr, "robot 또는 model", nullptr},
    {"position", Obj_position, nullptr, "(x, y, z) m", nullptr},
    {"quaternion", Obj_quaternion, nullptr, "(x, y, z, w)", nullptr},
    {"joints", Obj_joints, nullptr, "{관절 이름: 값} (rad 또는 m)", nullptr},
    {"moving", Obj_moving, nullptr, "move_to / move_joint 진행 중인가", nullptr},
    {nullptr, nullptr, nullptr, nullptr, nullptr}};

PyMethodDef g_methods[] = {
    {"info", Obj_info, METH_NOARGS, "이름, 종류, 자세, 관절(값/한계/최대속도)을 dict 로"},
    {"joint", Obj_joint, METH_VARARGS, "joint(이름) -> 값"},
    {"set_pose", Obj_set_pose, METH_VARARGS, "set_pose(x, y, z, qx, qy, qz, qw): 다음 틱에 자세를 지정"},
    {"set_position", Obj_set_position, METH_VARARGS, "set_position(x, y, z): 방향은 그대로 위치만 지정"},
    {"set_joint", Obj_set_joint, METH_VARARGS, "set_joint(이름, 값): 다음 틱에 관절 값을 지정"},
    {"move_to", Obj_move_to, METH_VARARGS, "move_to(x, y, z, 속도 m/s): 직선 이동. 닿으면 arrived 이벤트"},
    {"move_joint", Obj_move_joint, METH_VARARGS, "move_joint(이름, 목표 [, 속도]): 속도를 주지 않으면 관절 최대 속도. 닿으면 joint_reached 이벤트"},
    {"stop", Obj_stop, METH_NOARGS, "이동과 관절 움직임을 멈춘다"},
    {"command", Obj_command, METH_VARARGS, "command(이름, 인자...): 위 명령을 이름으로 부른다"},
    {nullptr, nullptr, 0, nullptr}};

PyType_Slot g_slots[] = {
    {Py_tp_getset, g_getset}, {Py_tp_methods, g_methods}, {Py_tp_repr, reinterpret_cast<void*>(Obj_repr)},
    {Py_tp_richcompare, reinterpret_cast<void*>(Obj_richcompare)}, {Py_tp_hash, reinterpret_cast<void*>(Obj_hash)},
    {Py_tp_new, reinterpret_cast<void*>(Obj_new)}, {Py_tp_dealloc, reinterpret_cast<void*>(Obj_dealloc)},
    {0, nullptr}};

PyType_Spec g_spec = {"simul.Object", sizeof(ObjectProxy), 0, Py_TPFLAGS_DEFAULT, g_slots};

// ---- 모듈 함수 ----

PyObject* M_time(PyObject*, PyObject*) { return PyFloat_FromDouble(Eng()->Time()); }
PyObject* M_dt(PyObject*, PyObject*) { return PyFloat_FromDouble(Eng()->LastDt()); }
PyObject* M_tick(PyObject*, PyObject*) { return PyLong_FromUnsignedLongLong(Eng()->TickCount()); }
PyObject* M_running(PyObject*, PyObject*) { return PyBool_FromLong(Eng()->Running() ? 1 : 0); }

PyObject* M_collisions(PyObject*, PyObject*) {
    simul::Array<sim::ContactInfo> list;
    Eng()->Contacts(&list);
    PyObject* l = PyList_New(0);
    for (size_t i = 0; i < list.size(); i++) {
        const sim::ContactInfo& c = list[i];
        PyObject* d = Py_BuildValue("{s:s,s:s,s:s,s:s,s:O}", "a", c.objectA.c_str(), "link_a", c.linkA.c_str(), "b", c.objectB.c_str(), "link_b", c.linkB.c_str(), "same", c.sameObject ? Py_True : Py_False);
        if (!d) {
            Py_DECREF(l);
            return nullptr;
        }
        PyList_Append(l, d);
        Py_DECREF(d);
    }
    return l;
}

PyObject* M_objects(PyObject*, PyObject*) {
    Eng()->Refresh();
    PyObject* l = PyList_New(0);
    for (int i = 0; i < Eng()->Count(); i++) {
        PyObject* o = NewObject(Eng()->HandleAt(i));
        if (!o) {
            Py_DECREF(l);
            return nullptr;
        }
        PyList_Append(l, o);
        Py_DECREF(o);
    }
    return l;
}

PyObject* M_find(PyObject*, PyObject* args) {
    const char* name;
    if (!PyArg_ParseTuple(args, "s", &name)) return nullptr;
    const sim::Handle h = Eng()->Find(name);
    if (h.slot < 0) Py_RETURN_NONE;
    return NewObject(h);
}


PyObject* AddCallback(PyObject* args, int kind) {
    PyObject* fn;
    if (!PyArg_ParseTuple(args, "O", &fn)) return nullptr;
    if (!PyCallable_Check(fn)) {
        PyErr_SetString(PyExc_TypeError, "호출할 수 있는 것(함수)이어야 합니다");
        return nullptr;
    }
    Py_INCREF(fn);
    g_cb[kind].push_back(fn);
    Py_INCREF(fn);
    return fn;   // 데코레이터로 쓸 수 있게 그대로 돌려준다
}
PyObject* M_on_start(PyObject*, PyObject* a) { return AddCallback(a, kStart); }
PyObject* M_on_tick(PyObject*, PyObject* a) { return AddCallback(a, kTick); }
PyObject* M_on_event(PyObject*, PyObject* a) { return AddCallback(a, kEvent); }
PyObject* M_on_stop(PyObject*, PyObject* a) { return AddCallback(a, kStop); }

PyObject* M_clear_callbacks(PyObject*, PyObject*) {
    for (int k = 0; k < 4; k++) {
        for (size_t i = 0; i < g_cb[k].size(); i++) Py_DECREF(g_cb[k][i]);
        g_cb[k].clear();
    }
    Py_RETURN_NONE;
}

PyObject* M_quit(PyObject*, PyObject*) {
    g_rt->RequestQuit();
    Py_RETURN_NONE;
}

PyObject* M_write(PyObject*, PyObject* args) {
    int isErr;
    const char* text;
    if (!PyArg_ParseTuple(args, "is", &isErr, &text)) return nullptr;
    g_rt->Emit(isErr != 0, text);
    Py_RETURN_NONE;
}

PyObject* M_step(PyObject*, PyObject* args) {
    double dt;
    if (!PyArg_ParseTuple(args, "d", &dt)) return nullptr;
    PyErr_SetString(g_errType, "step 은 스크립트에서 부를 수 없습니다 (루프는 호스트가 돌립니다)");
    return nullptr;
}

PyMethodDef g_modMethods[] = {
    {"time", M_time, METH_NOARGS, "시뮬레이션 시계 (s)"},
    {"dt", M_dt, METH_NOARGS, "마지막 틱의 dt (s)"},
    {"tick", M_tick, METH_NOARGS, "틱 번호"},
    {"running", M_running, METH_NOARGS, "시뮬레이션이 돌고 있는가"},
    {"objects", M_objects, METH_NOARGS, "지금 있는 모든 오브젝트 [Object]"},
    {"collisions", M_collisions, METH_NOARGS, "지금 겹친 충돌 쌍 [{a, link_a, b, link_b, same}]"},
    {"find", M_find, METH_VARARGS, "find(이름) -> Object 또는 None"},
    {"on_start", M_on_start, METH_VARARGS, "on_start(fn): 시작할 때 fn()"},
    {"on_tick", M_on_tick, METH_VARARGS, "on_tick(fn): 매 틱 fn(t, dt)"},
    {"on_event", M_on_event, METH_VARARGS, "on_event(fn): 이벤트마다 fn(dict)"},
    {"on_stop", M_on_stop, METH_VARARGS, "on_stop(fn): 멈출 때 fn()"},
    {"clear_callbacks", M_clear_callbacks, METH_NOARGS, "등록한 콜백을 모두 지운다"},
    {"quit", M_quit, METH_NOARGS, "프로그램 종료를 요청"},
    {"_write", M_write, METH_VARARGS, "내부용: 출력"},
    {"_step", M_step, METH_VARARGS, "내부용"},
    {nullptr, nullptr, 0, nullptr}};

PyModuleDef g_modDef = {PyModuleDef_HEAD_INIT, "simul", "Project Simul: 엔진과 통신하는 저수준 모듈 (Data/Python/main.py 가 이 위에서 동작)", -1, g_modMethods, nullptr, nullptr, nullptr, nullptr};

PyObject* PyInit_simul() {
    PyObject* m = PyModule_Create(&g_modDef);
    if (!m) return nullptr;
    g_errType = PyErr_NewException("simul.CommandError", PyExc_RuntimeError, nullptr);
    Py_INCREF(g_errType);
    PyModule_AddObject(m, "CommandError", g_errType);
    g_objType = reinterpret_cast<PyTypeObject*>(PyType_FromSpec(&g_spec));
    if (!g_objType) return nullptr;
    Py_INCREF(g_objType);
    PyModule_AddObject(m, "Object", reinterpret_cast<PyObject*>(g_objType));
    PyModule_AddIntConstant(m, "API_VERSION", 1);
    return m;
}

String ExcText() {
    // 현재 예외를 "형식: 내용" 한 줄로 (예외는 지운다)
    PyObject *t, *v, *tb;
    PyErr_Fetch(&t, &v, &tb);
    PyErr_NormalizeException(&t, &v, &tb);
    String out = "Python 예외";
    if (t) {
        PyObject* n = PyObject_GetAttrString(t, "__name__");
        if (n && PyUnicode_Check(n)) out = PyUnicode_AsUTF8(n);
        Py_XDECREF(n);
        PyErr_Clear();
    }
    if (v) {
        PyObject* s = PyObject_Str(v);
        if (s && PyUnicode_Check(s)) {
            out += ": ";
            out += PyUnicode_AsUTF8(s);
        }
        Py_XDECREF(s);
        PyErr_Clear();
    }
    Py_XDECREF(t);
    Py_XDECREF(v);
    Py_XDECREF(tb);
    return out;
}

// 예외를 traceback 과 함께 출력으로 보내고 한 줄 요약을 돌려준다. SystemExit 는 종료 요청으로 바꾼다.
String PrintAndSummarize(bool* wasExit) {
    *wasExit = false;
    if (PyErr_ExceptionMatches(PyExc_SystemExit)) {
        PyErr_Clear();
        *wasExit = true;
        g_rt->RequestQuit();
        return String();
    }
    PyObject *t, *v, *tb;
    PyErr_Fetch(&t, &v, &tb);
    PyErr_NormalizeException(&t, &v, &tb);
    Py_XINCREF(t);
    Py_XINCREF(v);
    Py_XINCREF(tb);
    PyErr_Restore(t, v, tb);
    String summary = ExcText();   // 지운다
    PyErr_Restore(t, v, tb);
    PyErr_Print();                // traceback -> sys.stderr (우리 출력)
    return summary;
}

}  // namespace

// ---- Runtime ----

Runtime::Runtime(sim::Engine* engine) : engine_(engine), impl_(new Impl), initialized_(false), quit_(false), out_(nullptr), outUser_(nullptr) {}

Runtime::~Runtime() {
    if (initialized_) {
        M_clear_callbacks(nullptr, nullptr);
        if (engine_ && engine_->GetListener() == this) engine_->SetListener(nullptr);
        Py_Finalize();
        g_rt = nullptr;
    }
    delete impl_;
}

String Runtime::PythonVersion() { return String(PY_VERSION); }

void Runtime::Emit(bool isError, const char* text) {
    String& buf = impl_->lineBuf[isError ? 1 : 0];
    for (const char* c = text; *c; c++) {
        if (*c == '\n') {
            if (out_) out_(isError, buf.c_str(), outUser_);
            buf = String();
        } else if (*c != '\r') {
            buf += *c;
        }
    }
}

void Runtime::FlushOutput() {
    for (int k = 0; k < 2; k++) {
        if (!impl_->lineBuf[k].empty() && out_) out_(k == 1, impl_->lineBuf[k].c_str(), outUser_);
        impl_->lineBuf[k] = String();
    }
}

bool Runtime::Initialize(String* error) {
    if (initialized_) return true;
    if (g_rt) {
        if (error) *error = "Python 인터프리터는 프로세스에 하나만 만들 수 있습니다";
        return false;
    }
    g_rt = this;
    if (PyImport_AppendInittab("simul", PyInit_simul) == -1) {
        g_rt = nullptr;
        if (error) *error = "simul 모듈을 등록하지 못했습니다";
        return false;
    }
    Py_InitializeEx(0);   // 신호 처리기는 건드리지 않는다
    if (!Py_IsInitialized()) {
        g_rt = nullptr;
        if (error) *error = "Python 인터프리터를 시작하지 못했습니다";
        return false;
    }
    initialized_ = true;
    engine_->SetListener(this);
    const char* boot =
        "import sys, simul\n"
        "class _Out(object):\n"
        "    def __init__(self, err): self._err = err\n"
        "    def write(self, s):\n"
        "        simul._write(1 if self._err else 0, s)\n"
        "        return len(s)\n"
        "    def flush(self): pass\n"
        "sys.stdout = _Out(False)\n"
        "sys.stderr = _Out(True)\n";
    if (PyRun_SimpleString(boot) != 0) {
        if (error) *error = "Python 출력 연결에 실패했습니다";
        return false;
    }
    return true;
}

bool Runtime::report(String* error) {
    bool wasExit = false;
    const String s = PrintAndSummarize(&wasExit);
    FlushOutput();
    if (error) *error = s;
    return wasExit;
}

bool Runtime::RunFile(const String& path, String* error) {
    if (!initialized_) {
        if (error) *error = "Python 이 초기화되지 않았습니다";
        return false;
    }
    Array<unsigned char> bytes;
    String err;
    if (!ReadAllBytes(path, &bytes, &err)) {
        if (error) *error = "스크립트를 읽을 수 없습니다: " + path;
        return false;
    }
    String src;
    for (size_t i = 0; i < bytes.size(); i++) src += static_cast<char>(bytes[i]);
    // 그 폴더를 import 경로 앞에 둔다
    size_t cut = path.size();
    while (cut > 0 && path[cut - 1] != '/' && path[cut - 1] != '\\') cut--;
    String dir = cut > 0 ? path.substr(0, cut - 1) : String(".");
    if (dir.empty()) dir = "/";
    PyObject* sysPath = PySys_GetObject("path");   // 빌린 참조
    if (sysPath) {
        PyObject* d = Str(dir);
        bool have = false;
        for (Py_ssize_t i = 0; i < PyList_Size(sysPath); i++)
            if (PyObject_RichCompareBool(PyList_GetItem(sysPath, i), d, Py_EQ) == 1) have = true;
        if (!have) PyList_Insert(sysPath, 0, d);
        Py_DECREF(d);
    }
    PyObject* main = PyImport_AddModule("__main__");   // 빌린 참조
    PyObject* globals = PyModule_GetDict(main);
    PyObject* f = Str(path);
    PyDict_SetItemString(globals, "__file__", f);
    Py_DECREF(f);
    PyObject* code = Py_CompileString(src.c_str(), path.c_str(), Py_file_input);
    PyObject* r = code ? PyEval_EvalCode(code, globals, globals) : nullptr;
    Py_XDECREF(code);
    if (!r) {
        const bool exit = report(error);
        return exit;   // SystemExit 는 정상 종료로 본다
    }
    Py_DECREF(r);
    FlushOutput();
    return true;
}

bool Runtime::RunSnippet(const String& code, String* error) {
    if (!initialized_) {
        if (error) *error = "Python 이 초기화되지 않았습니다";
        return false;
    }
    PyObject* main = PyImport_AddModule("__main__");
    PyObject* globals = PyModule_GetDict(main);
    PyObject* c = Py_CompileString(code.c_str(), "<console>", Py_single_input);
    PyObject* r = c ? PyEval_EvalCode(c, globals, globals) : nullptr;
    Py_XDECREF(c);
    if (!r) return report(error);
    Py_DECREF(r);
    FlushOutput();
    return true;
}

void Runtime::callAll(int kind, const char* what, const sim::Event* ev, double t, double dt) {
    // 콜백 목록이 호출 중에 바뀔 수 있으니 복사해서 돈다
    Array<PyObject*> list;
    for (size_t i = 0; i < g_cb[kind].size(); i++) {
        Py_INCREF(g_cb[kind][i]);
        list.push_back(g_cb[kind][i]);
    }
    for (size_t i = 0; i < list.size(); i++) {
        PyObject* r = nullptr;
        if (kind == kTick) {
            r = PyObject_CallFunction(list[i], "dd", t, dt);
        } else if (kind == kEvent) {
            PyObject* obj = ev->object.slot >= 0 ? NewObject(ev->object) : nullptr;
            if (!obj) {
                Py_INCREF(Py_None);
                obj = Py_None;
            }
            PyObject* name = Str(ev->name);
            PyObject* joint = Str(ev->joint);
            PyObject* msg = Str(ev->message);
            PyObject* d = Py_BuildValue("{s:s,s:O,s:O,s:O,s:O,s:d}", "type", sim::EventTypeName(ev->type), "object", obj, "name", name, "joint", joint, "message", msg, "time", ev->time);
            // 경고: Py_BuildValue 의 "O" 는 참조를 늘리므로 아래에서 우리 몫을 놓는다
            Py_DECREF(obj);
            Py_DECREF(name);
            Py_DECREF(joint);
            Py_DECREF(msg);
            r = d ? PyObject_CallFunctionObjArgs(list[i], d, nullptr) : nullptr;
            Py_XDECREF(d);
        } else {
            r = PyObject_CallFunction(list[i], nullptr);
        }
        if (!r) {
            bool wasExit = false;
            const String why = PrintAndSummarize(&wasExit);
            FlushOutput();
            if (!wasExit) {
                Emit(true, (String("[simul] ") + what + " 콜백에서 예외가 나서 그 콜백을 해제합니다: " + why + "\n").c_str());
                // 예외가 난 콜백은 더 부르지 않는다 (그 스크립트만 멈춘다)
                for (size_t k = g_cb[kind].size(); k-- > 0;)
                    if (g_cb[kind][k] == list[i]) {
                        Py_DECREF(g_cb[kind][k]);
                        g_cb[kind].erase(g_cb[kind].begin() + k);
                        break;
                    }
            }
        } else {
            Py_DECREF(r);
        }
    }
    for (size_t i = 0; i < list.size(); i++) Py_DECREF(list[i]);
    FlushOutput();
}

void Runtime::OnStart(sim::Engine&) { callAll(kStart, "on_start", nullptr, 0, 0); }
void Runtime::OnTick(sim::Engine&, double time, double dt) { callAll(kTick, "on_tick", nullptr, time, dt); }
void Runtime::OnEvent(sim::Engine&, const sim::Event& e) { callAll(kEvent, "on_event", &e, 0, 0); }
void Runtime::OnStop(sim::Engine&) { callAll(kStop, "on_stop", nullptr, 0, 0); }

}  // namespace py
}  // namespace simul
