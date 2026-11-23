#include "simul_api.h"

#include <string.h>

#include "3d/dhMat.h"
#include "CSimul/Engine.h"
#include "CSimul/Headless.h"

using namespace simul;
using namespace simul::sim;

struct simul_sim {
    HeadlessScene scene;
    Engine engine;
    String error;
    String scratch;
    Array<ContactInfo> contacts;
    Array<Event> events;
    simul_sim() : engine(&scene) {}
};

namespace {

bool Fail(simul_sim* s, const String& msg) {
    if (s) s->error = msg;
    return false;
}

bool Check(simul_sim* s, const char* what, const char* text) {
    if (!s) return false;
    if (!text) return Fail(s, String(what) + " 가 NULL 입니다");
    return true;
}

const char* Str(simul_sim* s, const String& v) {
    s->scratch = v;
    return s->scratch.c_str();
}

bool Finite(double v) { return v == v && v - v == 0.0; }

}  // namespace

extern "C" {

int simul_api_version(void) { return SIMUL_API_VERSION; }
const char* simul_version(void) { return "0.1.0"; }

simul_sim* simul_create(void) { return new simul_sim(); }
void simul_destroy(simul_sim* sim) { delete sim; }
const char* simul_last_error(simul_sim* sim) { return sim ? sim->error.c_str() : "simul_sim 이 NULL 입니다"; }

int simul_add_model(simul_sim* sim, const char* path, double x, double y, double z, double roll, double pitch, double yaw) {
    if (!Check(sim, "path", path)) return -1;
    if (!Finite(x) || !Finite(y) || !Finite(z) || !Finite(roll) || !Finite(pitch) || !Finite(yaw)) {
        Fail(sim, "위치와 자세는 유한한 수여야 합니다");
        return -1;
    }
    const dhMat m = dhMat::Trans(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)) * dhMat::RotZ(static_cast<float>(yaw)) *
                    dhMat::RotY(static_cast<float>(pitch)) * dhMat::RotX(static_cast<float>(roll));
    String err;
    const int idx = sim->scene.AddModel(String(path), m, &err);
    if (idx < 0) {
        Fail(sim, err);
        return -1;
    }
    sim->engine.Refresh();
    return idx;
}

int simul_remove_model(simul_sim* sim, const char* name) {
    if (!Check(sim, "name", name)) return 0;
    if (!sim->scene.RemoveModel(String(name))) return Fail(sim, String("오브젝트 '") + name + "' 가 없습니다");
    sim->engine.Refresh();
    return 1;
}

void simul_clear(simul_sim* sim) {
    if (!sim) return;
    sim->scene.Clear();
    sim->engine.Reset();
    sim->engine.Refresh();
}

void simul_add_search_dir(simul_sim* sim, const char* dir) {
    if (!Check(sim, "dir", dir)) return;
    sim->scene.Options().searchDirs.push_back(String(dir));
}

int simul_search_dir_count(simul_sim* sim) { return sim ? static_cast<int>(sim->scene.Options().searchDirs.size()) : 0; }
const char* simul_search_dir(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->scene.Options().searchDirs.size()) return "";
    return Str(sim, sim->scene.Options().searchDirs[static_cast<size_t>(i)]);
}

int simul_warning_count(simul_sim* sim) { return sim ? static_cast<int>(sim->scene.Warnings().size()) : 0; }
const char* simul_warning(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->scene.Warnings().size()) return "";
    return Str(sim, sim->scene.Warnings()[static_cast<size_t>(i)]);
}

int simul_object_count(simul_sim* sim) { return sim ? sim->scene.ObjectCount() : 0; }

int simul_find_object(simul_sim* sim, const char* name) {
    if (!Check(sim, "name", name)) return -1;
    for (int i = 0; i < sim->scene.ObjectCount(); i++)
        if (sim->scene.ModelName(i) == String(name)) return i;
    return -1;
}

const char* simul_object_name(simul_sim* sim, int index) { return sim ? Str(sim, sim->scene.ModelName(index)) : ""; }
const char* simul_object_path(simul_sim* sim, int index) { return sim ? Str(sim, sim->scene.ModelPath(index)) : ""; }

const char* simul_object_kind(simul_sim* sim, int index) {
    if (!sim) return "";
    ObjectInfo info;
    if (!sim->scene.ReadObject(index, &info)) return "";
    return Str(sim, info.kind);
}

int simul_object_pose(simul_sim* sim, int index, double out[7]) {
    if (!sim || !out) return 0;
    ObjectInfo info;
    if (!sim->scene.ReadObject(index, &info)) return Fail(sim, "없는 오브젝트 번호입니다");
    out[0] = info.pose.position.x; out[1] = info.pose.position.y; out[2] = info.pose.position.z;
    out[3] = info.pose.orientation.x; out[4] = info.pose.orientation.y; out[5] = info.pose.orientation.z; out[6] = info.pose.orientation.w;
    return 1;
}

int simul_joint_count(simul_sim* sim, int index) {
    if (!sim) return 0;
    ObjectInfo info;
    if (!sim->scene.ReadObject(index, &info)) return 0;
    return static_cast<int>(info.joints.size());
}

const char* simul_joint_name(simul_sim* sim, int index, int joint) {
    if (!sim) return "";
    ObjectInfo info;
    if (!sim->scene.ReadObject(index, &info) || joint < 0 || static_cast<size_t>(joint) >= info.joints.size()) return "";
    return Str(sim, info.joints[static_cast<size_t>(joint)].name);
}

int simul_joint_info(simul_sim* sim, int index, int joint, double out[6]) {
    if (!sim || !out) return 0;
    ObjectInfo info;
    if (!sim->scene.ReadObject(index, &info) || joint < 0 || static_cast<size_t>(joint) >= info.joints.size()) {
        Fail(sim, "없는 오브젝트 또는 관절 번호입니다");
        return 0;
    }
    const JointInfo& j = info.joints[static_cast<size_t>(joint)];
    out[0] = j.value; out[1] = j.lower; out[2] = j.upper; out[3] = j.velocity; out[4] = j.hasLimit ? 1.0 : 0.0; out[5] = static_cast<double>(j.kind);
    return 1;
}

int simul_collision_count(simul_sim* sim) {
    if (!sim) return 0;
    sim->engine.Contacts(&sim->contacts);
    return static_cast<int>(sim->contacts.size());
}

#define SIMUL_CONTACT_FIELD(name, expr)                                                      \
    const char* name(simul_sim* sim, int i) {                                                \
        if (!sim || i < 0 || static_cast<size_t>(i) >= sim->contacts.size()) return "";      \
        const ContactInfo& c = sim->contacts[static_cast<size_t>(i)];                        \
        return (expr).c_str();                                                               \
    }
SIMUL_CONTACT_FIELD(simul_collision_object_a, c.objectA)
SIMUL_CONTACT_FIELD(simul_collision_link_a, c.linkA)
SIMUL_CONTACT_FIELD(simul_collision_object_b, c.objectB)
SIMUL_CONTACT_FIELD(simul_collision_link_b, c.linkB)

int simul_collision_same_object(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->contacts.size()) return 0;
    return sim->contacts[static_cast<size_t>(i)].sameObject ? 1 : 0;
}

namespace {
bool Resolve(simul_sim* sim, const char* name, Handle* h) {
    if (!Check(sim, "name", name)) return false;
    *h = sim->engine.Find(String(name));
    if (h->slot < 0) return Fail(sim, String("오브젝트 '") + name + "' 가 없습니다");
    return true;
}
}  // namespace

int simul_set_pose(simul_sim* sim, const char* name, double x, double y, double z, double qx, double qy, double qz, double qw) {
    Handle h;
    if (!Resolve(sim, name, &h)) return 0;
    dhPose p;
    p.position = dhVector(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    p.orientation.x = static_cast<float>(qx); p.orientation.y = static_cast<float>(qy); p.orientation.z = static_cast<float>(qz); p.orientation.w = static_cast<float>(qw);
    String err;
    if (!sim->engine.SetPose(h, p, &err)) return Fail(sim, err);
    return 1;
}

int simul_set_joint(simul_sim* sim, const char* name, const char* joint, double value) {
    Handle h;
    if (!Resolve(sim, name, &h) || !Check(sim, "joint", joint)) return 0;
    String err;
    if (!sim->engine.SetJoint(h, String(joint), value, &err)) return Fail(sim, err);
    return 1;
}

int simul_move_to(simul_sim* sim, const char* name, double x, double y, double z, double speed) {
    Handle h;
    if (!Resolve(sim, name, &h)) return 0;
    String err;
    if (!sim->engine.MoveTo(h, dhVector(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)), speed, &err)) return Fail(sim, err);
    return 1;
}

int simul_move_joint(simul_sim* sim, const char* name, const char* joint, double target, double speed) {
    Handle h;
    if (!Resolve(sim, name, &h) || !Check(sim, "joint", joint)) return 0;
    String err;
    if (!sim->engine.MoveJoint(h, String(joint), target, speed, &err)) return Fail(sim, err);
    return 1;
}

int simul_stop_motion(simul_sim* sim, const char* name) {
    Handle h;
    if (!Resolve(sim, name, &h)) return 0;
    String err;
    if (!sim->engine.StopMotion(h, &err)) return Fail(sim, err);
    return 1;
}

int simul_moving(simul_sim* sim, const char* name) {
    Handle h;
    if (!Resolve(sim, name, &h)) return 0;
    return sim->engine.Moving(h) ? 1 : 0;
}

int simul_step(simul_sim* sim, double dt) {
    if (!sim) return 0;
    if (!sim->engine.Running()) sim->engine.Start();
    String err;
    if (!sim->engine.Step(dt, &err)) return Fail(sim, err);
    return 1;
}

double simul_time(simul_sim* sim) { return sim ? sim->engine.Time() : 0.0; }
unsigned long long simul_tick_count(simul_sim* sim) { return sim ? sim->engine.TickCount() : 0ULL; }

void simul_reset_time(simul_sim* sim) {
    if (!sim) return;
    sim->engine.Reset();
    sim->engine.Refresh();
}

int simul_take_events(simul_sim* sim) {
    if (!sim) return 0;
    sim->events = sim->engine.TakeEvents();
    return static_cast<int>(sim->events.size());
}

int simul_event_type(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->events.size()) return -1;
    return static_cast<int>(sim->events[static_cast<size_t>(i)].type);
}
const char* simul_event_object(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->events.size()) return "";
    return sim->events[static_cast<size_t>(i)].name.c_str();
}
const char* simul_event_joint(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->events.size()) return "";
    return sim->events[static_cast<size_t>(i)].joint.c_str();
}
const char* simul_event_message(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->events.size()) return "";
    return sim->events[static_cast<size_t>(i)].message.c_str();
}
double simul_event_time(simul_sim* sim, int i) {
    if (!sim || i < 0 || static_cast<size_t>(i) >= sim->events.size()) return 0.0;
    return sim->events[static_cast<size_t>(i)].time;
}

}  // extern "C"
