#include "CURDF/World.h"

#include <math.h>
#include "CURDF/Scale.h"

#include "CBase/Map.h"
#include "CBase/Text.h"
#include "CMesh/MeshLoader.h"

namespace simul {
namespace urdf {
namespace {

// 경로의 파일 이름(확장자 없이)
String BaseName(const String& path) {
    size_t s = path.find_last_of("/\\");
    String f = s == String::npos ? path : path.substr(s + 1);
    const size_t dot = f.rfind('.');
    if (dot != String::npos && dot > 0) f = f.substr(0, dot);
    return f.empty() ? String("model") : f;
}

String Extension(const String& path) {
    const size_t s = path.find_last_of("/\\");
    const size_t dot = path.rfind('.');
    if (dot == String::npos || (s != String::npos && dot < s)) return String();
    return ToLower(path.substr(dot + 1));
}

// used 에 없는 이름이면 그대로, 있으면 prefix + 이름, 그래도 겹치면 숫자를 붙인다. 고른 이름을 used 에 넣는다.
String UniqueName(const String& name, const String& prefix, Set<String>* used) {
    String cand = name;
    if (used->count(cand)) cand = prefix + name;
    String base = cand;
    for (int n = 2; used->count(cand); n++) cand = base + "_" + ToString(n);
    used->insert(cand);
    return cand;
}

void ResolveGeometry(Geometry* g, const String& baseDir, const LoadOptions& opt) {
    if (g->type != kGeomMesh || g->filename.empty()) return;
    String resolved;
    if (ResolveMeshPath(g->filename, baseDir, opt, &resolved)) g->filename = resolved;
}

}  // namespace

bool World::Add(const String& path, String* error) {
    WorldEntry e;
    e.path = path;
    const String ext = Extension(path);
    if (ext == "urdf") {
        if (!LoadUrdf(path, &e.robot, error)) return false;
    } else {
        // 메시 파일 하나 -> 링크 하나, visual 하나짜리 로봇. 형식은 파일 확장자로 판별한다 (읽는 것은 Build 뒤 LoadRobotMeshes).
        String err;
        if (!LoadMeshCached(path, nullptr, &err)) {   // 읽을 수 있는 파일인지만 본다 (캐시에 남아 뒤의 본 읽기가 다시 파싱하지 않는다)
            if (error) *error = err;
            return false;
        }
        Link l;
        l.name = BaseName(path);
        Visual v;
        v.geometry.type = kGeomMesh;
        v.geometry.filename = path;
        l.visuals.push_back(v);
        e.robot.name = l.name;
        e.robot.links.push_back(l);
        e.robot.root = 0;
    }
    // 장면 안에서 고유한 이름
    String base = BaseName(path);
    String name = base;
    for (int n = 2;; n++) {
        bool dup = false;
        for (size_t i = 0; i < entries_.size(); i++) if (entries_[i].name == name) dup = true;
        if (!dup) break;
        name = base + "_" + ToString(n);
    }
    e.name = name;
    e.base = e.robot;
    entries_.push_back(e);
    return true;
}

bool World::AddRobot(const String& name, const Robot& robot, String* error) {
    if (robot.links.empty() || robot.root < 0) {
        if (error) *error = "비어 있는 로봇은 추가할 수 없습니다";
        return false;
    }
    WorldEntry e;
    e.robot = robot;
    const String base = name.empty() ? String("model") : name;
    String unique = base;
    for (int n = 2;; n++) {
        bool dup = false;
        for (size_t i = 0; i < entries_.size(); i++) if (entries_[i].name == unique) dup = true;
        if (!dup) break;
        unique = base + "_" + ToString(n);
    }
    e.name = unique;
    e.base = e.robot;
    entries_.push_back(e);
    return true;
}

void World::Remove(size_t index) {
    if (index >= entries_.size()) return;
    const String removed = entries_[index].name;
    for (size_t i = 0; i < entries_.size(); i++)
        if (i != index && entries_[i].attach.connected && entries_[i].attach.parentModel == removed) entries_[i].attach = AttachSpec();
    Array<WorldEntry> keep;
    for (size_t i = 0; i < entries_.size(); i++) if (i != index) keep.push_back(entries_[i]);
    entries_ = keep;
}

int World::ParentOf(size_t i) const {
    if (i >= entries_.size() || !entries_[i].attach.connected) return -1;
    for (size_t k = 0; k < entries_.size(); k++)
        if (k != i && entries_[k].name == entries_[i].attach.parentModel) return static_cast<int>(k);
    return -1;
}

int World::RootOf(size_t i) const {
    int cur = static_cast<int>(i);
    for (size_t guard = 0; guard <= entries_.size(); guard++) {
        const int p = ParentOf(static_cast<size_t>(cur));
        if (p < 0) return cur;
        cur = p;
    }
    return cur;   // 순환이 있으면 (SetAttach 가 막으므로 일어나지 않는다) 어딘가에서 멈춘다
}

bool World::IsAncestor(size_t anc, size_t i) const {
    int cur = ParentOf(i);
    for (size_t guard = 0; cur >= 0 && guard <= entries_.size(); guard++) {
        if (static_cast<size_t>(cur) == anc) return true;
        cur = ParentOf(static_cast<size_t>(cur));
    }
    return false;
}

bool World::SetAttach(size_t i, const AttachSpec& in, String* error) {
    auto fail = [&](const String& m) {
        if (error) *error = m;
        return false;
    };
    if (i >= entries_.size()) return fail("모델 번호가 올바르지 않습니다");
    int p = -1;
    for (size_t k = 0; k < entries_.size(); k++)
        if (entries_[k].name == in.parentModel) p = static_cast<int>(k);
    if (p < 0) return fail("부모 모델을 찾을 수 없습니다: " + in.parentModel);
    if (static_cast<size_t>(p) == i) return fail("모델을 자기 자신에 연결할 수 없습니다");
    if (IsAncestor(i, static_cast<size_t>(p))) return fail("순환 연결입니다: 부모가 이 모델에 매달려 있습니다");
    if (entries_[static_cast<size_t>(p)].robot.FindLink(in.parentLink) < 0) return fail("부모 모델에 링크가 없습니다: " + in.parentLink);
    if (in.type != kJointFixed && in.type != kJointRevolute && in.type != kJointContinuous && in.type != kJointPrismatic)
        return fail("연결에 쓸 수 있는 관절 종류는 fixed, revolute, continuous, prismatic 입니다");
    AttachSpec a = in;
    if (a.type != kJointFixed) {
        const float len = sqrtf(a.axis.x * a.axis.x + a.axis.y * a.axis.y + a.axis.z * a.axis.z);
        if (!(len > 1e-9f)) return fail("관절 축이 0 입니다");
        a.axis = dhVector(a.axis.x / len, a.axis.y / len, a.axis.z / len);
        if ((a.type == kJointRevolute || a.type == kJointPrismatic) && (!a.hasLimit || a.lower > a.upper))
            return fail("revolute/prismatic 관절에는 lower <= upper 인 한계가 필요합니다");
    } else {
        a.hasLimit = false;
    }
    a.connected = true;
    entries_[i].attach = a;
    return true;
}

void World::ClearAttach(size_t i) {
    if (i < entries_.size()) entries_[i].attach = AttachSpec();
}

void World::applyScales(WorldEntry* e) const {
    ScaleRobot(e->base, e->modelScale, e->massScales, &e->robot);
    if (e->unitScale != 1.0) {   // 메시만 따로 곱한다 (mm 로 만든 파일 같은 것)
        const float u = static_cast<float>(e->unitScale);
        for (size_t l = 0; l < e->robot.links.size(); l++) {
            Link& link = e->robot.links[l];
            for (size_t v = 0; v < link.visuals.size(); v++) {
                Geometry& g = link.visuals[v].geometry;
                if (g.type == kGeomMesh) for (int k = 0; k < 3; k++) g.scale[k] *= u;
            }
            for (size_t c = 0; c < link.collisions.size(); c++) {
                Geometry& g = link.collisions[c].geometry;
                if (g.type == kGeomMesh) for (int k = 0; k < 3; k++) g.scale[k] *= u;
            }
        }
    }
}

bool World::SetMeshScale(size_t i, double s) {
    if (i >= entries_.size() || !(s > 0.0) || !(s < 1e9)) return false;
    entries_[i].unitScale = s;
    applyScales(&entries_[i]);
    return true;
}

bool World::SetModelScale(size_t i, double s, bool massScales) {
    if (i >= entries_.size() || !(s > 0.0) || !(s < 1e9)) return false;
    entries_[i].modelScale = s;
    entries_[i].massScales = massScales;
    applyScales(&entries_[i]);
    return true;
}

bool World::Build(const LoadOptions& opt, Robot* out, String* error) {
    if (entries_.empty()) {
        if (error) *error = "올린 모델이 없습니다";
        return false;
    }
    out->Clear();
    if (!Composite()) {
        *out = entries_[0].robot;
        entries_[0].rootName = out->links[static_cast<size_t>(out->root)].name;
        entries_[0].linkNames.clear();
        for (size_t i = 0; i < out->links.size(); i++) entries_[0].linkNames.push_back(out->links[i].name);
        // 메시 파일 하나짜리(경로가 이미 절대/상대 파일 경로)는 그대로 읽힌다
        return true;
    }
    out->name = "world";
    Set<String> linkUsed, jointUsed;
    Link w;
    w.name = "world";
    linkUsed.insert(w.name);
    out->links.push_back(w);
    // 1단계: 모든 모델의 링크/관절 이름을 정한다 (부모 모델이 뒤에 있어도 연결할 수 있게). 이름을 정하는 순서는 모델마다 링크, 관절, attach 관절이다.
    const size_t n = entries_.size();
    Array<Array<String> > newLink(n), newJoint(n);
    Array<String> attachName(n);
    for (size_t k = 0; k < n; k++) {
        WorldEntry& e = entries_[k];
        const Robot& r = e.robot;
        const String prefix = e.name + "/";
        e.linkNames.clear();
        for (size_t i = 0; i < r.links.size(); i++) {
            newLink[k].push_back(UniqueName(r.links[i].name, prefix, &linkUsed));
            e.linkNames.push_back(newLink[k][i]);
        }
        e.rootName = newLink[k][static_cast<size_t>(r.root)];
        for (size_t i = 0; i < r.joints.size(); i++) newJoint[k].push_back(UniqueName(r.joints[i].name, prefix, &jointUsed));
        attachName[k] = UniqueName(AttachJointName(k), prefix, &jointUsed);
    }
    // 2단계: 링크, 관절, 센서를 내보내고 모델마다 attach 관절을 만든다
    for (size_t k = 0; k < n; k++) {
        WorldEntry& e = entries_[k];
        const Robot& r = e.robot;
        for (size_t i = 0; i < r.links.size(); i++) {
            Link l = r.links[i];
            l.name = newLink[k][i];
            for (size_t v = 0; v < l.visuals.size(); v++) {
                ResolveGeometry(&l.visuals[v].geometry, r.baseDir, opt);
                const String& tex = l.visuals[v].material.texture;
                if (!tex.empty()) {
                    String t;
                    if (ResolveMeshPath(tex, r.baseDir, opt, &t)) l.visuals[v].material.texture = t;
                }
            }
            for (size_t c = 0; c < l.collisions.size(); c++) ResolveGeometry(&l.collisions[c].geometry, r.baseDir, opt);
            out->links.push_back(l);
        }
        // mimic 이 가리키는 관절 이름은 새 이름으로 맞춘다.
        for (size_t i = 0; i < r.joints.size(); i++) {
            Joint j = r.joints[i];
            j.name = newJoint[k][i];
            j.parent = newLink[k][static_cast<size_t>(r.FindLink(r.joints[i].parent))];
            j.child = newLink[k][static_cast<size_t>(r.FindLink(r.joints[i].child))];
            if (j.hasMimic) {
                const int m = r.FindJoint(j.mimicJoint);
                if (m >= 0) j.mimicJoint = newJoint[k][static_cast<size_t>(m)];
            }
            out->joints.push_back(j);
        }
        for (size_t i = 0; i < r.lidars.size(); i++) {
            LidarDef d = r.lidars[i];
            const int li = r.FindLink(d.link);
            if (li >= 0) d.link = newLink[k][static_cast<size_t>(li)];
            out->lidars.push_back(d);
        }
        for (size_t i = 0; i < r.cameras.size(); i++) {
            CameraDef d = r.cameras[i];
            const int li = r.FindLink(d.link);
            if (li >= 0) d.link = newLink[k][static_cast<size_t>(li)];
            out->cameras.push_back(d);
        }
        for (size_t i = 0; i < r.warnings.size(); i++) out->warnings.push_back(e.name + ": " + r.warnings[i]);
        // 월드(또는 부모 모델의 링크)에 매다는 관절
        Joint a;
        a.name = attachName[k];
        a.type = kJointFixed;
        a.parent = "world";
        a.child = e.rootName;
        a.origin = e.placement;
        if (e.attach.connected) {
            const int p = ParentOf(k);
            const int pl = p < 0 ? -1 : entries_[static_cast<size_t>(p)].robot.FindLink(e.attach.parentLink);
            if (p < 0 || pl < 0) {
                if (error) *error = "모델 " + e.name + " 의 부모 링크를 찾을 수 없습니다: " + e.attach.parentModel + "/" + e.attach.parentLink;
                return false;
            }
            a.parent = newLink[static_cast<size_t>(p)][static_cast<size_t>(pl)];
            a.type = e.attach.type;
            a.axis = e.attach.axis;
            a.hasLimit = e.attach.hasLimit;
            a.lower = e.attach.lower;
            a.upper = e.attach.upper;
            a.effort = e.attach.effort;
            a.velocity = e.attach.velocity;
        }
        out->joints.push_back(a);
    }
    // 구조 검증과 순서 계산은 URDF 읽기와 같은 코드를 쓴다
    return FinalizeRobot(out, error);
}

int World::EntryOfLink(const Robot& built, int link) const {
    if (link < 0 || static_cast<size_t>(link) >= built.links.size()) return -1;
    const String& n = built.links[static_cast<size_t>(link)].name;
    for (size_t k = 0; k < entries_.size(); k++)
        for (size_t i = 0; i < entries_[k].linkNames.size(); i++)
            if (entries_[k].linkNames[i] == n) return static_cast<int>(k);
    return -1;
}

int World::EntryOfRoot(const Robot& built, int link) const {
    if (link < 0 || static_cast<size_t>(link) >= built.links.size()) return -1;
    const String& n = built.links[static_cast<size_t>(link)].name;
    for (size_t k = 0; k < entries_.size(); k++) if (entries_[k].rootName == n) return static_cast<int>(k);
    return -1;
}

}  // namespace urdf
}  // namespace simul
