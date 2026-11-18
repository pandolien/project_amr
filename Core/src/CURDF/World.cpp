#include "CURDF/World.h"

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
    entries_.push_back(e);
    return true;
}

void World::Remove(size_t index) {
    if (index >= entries_.size()) return;
    Array<WorldEntry> keep;
    for (size_t i = 0; i < entries_.size(); i++) if (i != index) keep.push_back(entries_[i]);
    entries_ = keep;
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
    for (size_t k = 0; k < entries_.size(); k++) {
        WorldEntry& e = entries_[k];
        const Robot& r = e.robot;
        const String prefix = e.name + "/";
        Array<String> newLink(r.links.size());
        e.linkNames.clear();
        for (size_t i = 0; i < r.links.size(); i++) {
            newLink[i] = UniqueName(r.links[i].name, prefix, &linkUsed);
            e.linkNames.push_back(newLink[i]);
        }
        e.rootName = newLink[static_cast<size_t>(r.root)];
        for (size_t i = 0; i < r.links.size(); i++) {
            Link l = r.links[i];
            l.name = newLink[i];
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
        // 관절 이름도 겹치면 prefix 를 붙인다. mimic 이 가리키는 관절 이름은 새 이름으로 맞춘다.
        Array<String> newJoint(r.joints.size());
        for (size_t i = 0; i < r.joints.size(); i++) newJoint[i] = UniqueName(r.joints[i].name, prefix, &jointUsed);
        for (size_t i = 0; i < r.joints.size(); i++) {
            Joint j = r.joints[i];
            j.name = newJoint[i];
            j.parent = newLink[static_cast<size_t>(r.FindLink(r.joints[i].parent))];
            j.child = newLink[static_cast<size_t>(r.FindLink(r.joints[i].child))];
            if (j.hasMimic) {
                const int m = r.FindJoint(j.mimicJoint);
                if (m >= 0) j.mimicJoint = newJoint[static_cast<size_t>(m)];
            }
            out->joints.push_back(j);
        }
        for (size_t i = 0; i < r.lidars.size(); i++) {
            LidarDef d = r.lidars[i];
            const int li = r.FindLink(d.link);
            if (li >= 0) d.link = newLink[static_cast<size_t>(li)];
            out->lidars.push_back(d);
        }
        for (size_t i = 0; i < r.warnings.size(); i++) out->warnings.push_back(e.name + ": " + r.warnings[i]);
        // 월드에 매다는 고정 관절
        Joint a;
        a.name = UniqueName(AttachJointName(k), prefix, &jointUsed);
        a.type = kJointFixed;
        a.parent = "world";
        a.child = e.rootName;
        a.origin = e.placement;
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
