#include "CSimul/Headless.h"

namespace simul {
namespace sim {

namespace {

int KindOf(urdf::JointType t) {
    switch (t) {
        case urdf::kJointRevolute: return kJointRevoluteKind;
        case urdf::kJointContinuous: return kJointContinuousKind;
        case urdf::kJointPrismatic: return kJointPrismaticKind;
        default: return kJointFixedKind;
    }
}

}  // namespace

HeadlessScene::HeadlessScene() : dirty_(true) { world_.SetForceComposite(true); }

void HeadlessScene::Clear() {
    world_.Clear();
    world_.SetForceComposite(true);
    robot_ = urdf::Robot();
    colModel_ = RobotCollisionModel();
    q_.clear();
    linkT_.clear();
    report_ = SelfCollisionReport();
    warnings_.clear();
    dirty_ = true;
}

bool HeadlessScene::rebuild(String* error) {
    urdf::Robot built;
    if (!world_.Build(options_, &built, error)) return false;
    urdf::JointValues q;
    for (urdf::JointValues::const_iterator it = q_.begin(); it != q_.end(); ++it)
        if (built.FindJoint(it->first) >= 0) q[it->first] = it->second;   // 이미 올라 있던 모델의 관절 값은 유지한다
    q_ = q;
    robot_ = built;
    warnings_.clear();
    for (size_t i = 0; i < robot_.warnings.size(); i++) warnings_.push_back(robot_.warnings[i]);
    CollisionModelOptions copt;
    copt.load = options_;
    copt.fallbackToVisual = true;
    colModel_ = RobotCollisionModel();
    BuildCollisionModel(robot_, copt, &colModel_);
    for (size_t i = 0; i < colModel_.warnings.size(); i++) warnings_.push_back(colModel_.warnings[i]);
    dirty_ = true;
    return true;
}

int HeadlessScene::AddModel(const String& path, const dhMat& placement, String* error) {
    if (!world_.Add(path, error)) return -1;
    const size_t idx = world_.size() - 1;
    world_.SetPlacement(idx, placement);
    if (!rebuild(error)) {
        world_.Remove(idx);   // 합치지 못하면 추가를 취소한다
        if (world_.size() > 0) rebuild(nullptr);
        return -1;
    }
    return static_cast<int>(idx);
}

bool HeadlessScene::RemoveModel(const String& name) {
    for (size_t i = 0; i < world_.size(); i++) {
        if (world_.entry(i).name != name) continue;
        world_.Remove(i);
        if (world_.size() == 0) {
            Clear();
        } else {
            rebuild(nullptr);
        }
        return true;
    }
    return false;
}

String HeadlessScene::ModelName(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return String();
    return world_.entry(static_cast<size_t>(index)).name;
}

String HeadlessScene::ModelPath(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return String();
    return world_.entry(static_cast<size_t>(index)).path;
}

dhMat HeadlessScene::ModelPlacement(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return dhMat();
    return world_.entry(static_cast<size_t>(index)).placement;
}

bool HeadlessScene::ReadObject(int index, ObjectInfo* out) const {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) return false;
    out->name = world_.entry(static_cast<size_t>(index)).name;
    out->pose = dhPose::FromMat(world_.entry(static_cast<size_t>(index)).placement);
    out->joints.clear();
    for (size_t j = 0; j < robot_.joints.size(); j++) {
        const urdf::Joint& jt = robot_.joints[j];
        if (!jt.IsMovable() || jt.hasMimic) continue;
        const int child = robot_.FindLink(jt.child);
        if (child < 0 || world_.EntryOfLink(robot_, child) != index) continue;
        JointInfo ji;
        ji.name = jt.name;
        ji.kind = KindOf(jt.type);
        urdf::JointValues::const_iterator it = q_.find(jt.name);
        ji.value = it != q_.end() ? static_cast<double>(it->second) : 0.0;
        ji.hasLimit = jt.hasLimit;
        ji.lower = jt.lower;
        ji.upper = jt.upper;
        ji.velocity = jt.velocity;
        out->joints.push_back(ji);
    }
    out->kind = out->joints.empty() ? "model" : "robot";
    return true;
}

bool HeadlessScene::SetPose(int index, const dhPose& pose, String* error) {
    if (index < 0 || static_cast<size_t>(index) >= world_.size()) {
        if (error) *error = "모델이 없습니다";
        return false;
    }
    const dhMat m = pose.Normalized().ToMat();
    world_.SetPlacement(static_cast<size_t>(index), m);
    const String& root = world_.entry(static_cast<size_t>(index)).rootName;
    for (size_t j = 0; j < robot_.joints.size(); j++)
        if (robot_.joints[j].child == root && robot_.joints[j].parent == "world") robot_.joints[j].origin = m;
    dirty_ = true;
    return true;
}

bool HeadlessScene::SetJoint(int index, const String& joint, double value, String* error) {
    ObjectInfo info;
    if (!ReadObject(index, &info)) {
        if (error) *error = "모델이 없습니다";
        return false;
    }
    bool mine = false;
    for (size_t k = 0; k < info.joints.size(); k++) mine = mine || info.joints[k].name == joint;
    if (!mine) {
        if (error) *error = "이 모델에 움직이는 관절 '" + joint + "' 가 없습니다";
        return false;
    }
    q_[joint] = static_cast<float>(value);
    dirty_ = true;
    return true;
}

void HeadlessScene::refresh() const {
    if (!dirty_) return;
    dirty_ = false;
    if (robot_.links.empty()) {
        linkT_.clear();
        report_ = SelfCollisionReport();
        return;
    }
    linkT_ = robot_.LinkTransforms(q_);
    report_ = CheckSelfCollisions(robot_, colModel_, q_);
}

void HeadlessScene::ReadCollisions(Array<ContactInfo>* out) const {
    out->clear();
    refresh();
    for (size_t i = 0; i < report_.collisions.size(); i++) {
        const int a = report_.collisions[i].a, b = report_.collisions[i].b;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= robot_.links.size() || static_cast<size_t>(b) >= robot_.links.size()) continue;
        const int ma = world_.EntryOfLink(robot_, a), mb = world_.EntryOfLink(robot_, b);
        if (ma < 0 || mb < 0) continue;   // 가상 world 링크는 오브젝트가 아니다
        ContactInfo c;
        c.objectA = world_.entry(static_cast<size_t>(ma)).name;
        c.linkA = robot_.links[static_cast<size_t>(a)].name;
        c.objectB = world_.entry(static_cast<size_t>(mb)).name;
        c.linkB = robot_.links[static_cast<size_t>(b)].name;
        c.sameObject = ma == mb;
        out->push_back(c);
    }
}

}  // namespace sim
}  // namespace simul
