#include "CoreQt/SimulRunner.h"

#include <QDateTime>

#include "CoreQt/ModelViewer.h"
#include "CoreQt/QtString.h"

using simul::String;
using namespace simul::sim;

namespace {

int KindOf(simul::urdf::JointType t) {
    switch (t) {
        case simul::urdf::kJointRevolute: return kJointRevoluteKind;
        case simul::urdf::kJointContinuous: return kJointContinuousKind;
        case simul::urdf::kJointPrismatic: return kJointPrismaticKind;
        default: return kJointFixedKind;
    }
}

}  // namespace

int ViewerBackend::ObjectCount() const { return viewer_->ModelCount(); }

bool ViewerBackend::ReadObject(int index, ObjectInfo* out) const {
    if (index < 0 || index >= viewer_->ModelCount()) return false;
    out->name = simulqt::FromQ(viewer_->ModelName(index));
    out->pose = dhPose::FromMat(viewer_->ModelPose(index));
    out->joints.clear();
    const simul::urdf::Robot& robot = viewer_->Robot();
    const simul::urdf::JointValues& q = viewer_->CurrentJointValues();
    for (size_t j = 0; j < robot.joints.size(); j++) {
        const simul::urdf::Joint& jt = robot.joints[j];
        if (!jt.IsMovable() || jt.hasMimic) continue;
        const int child = robot.FindLink(jt.child);
        if (child < 0 || viewer_->ModelOfLink(child) != index) continue;
        JointInfo ji;
        ji.name = jt.name;
        ji.kind = KindOf(jt.type);
        simul::urdf::JointValues::const_iterator it = q.find(jt.name);
        ji.value = it != q.end() ? static_cast<double>(it->second) : 0.0;
        ji.hasLimit = jt.hasLimit;
        ji.lower = jt.lower;
        ji.upper = jt.upper;
        ji.velocity = jt.velocity;
        out->joints.push_back(ji);
    }
    out->kind = out->joints.empty() ? "model" : "robot";
    return true;
}

bool ViewerBackend::SetPose(int index, const dhPose& pose, String* error) {
    if (index < 0 || index >= viewer_->ModelCount()) {
        if (error) *error = "모델이 없습니다";
        return false;
    }
    viewer_->SetModelPose(index, pose.ToMat());
    return true;
}

bool ViewerBackend::SetJoint(int index, const String& joint, double value, String* error) {
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
    simul::urdf::JointValues q = viewer_->CurrentJointValues();
    q[joint] = static_cast<float>(value);
    viewer_->SetJointValues(q);
    return true;
}

void ViewerBackend::ReadCollisions(simul::Array<ContactInfo>* out) const {
    out->clear();
    const simul::urdf::Robot& robot = viewer_->Robot();
    const simul::Array<simul::CollisionPair>& pairs = viewer_->SelfCollisions();
    for (size_t i = 0; i < pairs.size(); i++) {
        const int a = pairs[i].a, b = pairs[i].b;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= robot.links.size() || static_cast<size_t>(b) >= robot.links.size()) continue;
        const int ma = viewer_->ModelOfLink(a), mb = viewer_->ModelOfLink(b);
        if (ma < 0 || mb < 0) continue;   // world 링크는 오브젝트가 아니다
        ContactInfo c;
        c.objectA = simulqt::FromQ(viewer_->ModelName(ma));
        c.linkA = robot.links[static_cast<size_t>(a)].name;
        c.objectB = simulqt::FromQ(viewer_->ModelName(mb));
        c.linkB = robot.links[static_cast<size_t>(b)].name;
        c.sameObject = ma == mb;
        out->push_back(c);
    }
}

SimulRunner::SimulRunner(Engine* engine, QObject* parent) : QObject(parent), engine_(engine), timer_(new QTimer(this)), dt_(1.0 / 60.0), lastMs_(0), carry_(0.0) {
    timer_->setInterval(8);   // dt 보다 잘게 깨어나서 실제 흐른 시간만큼 틱을 낸다
    connect(timer_, &QTimer::timeout, this, &SimulRunner::onTimer);
}

void SimulRunner::SetRate(double hz) {
    if (!(hz >= 1.0)) hz = 1.0;
    if (hz > 1000.0) hz = 1000.0;
    dt_ = 1.0 / hz;
}

void SimulRunner::Start() {
    if (timer_->isActive()) return;
    engine_->Start();
    lastMs_ = QDateTime::currentMSecsSinceEpoch();
    carry_ = 0.0;
    timer_->start();
    emit started();
}

void SimulRunner::Stop() {
    if (timer_->isActive()) timer_->stop();
    if (engine_->Running()) {
        engine_->Stop();
        emit stopped();
    }
}

void SimulRunner::StepOnce() {
    String err;
    if (!engine_->Running()) engine_->Start();
    if (!engine_->Step(dt_, &err)) emit stepFailed(simulqt::ToQ(err));
    else emit ticked(engine_->Time());
}

void SimulRunner::onTimer() {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    carry_ += static_cast<double>(now - lastMs_) / 1000.0;
    lastMs_ = now;
    const int maxCatchUp = 5;   // 밀려도 한 번에 이만큼만 (화면이 얼지 않게). 넘치는 시간은 버린다
    int n = 0;
    while (carry_ >= dt_ && n < maxCatchUp && timer_->isActive()) {
        String err;
        if (!engine_->Step(dt_, &err)) {
            emit stepFailed(simulqt::ToQ(err));
            break;
        }
        carry_ -= dt_;
        n++;
        emit ticked(engine_->Time());
    }
    if (carry_ > dt_ * maxCatchUp) carry_ = 0.0;
}
