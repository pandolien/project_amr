#include "CRL/ArmReachEnv.h"

#include "CBase/Math.h"

namespace simul {
namespace rl {

namespace {
const int kMaxTicksPerStep = 200;   // 한 행동이 영원히 끝나지 않는 일을 막는 안전장치

int BinOf(double q, double lo, double width, int n) {
    int i = static_cast<int>(Floor((q - lo) / width));
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;
    return i;
}
}  // namespace

ArmReachEnv::ArmReachEnv()
    : engine_(&scene_), endLinkIndex_(-1), loA_(0), loB_(0), widthA_(1), widthB_(1), dist_(0), steps_(0), success_(false), ready_(false) {}

bool ArmReachEnv::Init(const String& urdfPath, const ArmReachConfig& config, String* error) {
    cfg_ = config;
    ready_ = false;
    if (cfg_.binsA < 2 || cfg_.binsB < 2 || cfg_.maxSteps < 1 || !(cfg_.dt > 0.0) || !(cfg_.jointSpeed > 0.0)) {
        if (error) *error = "설정 값이 올바르지 않습니다 (bins >= 2, maxSteps >= 1, dt > 0, jointSpeed > 0)";
        return false;
    }
    if (cfg_.goalA < 0 || cfg_.goalA >= cfg_.binsA || cfg_.goalB < 0 || cfg_.goalB >= cfg_.binsB) {
        if (error) *error = "목표 칸이 범위를 벗어났습니다";
        return false;
    }
    if (!urdf::LoadUrdf(urdfPath, &robot_, error)) return false;
    endLinkIndex_ = robot_.FindLink(cfg_.endLink);
    if (endLinkIndex_ < 0) {
        if (error) *error = "링크 '" + cfg_.endLink + "' 가 없습니다";
        return false;
    }
    scene_.Clear();
    if (scene_.AddModel(urdfPath, dhMat(), error) < 0) return false;
    engine_.Reset();
    engine_.Start();
    engine_.Refresh();
    if (engine_.Count() < 1) {
        if (error) *error = "시뮬레이터에 로봇이 없습니다";
        return false;
    }
    handle_ = engine_.HandleAt(0);

    sim::ObjectInfo info;
    engine_.Info(handle_, &info);
    bool foundA = false, foundB = false;
    for (size_t i = 0; i < info.joints.size(); i++) {
        const sim::JointInfo& j = info.joints[i];
        if (j.name == cfg_.jointA && j.hasLimit) { loA_ = j.lower; widthA_ = (j.upper - j.lower) / cfg_.binsA; foundA = true; }
        if (j.name == cfg_.jointB && j.hasLimit) { loB_ = j.lower; widthB_ = (j.upper - j.lower) / cfg_.binsB; foundB = true; }
    }
    if (!foundA || !foundB) {
        if (error) *error = "관절 '" + cfg_.jointA + "', '" + cfg_.jointB + "' 중 한계(limit)가 있는 움직이는 관절을 찾지 못했습니다";
        return false;
    }

    // 목표 위치: 목표 칸 자세에서 팔 끝이 있는 곳
    for (size_t i = 0; i < info.joints.size(); i++) {
        if (info.joints[i].name == cfg_.jointA) info.joints[i].value = BinCenterA(cfg_.goalA);
        if (info.joints[i].name == cfg_.jointB) info.joints[i].value = BinCenterB(cfg_.goalB);
    }
    goal_ = endEffectorFrom(info);
    ready_ = true;
    return true;
}

dhVector ArmReachEnv::endEffectorFrom(const sim::ObjectInfo& info) const {
    urdf::JointValues q;
    for (size_t i = 0; i < info.joints.size(); i++) q[info.joints[i].name] = static_cast<float>(info.joints[i].value);
    const Array<dhMat> t = robot_.LinkTransforms(q);
    return t[static_cast<size_t>(endLinkIndex_)].O();
}

int ArmReachEnv::readState() {
    sim::ObjectInfo info;
    engine_.Info(handle_, &info);
    double qa = 0.0, qb = 0.0;
    for (size_t i = 0; i < info.joints.size(); i++) {
        if (info.joints[i].name == cfg_.jointA) qa = info.joints[i].value;
        if (info.joints[i].name == cfg_.jointB) qb = info.joints[i].value;
    }
    ee_ = endEffectorFrom(info);
    const double dx = static_cast<double>(ee_.x) - static_cast<double>(goal_.x);
    const double dy = static_cast<double>(ee_.y) - static_cast<double>(goal_.y);
    const double dz = static_cast<double>(ee_.z) - static_cast<double>(goal_.z);
    dist_ = Sqrt(dx * dx + dy * dy + dz * dz);
    return StateOf(BinOf(qa, loA_, widthA_, cfg_.binsA), BinOf(qb, loB_, widthB_, cfg_.binsB));
}

int ArmReachEnv::SelfCollisions() {
    Array<sim::ContactInfo> c;
    engine_.Contacts(&c);
    int n = 0;
    for (size_t i = 0; i < c.size(); i++) n += c[i].sameObject ? 1 : 0;
    return n;
}

int ArmReachEnv::Reset(Rng* rng) {
    engine_.Reset();
    engine_.TakeEvents();
    int a, b;
    do {   // 목표 칸이 아닌 곳에서 시작
        a = rng->UniformInt(cfg_.binsA);
        b = rng->UniformInt(cfg_.binsB);
    } while (a == cfg_.goalA && b == cfg_.goalB);
    engine_.SetJoint(handle_, cfg_.jointA, BinCenterA(a));
    engine_.SetJoint(handle_, cfg_.jointB, BinCenterB(b));
    engine_.Step(cfg_.dt);   // 명령은 틱 시작에 적용된다
    engine_.TakeEvents();
    steps_ = 0;
    success_ = false;
    return readState();
}

int ArmReachEnv::Step(int action, StepResult* result) {
    *result = StepResult();
    const int s = readState();
    int a = BinA(s), b = BinB(s);
    const bool onA = action == 0 || action == 1;
    int& bin = onA ? a : b;
    bin += (action == 0 || action == 2) ? 1 : -1;
    const int n = onA ? cfg_.binsA : cfg_.binsB;
    if (bin >= 0 && bin < n) {   // 범위를 벗어나는 행동은 아무 일도 일어나지 않는다 (제자리)
        const String& joint = onA ? cfg_.jointA : cfg_.jointB;
        const double target = onA ? BinCenterA(bin) : BinCenterB(bin);
        engine_.MoveJoint(handle_, joint, target, cfg_.jointSpeed);
        int ticks = 0;
        do {
            engine_.Step(cfg_.dt);
            ticks++;
        } while (engine_.Moving(handle_) && ticks < kMaxTicksPerStep);
        engine_.TakeEvents();
    }
    steps_++;
    const int next = readState();

    double r = -cfg_.stepCost - cfg_.distanceWeight * dist_;
    if (cfg_.collisionPenalty > 0.0 && SelfCollisions() > 0) r -= cfg_.collisionPenalty;
    if (dist_ < cfg_.goalTolerance) {
        r += cfg_.goalBonus;
        result->terminal = true;
        success_ = true;
    } else if (steps_ >= cfg_.maxSteps) {
        result->truncated = true;
    }
    result->reward = r;
    return next;
}

}  // namespace rl
}  // namespace simul
