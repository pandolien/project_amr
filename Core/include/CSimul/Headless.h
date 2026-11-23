#ifndef CORE_CSIMUL_HEADLESS_H
#define CORE_CSIMUL_HEADLESS_H

#include "3d/dhMat.h"
#include "CCollision/RobotCollision.h"
#include "CSimul/Engine.h"
#include "CURDF/Urdf.h"
#include "CURDF/World.h"

// 화면(Qt) 없이 장면을 들고 있는 Backend. 모델(URDF 또는 메시)을 올리고, 오브젝트 자세와 관절 값을 바꾸고,
// 링크끼리 겹치는지(충돌)를 계산한다. DLL(simul_core)과 python 이 이것을 쓴다.
//   - 모델은 항상 가상 "world" 아래에 매달아 합친다 (회전도 자세로 표현할 수 있다).
//   - 관절 값과 자세를 바꿀 때는 계산하지 않고, 충돌을 읽을 때 한 번에 계산한다 (한 틱에 여러 번 바꿔도 한 번만 계산).
namespace simul {
namespace sim {

class HeadlessScene : public Backend {
public:
    HeadlessScene();

    // 모델을 올린다. 성공하면 오브젝트 번호(0..)를 돌려주고, 실패하면 -1 과 error 를 돌려준다.
    int AddModel(const String& path, const dhMat& placement, String* error);
    void Clear();
    // 이름의 모델(오브젝트)을 지운다. 번호는 지운 것 뒤로 하나씩 당겨진다. 없으면 false.
    bool RemoveModel(const String& name);

    urdf::LoadOptions& Options() { return options_; }
    int ModelCount() const { return static_cast<int>(world_.size()); }
    String ModelName(int index) const;
    String ModelPath(int index) const;
    dhMat ModelPlacement(int index) const;
    // 마지막으로 모델을 올릴 때 나온 경고들 (읽지 못한 메시 등)
    const Array<String>& Warnings() const { return warnings_; }

    // ---- Backend ----
    int ObjectCount() const override { return ModelCount(); }
    bool ReadObject(int index, ObjectInfo* out) const override;
    bool SetPose(int index, const dhPose& pose, String* error) override;
    bool SetJoint(int index, const String& joint, double value, String* error) override;
    void ReadCollisions(Array<ContactInfo>* out) const override;

private:
    bool rebuild(String* error);
    void refresh() const;   // 관절 값/자세가 바뀌었으면 링크 위치와 충돌을 다시 계산한다

    urdf::World world_;
    urdf::LoadOptions options_;
    urdf::Robot robot_;
    RobotCollisionModel colModel_;
    urdf::JointValues q_;
    mutable bool dirty_;
    mutable Array<dhMat> linkT_;
    mutable SelfCollisionReport report_;
    Array<String> warnings_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_HEADLESS_H
