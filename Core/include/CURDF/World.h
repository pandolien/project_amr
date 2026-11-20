#ifndef CORE_CURDF_WORLD_H
#define CORE_CURDF_WORLD_H

#include "CBase/Array.h"
#include "CBase/String.h"
#include "CURDF/Urdf.h"

// 여러 모델(URDF 로봇, 또는 STL/OBJ/DAE/glTF 메시 파일)을 한 장면에 올린다.
// Build() 가 이 모델들을 하나의 Robot 으로 합친다:
//   - 모델이 하나면 그 모델 그대로.
//   - 둘 이상이면 가상 링크 "world" 를 루트로 두고, 모델마다 고정 관절 "<모델이름>_attach" 로 모델의 루트 링크를 매단다.
//     그 관절의 origin 이 모델의 월드 자세(placement)다. 합친 Robot 은 구조 트리, 관절 패널, 충돌, 중력, 라이다를 그대로 쓴다.
//   - 이름(링크, 관절)이 겹치면 뒤에 온 모델의 이름 앞에 "<모델이름>/" 을 붙인다. 먼저 온 모델의 이름은 바뀌지 않는다.
//   - 메시 경로는 모델마다 URDF 폴더가 달라서, 합치면서 실제 파일의 절대 경로로 바꾼다.
namespace simul {
namespace urdf {

// 모델을 다른 모델의 링크에 관절로 붙이는 정보. connected 가 false 이면 world 에 고정 관절로 매달린다 (기본).
// connected 이면 WorldEntry::placement 가 "부모 링크 -> 이 모델의 루트 링크" 관절 origin 이다 (world 에 매달릴 때는 월드 자세).
struct AttachSpec {
    bool connected;
    simul::String parentModel;   // 부모 모델의 장면 이름 (WorldEntry::name)
    simul::String parentLink;    // 부모 모델 안의 링크 이름 (그 모델을 읽은 그대로의 이름. 합친 뒤의 이름이 아니다)
    JointType type;              // fixed, revolute, continuous, prismatic
    dhVector axis;               // 관절 좌표계의 축 (SetAttach 가 정규화한다)
    bool hasLimit;
    float lower, upper, effort, velocity;
    simul::String jointName;     // 비면 "<모델이름>_attach"
    AttachSpec() : connected(false), type(kJointFixed), axis(0, 0, 1), hasLimit(false), lower(0), upper(0), effort(10), velocity(1) {}   // effort/velocity: URDF <limit> 의 필수 값이라 0 이 아닌 기본값
};

struct WorldEntry {
    simul::String name;      // 장면 안에서 고유한 이름 (파일 이름에서 만든다)
    simul::String path;      // 읽은 파일
    Robot robot;             // 읽은 그대로
    dhMat placement;         // 월드 기준 자세 (모델이 둘 이상일 때 attach 관절의 origin). attach.connected 이면 부모 링크 기준 관절 origin
    AttachSpec attach;       // 다른 모델에 관절로 붙였는가
    simul::String rootName;  // Build() 결과에서 이 모델의 루트 링크 이름
    simul::Array<simul::String> linkNames;  // Build() 결과에서 이 모델에 속한 링크 이름들
    Robot base;              // 크기를 바꾸기 전의 원래 로봇 (robot 은 여기에 크기 배율을 적용한 것)
    double unitScale = 1.0;  // 메시 크기 배율 (SetMeshScale 로 정한 값). 파일이 mm 이면 0.001
    double modelScale = 1.0; // 모델 전체 배율 (SetModelScale)
    bool massScales = true;
};

class World {
public:
    // 파일을 읽어 맨 뒤에 추가한다. .urdf 는 URDF 로, 그 밖은 메시 하나짜리 로봇으로 읽는다.
    bool Add(const simul::String& path, simul::String* error);
    // 파일 없이 만든 로봇(예: MakePrimitiveRobot)을 맨 뒤에 추가한다. 이름이 겹치면 숫자를 붙인다. 추가된 모델의 장면 이름은 entry(size()-1).name.
    bool AddRobot(const simul::String& name, const Robot& robot, simul::String* error);
    // 모델을 지운다. 이 모델을 부모로 삼던 모델은 연결이 풀려 world 에 매달린다 (placement 는 그대로이므로 월드 자세를 보존하려면 부른 쪽이 먼저 바꿔 둔다).
    void Remove(size_t index);
    void Clear() { entries_.clear(); forceComposite_ = false; }
    // 모델이 하나여도 가상 "world" 아래에 매달아 합친다 (회전처럼 attach 관절로 자세를 표현해야 할 때)
    void SetForceComposite(bool on) { forceComposite_ = on; }
    bool Composite() const { return entries_.size() >= 2 || (forceComposite_ && !entries_.empty()); }
    size_t size() const { return entries_.size(); }
    const WorldEntry& entry(size_t i) const { return entries_[i]; }
    void SetPlacement(size_t i, const dhMat& placement) { entries_[i].placement = placement; }
    // 모델의 메시(시각, 충돌)를 s 배로 만든다. 파일이 mm/cm/inch 단위일 때 m 로 맞추려고 쓴다 (mm 이면 0.001).
    // 지금 배율이 아니라 "원래 크기 기준"의 절대 배율이다 (두 번 불러도 겹쳐지지 않는다). 상자/원통/구 같은 기본 도형은 건드리지 않는다.
    bool SetMeshScale(size_t i, double s);
    double MeshScale(size_t i) const { return entries_[i].unitScale; }
    // 모델 전체(길이, 관절 위치, 도형, 메시)를 s 배로 한다. massScales 이면 질량도 s^3 으로 (urdf::ScaleRobot 참고). 원래 크기 기준의 절대 배율이다.
    bool SetModelScale(size_t i, double s, bool massScales = true);
    double ModelScale(size_t i) const { return entries_[i].modelScale; }

    // 모델 i 를 모델 a.parentModel 의 링크 a.parentLink 에 관절로 붙인다 (a.connected 는 true 로 바뀐다).
    // 거부하는 경우: 부모가 없거나 자기 자신, 부모 링크가 없음, 순환(부모가 이 모델의 자손), 지원하지 않는 종류, 축이 0, revolute/prismatic 에 한계가 없거나 lower > upper.
    // 관절 origin 은 placement 이므로 SetPlacement 로 따로 정한다.
    bool SetAttach(size_t i, const AttachSpec& a, simul::String* error);
    void ClearAttach(size_t i);   // world 에 고정 관절로 되돌린다 (placement 는 그대로)
    bool IsConnected(size_t i) const { return i < entries_.size() && entries_[i].attach.connected; }
    int ParentOf(size_t i) const;   // 부모 모델 번호, 없으면 -1
    int RootOf(size_t i) const;     // 부모를 따라 올라간 맨 위 모델 (world 에 매달린 모델). 연결되지 않았으면 자기 자신
    bool IsAncestor(size_t anc, size_t i) const;   // anc 가 i 의 (직접 또는 간접) 부모인가

    // 합친 Robot 을 만든다 (Add/Remove 뒤에 다시 부른다). 비어 있으면 false.
    bool Build(const LoadOptions& opt, Robot* out, simul::String* error);

    // 합친 Robot 의 링크가 몇 번째 모델에 속하는가 (가상 "world" 링크는 -1). 마지막 Build 기준.
    int EntryOfLink(const Robot& built, int link) const;
    // 이 링크가 어떤 모델의 루트 링크이면 그 모델 번호, 아니면 -1.
    int EntryOfRoot(const Robot& built, int link) const;
    // 모델의 attach 관절 이름 (모델이 둘 이상일 때만 합친 Robot 에 있다)
    simul::String AttachJointName(size_t i) const { return entries_[i].attach.jointName.empty() ? entries_[i].name + "_attach" : entries_[i].attach.jointName; }

private:
    void applyScales(WorldEntry* e) const;   // base 에 모델 배율과 메시 배율을 적용해 robot 을 다시 만든다
    simul::Array<WorldEntry> entries_;
    bool forceComposite_ = false;
};

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_WORLD_H
