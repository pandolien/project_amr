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

struct WorldEntry {
    simul::String name;      // 장면 안에서 고유한 이름 (파일 이름에서 만든다)
    simul::String path;      // 읽은 파일
    Robot robot;             // 읽은 그대로
    dhMat placement;         // 월드 기준 자세 (모델이 둘 이상일 때 attach 관절의 origin)
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

    // 합친 Robot 을 만든다 (Add/Remove 뒤에 다시 부른다). 비어 있으면 false.
    bool Build(const LoadOptions& opt, Robot* out, simul::String* error);

    // 합친 Robot 의 링크가 몇 번째 모델에 속하는가 (가상 "world" 링크는 -1). 마지막 Build 기준.
    int EntryOfLink(const Robot& built, int link) const;
    // 이 링크가 어떤 모델의 루트 링크이면 그 모델 번호, 아니면 -1.
    int EntryOfRoot(const Robot& built, int link) const;
    // 모델의 attach 관절 이름 (모델이 둘 이상일 때만 합친 Robot 에 있다)
    simul::String AttachJointName(size_t i) const { return entries_[i].name + "_attach"; }

private:
    void applyScales(WorldEntry* e) const;   // base 에 모델 배율과 메시 배율을 적용해 robot 을 다시 만든다
    simul::Array<WorldEntry> entries_;
    bool forceComposite_ = false;
};

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_WORLD_H
