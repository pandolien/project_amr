#ifndef CORE_CCOLLISION_ROBOTCOLLISION_H
#define CORE_CCOLLISION_ROBOTCOLLISION_H

#include "CBase/String.h"
#include "CBase/Util.h"
#include "CBase/Array.h"

#include "3d/dhMat.h"
#include "CCollision/Aabb.h"
#include "CCollision/Broadphase.h"
#include "CCollision/Obb.h"
#include "CMesh/Mesh.h"
#include "CURDF/Urdf.h"

// 로봇의 충돌 검사 (AABB 단계). URDF 의 <collision> 도형을 읽어 링크마다 월드 AABB 를 만들고,
// 서로 겹치는 링크 쌍을 찾는다. AABB 는 실제 모양보다 크므로 "겹침 후보" 이다 (거짓 양성은 있어도
// 거짓 음성은 없다). 정확한 판정이 필요하면 이후 단계(OBB, GJK 등)에서 후보만 다시 검사한다.
namespace simul {

// ---- 기본 AABB 계산

// 메시 정점을 t 로 변환한 뒤의 AABB (정점 단위라서 AABB 를 변환하는 것보다 정확하다)
Aabb AabbOfPrimitive(const Primitive& p, const dhMat& t);
// 장면의 모든 노드를 월드 변환한 뒤 다시 t 를 적용한 AABB. 정점이 없으면 빈 상자.
Aabb AabbOfScene(const Scene& scene, const dhMat& t);
// URDF 도형 하나의 월드 AABB. world 는 도형 좌표계 -> 월드 변환(링크 변환 * origin).
//   box, sphere, cylinder : 정확한 값 (원기둥은 기울어져도 해석적으로 계산한다)
//   mesh                  : meshLocal(도형 좌표계에서의 메시 AABB, scale/축 변환 적용 후)을 world 로 변환한 값
// 도형이 없거나 mesh 인데 meshLocal 이 없으면 빈 상자.
Aabb AabbOfGeometry(const urdf::Geometry& g, const dhMat& world, const Aabb* meshLocal);

// URDF 도형 하나의 월드 OBB (도형의 방향을 따라 돈다). 도형 좌표계에서의 경계 상자를 world 로 변환한 값이다.
//   box : 정확 / cylinder : 지름 x 길이를 감싸는 상자 / sphere : 지름 정육면체 / mesh : meshLocal 상자
// 도형이 없거나 mesh 인데 meshLocal 이 없으면 빈 상자.
Obb ObbOfGeometry(const urdf::Geometry& g, const dhMat& world, const Aabb* meshLocal);

// ---- 로봇

struct CollisionShape {
    int link;               // Robot::links 인덱스
    dhMat origin;           // link 좌표계 -> 도형 좌표계
    urdf::Geometry geometry;
    Aabb meshBounds;        // mesh 일 때만 의미 있음: 도형 좌표계에서의 메시 AABB
    bool fromVisual;        // <collision> 이 없어서 <visual> 을 대신 쓴 도형인가
    CollisionShape() : link(-1), fromVisual(false) {}
};

struct RobotCollisionModel {
    simul::Array<CollisionShape> shapes;
    simul::Array<simul::String> warnings;  // 읽지 못한 메시 등
};

struct CollisionModelOptions {
    urdf::LoadOptions load;  // 메시 파일 위치 (package 경로 등)
    // URDF 에 <collision> 이 하나도 없는 링크는 <visual> 로 대신한다. 기본은 끔
    // (collision 이 없는 링크는 원래 충돌하지 않는 링크이기도 하다).
    bool fallbackToVisual;
    CollisionModelOptions() : fallbackToVisual(false) {}
};

// 모든 링크의 충돌 도형을 모은다 (메시는 이때 한 번만 읽는다). 일부 메시를 읽지 못해도 나머지는 채워지며,
// 모두 성공했을 때만 true.
bool BuildCollisionModel(const urdf::Robot& robot, const CollisionModelOptions& opt, RobotCollisionModel* out);

// 관절 값 q 에서 링크별 월드 AABB (robot.links 와 같은 순서). 충돌 도형이 없는 링크는 빈 상자.
simul::Array<Aabb> ComputeLinkAabbs(const urdf::Robot& robot, const RobotCollisionModel& model, const urdf::JointValues& q,
                                   bool clampToLimits = true);

// 도형별 월드 OBB (model.shapes 와 같은 순서)
simul::Array<Obb> ComputeShapeObbs(const urdf::Robot& robot, const RobotCollisionModel& model, const urdf::JointValues& q,
                                  bool clampToLimits = true);

struct SelfCollisionOptions {
    // 관절로 직접 이어진 부모-자식 링크는 항상 맞닿아 있으므로 기본적으로 건너뛴다
    bool ignoreAdjacent;
    // 이 거리(m) 안으로 가까워진 쌍도 후보로 본다 (0 이면 겹칠 때만)
    float margin;
    // 검사에서 뺄 링크 이름 쌍 (순서 무관). 항상 겹쳐 있는 구조 등을 허용할 때 쓴다.
    simul::Array<simul::Pair<simul::String, simul::String> > ignoredPairs;
    SelfCollisionOptions() : ignoreAdjacent(true), margin(0.0f) {}
};

// 링크 AABB 끼리 겹치는 쌍 (Robot::links 인덱스, a < b). AABB 단계만 거친 "후보" 이다.
simul::Array<CollisionPair> FindSelfCollisions(const urdf::Robot& robot, const simul::Array<Aabb>& linkAabbs,
                                              const SelfCollisionOptions& opt = SelfCollisionOptions());

struct SelfCollisionReport {
    simul::Array<Aabb> linkAabbs;                // 링크별 월드 AABB (robot.links 와 같은 순서)
    simul::Array<Obb> shapeObbs;                 // 도형별 월드 OBB (model.shapes 와 같은 순서)
    simul::Array<CollisionPair> aabbCandidates;  // 1단계: 링크 AABB 가 겹친 쌍 (a < b)
    simul::Array<CollisionPair> collisions;      // 2단계: 그중 도형 OBB 까지 겹친 링크 쌍 (a < b)
    // collisions 의 근거가 된 도형 쌍 (model.shapes 인덱스. 첫째는 collisions 의 a 링크, 둘째는 b 링크의 도형)
    simul::Array<CollisionPair> shapePairs;
};

// 2단계 자기충돌 검사: (1) 링크 AABB 로 후보를 거르고 (2) 후보 링크의 도형끼리 OBB 로 다시 검사한다.
// opt.margin 은 두 단계 모두에 적용된다 (그 거리 안으로 가까워지면 충돌로 본다).
SelfCollisionReport CheckSelfCollisions(const urdf::Robot& robot, const RobotCollisionModel& model, const urdf::JointValues& q,
                                        const SelfCollisionOptions& opt = SelfCollisionOptions(), bool clampToLimits = true);

}  // namespace simul

#endif  // CORE_CCOLLISION_ROBOTCOLLISION_H
