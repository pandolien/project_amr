#ifndef CORE_CURDF_URDF_H
#define CORE_CURDF_URDF_H

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "3d/dhMat.h"
#include "3d/Units.h"
#include "3d/dhVector.h"
#include "CMesh/Mesh.h"

// URDF(Unified Robot Description Format) 로더. xacro 는 지원하지 않는다 (먼저 xacro 명령으로 URDF 로 변환).
// 좌표계는 URDF 규약(Z-up, 미터, 라디안, rpy = 고정축 roll-pitch-yaw)을 따른다.
namespace amr {
namespace urdf {

enum GeometryType { kGeomNone, kGeomBox, kGeomCylinder, kGeomSphere, kGeomMesh };

struct Geometry {
    GeometryType type;
    float size[3];         // box 의 전체 변 길이
    float radius;          // cylinder, sphere
    float length;          // cylinder (축 = Z)
    std::string filename;  // mesh: 원문 그대로 (package://..., file://..., 상대/절대 경로)
    float scale[3];        // mesh 의 scale (기본 1 1 1)
    Geometry() : type(kGeomNone), radius(0), length(0) {
        size[0] = size[1] = size[2] = 0;
        scale[0] = scale[1] = scale[2] = 1;
    }
};

struct MaterialDef {
    std::string name;
    Color color;
    bool hasColor;
    std::string texture;  // 원문 경로
    MaterialDef() : hasColor(false) {}
};

struct Visual {
    std::string name;
    dhMat origin;  // link 좌표계 -> visual 좌표계
    Geometry geometry;
    MaterialDef material;
    bool hasMaterial;
    Visual() : hasMaterial(false) {}
};

struct Collision {
    std::string name;
    dhMat origin;
    Geometry geometry;
};

struct Inertial {
    dhMat origin;
    float mass;
    float ixx, ixy, ixz, iyy, iyz, izz;
    Inertial() : mass(0), ixx(0), ixy(0), ixz(0), iyy(0), iyz(0), izz(0) {}
};

struct Link {
    std::string name;
    std::vector<Visual> visuals;
    std::vector<Collision> collisions;
    Inertial inertial;
    bool hasInertial;
    Link() : hasInertial(false) {}
};

enum JointType { kJointFixed, kJointRevolute, kJointContinuous, kJointPrismatic, kJointFloating, kJointPlanar };

struct Joint {
    std::string name;
    JointType type;
    std::string parent;  // 부모 link 이름
    std::string child;   // 자식 link 이름
    dhMat origin;        // 부모 link -> 관절 좌표계
    dhVector axis;       // 정규화된 회전/이동 축 (관절 좌표계, <axis> 가 없으면 기본 0 0 1 = z축. URDF 규격의 기본값 x 가 아니라 이 프로젝트의 규약)
    float lower, upper, effort, velocity;
    bool hasLimit;
    // <dynamics>: 점성 마찰 계수 damping (N·m·s/rad 또는 N·s/m)와 쿨롱 마찰 friction (N·m 또는 N). 없으면 0.
    float damping, friction;
    bool hasDynamics;
    bool hasMimic;
    std::string mimicJoint;
    float mimicMultiplier, mimicOffset;
    Joint() : type(kJointFixed), axis(0, 0, 1), lower(0), upper(0), effort(0), velocity(0), hasLimit(false),
              damping(0), friction(0), hasDynamics(false), hasMimic(false), mimicMultiplier(1), mimicOffset(0) {}
    // 회전/이동으로 움직이는 관절인가 (fixed, floating, planar 는 움직이지 않는 것으로 취급)
    bool IsMovable() const { return type == kJointRevolute || type == kJointContinuous || type == kJointPrismatic; }
    // 회전하는 관절인가 (각도: rad). prismatic 은 이동(m).
    bool IsAngular() const { return type == kJointRevolute || type == kJointContinuous; }
    // 사람에게 보여주는 값 <-> 내부 값(rad, m). 각도 관절만 degree 로 바꾸고 이동 관절은 그대로다.
    double ToDisplay(double si) const { return IsAngular() ? amr::units::RadToDeg(si) : si; }
    double FromDisplay(double display) const { return IsAngular() ? amr::units::DegToRad(display) : display; }
    const char* DisplayUnit() const { return IsAngular() ? "deg" : "m"; }
    // 토크/힘의 단위: 회전 관절은 N·m, 이동 관절은 N
    const char* TorqueUnit() const { return IsAngular() ? "N*m" : "N"; }
};

typedef std::map<std::string, float> JointValues;  // 관절 이름 -> 각도(rad) 또는 변위(m)

struct Robot {
    std::string name;
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::vector<MaterialDef> materials;  // <robot> 바로 아래 이름 있는 재질
    int root;                            // 루트 link 인덱스
    std::vector<int> order;              // 부모가 먼저 오도록 정렬된 joint 인덱스
    std::string baseDir;                 // URDF 파일이 있는 폴더 (문자열로 읽었으면 빈 문자열)
    std::vector<std::string> warnings;

    Robot() : root(-1) {}
    void Clear();
    int FindLink(const std::string& n) const;   // 없으면 -1
    int FindJoint(const std::string& n) const;  // 없으면 -1
    // 사용자가 직접 움직이는 관절 (movable 이면서 mimic 이 아닌 것)의 joint 인덱스
    std::vector<int> ActuatedJoints() const;
    // 관절 값을 limit 으로 제한한 뒤 순기구학을 계산한다. 결과는 links 와 같은 순서의 월드 변환.
    // q 에 없는 관절은 0. mimic 관절은 대상 관절 값으로 자동 결정된다.
    // T[child] = T[parent] * origin * motion(q)
    std::vector<dhMat> LinkTransforms(const JointValues& q, bool clampToLimits = true) const;
    // 관절의 최종 값 (mimic, 제한 반영). 관절 이름이 없으면 0.
    float EffectiveJointValue(const JointValues& q, int jointIndex, bool clampToLimits = true) const;
};

struct LoadOptions {
    // package://<이름>/... 를 찾을 폴더. (이름, 그 패키지의 루트 폴더)
    std::vector<std::pair<std::string, std::string> > packagePaths;
    // 추가 검색 폴더. 상대 경로와 package:// 경로를 이 폴더 아래에서도 찾는다.
    std::vector<std::string> searchDirs;
    // GLB(Y-up) 메시를 URDF(Z-up)에 맞게 X축 +90도 회전한다.
    bool glbYUpToZUp;
    LoadOptions() : glbYUpToZUp(true) {}
};

// ---- URDF 읽기. 성공하면 links/joints 의 구조가 검증된 상태다
//   검증: 이름 중복 없음, 존재하지 않는 link 참조 없음, 루트 하나, 순환 없음, 모든 link 가 루트에 연결됨
bool LoadUrdf(const std::string& path, Robot* out, std::string* error);
bool LoadUrdfFromString(const std::string& xml, Robot* out, std::string* error, const std::string& baseDir = std::string());

// 메시 경로를 실제 파일로 해석한다. 찾으면 true.
//   package://pkg/a/b : packagePaths -> searchDirs -> URDF 폴더와 그 상위 폴더 순서
//   file:///abs/path, 절대 경로, 상대 경로(URDF 폴더 -> searchDirs)
bool ResolveMeshPath(const std::string& filename, const std::string& baseDir, const LoadOptions& opt, std::string* resolved);

// Geometry 하나를 Scene 으로 만든다. box/cylinder/sphere 는 기본 도형을 생성하고, mesh 는 파일을 읽은 뒤
// URDF 의 scale 과 축 변환(opt.glbYUpToZUp)을 적용한다. cache 를 주면 같은 파일을 다시 읽지 않는다
// (경로 -> scale 적용 전 Scene). kGeomNone 이면 false 를 돌려주고 error 는 비워 둔다.
bool LoadGeometryScene(const Geometry& g, const std::string& baseDir, const LoadOptions& opt, Scene* out,
                       std::string* error, std::map<std::string, Scene>* cache = 0);

struct LinkVisual {
    int link;       // Robot::links 인덱스
    int visual;     // Link::visuals 인덱스
    dhMat local;    // link 좌표계 -> 이 장면의 좌표계 (visual origin)
    Scene scene;    // scale 과 축 변환이 이미 적용된 메시
    bool hasColor;  // URDF 에서 지정한 색 (있으면 장면 재질보다 우선해서 쓰면 된다)
    Color color;
    LinkVisual() : link(-1), visual(-1), hasColor(false) {}
};

struct RobotMeshes {
    std::vector<LinkVisual> visuals;
    std::vector<std::string> warnings;  // 읽지 못한 메시 등
};

// 모든 link 의 visual 을 읽는다 (box/cylinder/sphere 는 기본 도형으로 생성, mesh 는 파일 확장자로 판별).
// 일부 메시를 읽지 못해도 나머지는 채워지며, 모두 성공했을 때만 true.
bool LoadRobotMeshes(const Robot& robot, const LoadOptions& opt, RobotMeshes* out);

}  // namespace urdf
}  // namespace amr

#endif  // CORE_CURDF_URDF_H
