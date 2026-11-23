#ifndef CORE_CURDF_URDF_H
#define CORE_CURDF_URDF_H

#include "CBase/Map.h"
#include "CBase/String.h"
#include "CBase/Util.h"
#include "CBase/Array.h"

#include "3d/dhMat.h"
#include "3d/Units.h"
#include "3d/dhVector.h"
#include "CMesh/Mesh.h"

// URDF(Unified Robot Description Format) 로더. xacro 는 지원하지 않는다 (먼저 xacro 명령으로 URDF 로 변환).
// 좌표계는 URDF 규약(Z-up, 미터, 라디안, rpy = 고정축 roll-pitch-yaw)을 따른다.
namespace simul {
namespace urdf {

enum GeometryType { kGeomNone, kGeomBox, kGeomCylinder, kGeomSphere, kGeomMesh };

struct Geometry {
    GeometryType type;
    float size[3];         // box 의 전체 변 길이
    float radius;          // cylinder, sphere
    float length;          // cylinder (축 = Z)
    simul::String filename;  // mesh: 원문 그대로 (package://..., file://..., 상대/절대 경로)
    float scale[3];        // mesh 의 scale (기본 1 1 1)
    Geometry() : type(kGeomNone), radius(0), length(0) {
        size[0] = size[1] = size[2] = 0;
        scale[0] = scale[1] = scale[2] = 1;
    }
};

struct MaterialDef {
    simul::String name;
    Color color;
    bool hasColor;
    simul::String texture;  // 원문 경로
    MaterialDef() : hasColor(false) {}
};

struct Visual {
    simul::String name;
    dhMat origin;  // link 좌표계 -> visual 좌표계
    Geometry geometry;
    MaterialDef material;
    bool hasMaterial;
    Visual() : hasMaterial(false) {}
};

struct Collision {
    simul::String name;
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
    simul::String name;
    simul::Array<Visual> visuals;
    simul::Array<Collision> collisions;
    Inertial inertial;
    bool hasInertial;
    Link() : hasInertial(false) {}
};

enum JointType { kJointFixed, kJointRevolute, kJointContinuous, kJointPrismatic, kJointFloating, kJointPlanar };

struct Joint {
    simul::String name;
    JointType type;
    simul::String parent;  // 부모 link 이름
    simul::String child;   // 자식 link 이름
    dhMat origin;        // 부모 link -> 관절 좌표계
    dhVector axis;       // 정규화된 회전/이동 축 (관절 좌표계, <axis> 가 없으면 기본 0 0 1 = z축. URDF 규격의 기본값 x 가 아니라 이 프로젝트의 규약)
    float lower, upper, effort, velocity;
    bool hasLimit;
    // <dynamics>: 점성 마찰 계수 damping (N·m·s/rad 또는 N·s/m)와 쿨롱 마찰 friction (N·m 또는 N). 없으면 0.
    float damping, friction;
    bool hasDynamics;
    bool hasMimic;
    simul::String mimicJoint;
    float mimicMultiplier, mimicOffset;
    // 링크/관절을 이름으로 찾은 번호를 기억해 둔다 (Robot::JointParent/JointChild/JointMimic 이 채운다).
    // 매 프레임 도는 동역학이 문자열 검색을 반복하지 않게 하려는 것이고, 이름이 바뀌어 맞지 않으면 알아서 다시 찾는다.
    mutable int parentIdx, childIdx, mimicIdx;
    Joint() : type(kJointFixed), axis(0, 0, 1), lower(0), upper(0), effort(0), velocity(0), hasLimit(false),
              damping(0), friction(0), hasDynamics(false), hasMimic(false), mimicMultiplier(1), mimicOffset(0), parentIdx(-1), childIdx(-1), mimicIdx(-1) {}
    // 회전/이동으로 움직이는 관절인가 (fixed, floating, planar 는 움직이지 않는 것으로 취급)
    bool IsMovable() const { return type == kJointRevolute || type == kJointContinuous || type == kJointPrismatic; }
    // 회전하는 관절인가 (각도: rad). prismatic 은 이동(m).
    bool IsAngular() const { return type == kJointRevolute || type == kJointContinuous; }
    // 사람에게 보여주는 값 <-> 내부 값(rad, m). 각도 관절만 degree 로 바꾸고 이동 관절은 그대로다.
    double ToDisplay(double si) const { return IsAngular() ? simul::units::RadToDeg(si) : si; }
    double FromDisplay(double display) const { return IsAngular() ? simul::units::DegToRad(display) : display; }
    const char* DisplayUnit() const { return IsAngular() ? "deg" : "m"; }
    // 토크/힘의 단위: 회전 관절은 N·m, 이동 관절은 N
    const char* TorqueUnit() const { return IsAngular() ? "N*m" : "N"; }
};

typedef simul::Map<simul::String, float> JointValues;  // 관절 이름 -> 각도(rad) 또는 변위(m)

// <gazebo reference="링크"><sensor type="ray|gpu_ray|lidar|gpu_lidar"> 로 적은 라이다 (Gazebo 형식).
// 각도는 rad, 거리는 m. 방위각 샘플 hSamples 개가 hMin..hMax 에 같은 간격으로 놓인다 (양 끝 포함).
struct LidarDef {
    simul::String name;     // sensor name (없으면 링크 이름)
    simul::String link;     // 센서가 붙은 링크
    dhMat pose;           // 링크 기준 센서 자세 (<pose>x y z roll pitch yaw</pose>)
    int hSamples;
    float hMin, hMax;
    int vSamples;         // 1 이면 2D 라이다
    float vMin, vMax;     // 고각
    float rangeMin, rangeMax;
    float noiseStdDev;
    LidarDef() : hSamples(640), hMin(-1.5707963f), hMax(1.5707963f), vSamples(1), vMin(0), vMax(0), rangeMin(0.05f), rangeMax(30.0f), noiseStdDev(0) {}
};

// <gazebo reference="링크"><sensor type="camera|depth_camera|rgbd_camera"> 로 적은 카메라 (Gazebo 형식).
// 센서 좌표계는 라이다와 같다: +x 가 보는 방향, +y 왼쪽, +z 위. 각도는 rad, 거리는 m.
struct CameraDef {
    simul::String name;     // sensor name (없으면 링크 이름)
    simul::String link;     // 센서가 붙은 링크
    dhMat pose;           // 링크 기준 센서 자세 (<pose>x y z roll pitch yaw</pose>)
    int width, height;    // <image><width>, <height>
    float horizontalFov;  // <horizontal_fov> (rad)
    float nearClip, farClip;  // <clip><near>, <far>
    float colorNoise, depthNoise;  // <noise><stddev> (색, 0..1 비율), <noise><depth_stddev> (깊이, m). 기본값은 CameraSettings 와 같다 (0.02, 0.005). 0 이면 없음
    float distortion[5];  // <distortion><k1><k2><p1><p2><k3> (OpenCV plumb_bob 과 같은 뜻. 없으면 0 = 왜곡 없음)
    CameraDef() : width(320), height(240), horizontalFov(1.0471976f), nearClip(0.05f), farClip(10.0f), colorNoise(0.02f), depthNoise(0.005f) { for (int i = 0; i < 5; i++) distortion[i] = 0.0f; }
};

// <gazebo reference="관절"><sensor type="force_torque"> 로 적은 힘/토크 센서 (Gazebo 형식). 관절에 붙는다 (라이다/카메라는 링크).
//   <pose> 는 자식 링크 기준 센서 자세, <force_torque><measure_direction> 은 parent_to_child(기본) 또는 child_to_parent,
//   <noise><stddev> 는 힘 잡음(N), <noise><torque_stddev> 는 토크 잡음(N·m, 확장. 없으면 stddev 와 같은 값).
struct FtDef {
    simul::String name;    // sensor name (없으면 관절 이름)
    simul::String joint;   // 센서가 붙은 관절
    dhMat pose;          // 자식 링크 기준 센서 자세
    bool childToParent;
    float forceNoise, torqueNoise;
    FtDef() : childToParent(false), forceNoise(0), torqueNoise(0) {}
};

// <gazebo reference="링크"><sensor type="imu"> 로 적은 IMU (Gazebo 형식).
//   <pose> 는 링크 기준 센서 자세. 잡음은 <imu><linear_acceleration><x><noise><stddev> (가속도, m/s^2),
//   <imu><angular_velocity><x><noise><stddev> (각속도, rad/s), <imu><orientation_reference_frame> 은 읽지 않는다.
//   x 축 값 하나를 세 축에 같이 쓴다 (축마다 다르게 쓰는 것은 지원하지 않음). 자세 잡음은 확장 <imu><orientation><noise><stddev> (rad).
struct ImuDef {
    simul::String name;   // sensor name (없으면 링크 이름)
    simul::String link;
    dhMat pose;
    float accelNoise, gyroNoise, orientationNoise;
    ImuDef() : accelNoise(0), gyroNoise(0), orientationNoise(0) {}
};

// <gazebo reference="관절"><sensor type="encoder"> 로 적은 엔코더 (Gazebo 표준이 아닌 이 프로젝트의 확장).
//   <encoder><resolution> 엔코더 축 1회전당 카운트 (이동 관절은 1 m 당), <gear_ratio> 엔코더 축 회전수 / 관절 회전수 (기본 1),
//   <invert>true|false, <absolute>true|false, <noise><stddev> 카운트 잡음.
struct EncoderDef {
    simul::String name;    // sensor name (없으면 관절 이름)
    simul::String joint;
    double resolution, gearRatio;
    bool invert, absolute;
    float noiseCounts;
    EncoderDef() : resolution(1024.0), gearRatio(1.0), invert(false), absolute(false), noiseCounts(0) {}
};

struct Robot {
    simul::String name;
    simul::Array<Link> links;
    simul::Array<Joint> joints;
    simul::Array<MaterialDef> materials;  // <robot> 바로 아래 이름 있는 재질
    simul::Array<LidarDef> lidars;         // <gazebo> 센서 중 라이다
    simul::Array<CameraDef> cameras;       // <gazebo> 센서 중 카메라
    simul::Array<EncoderDef> encoders;     // <gazebo> 센서 중 엔코더 (확장)
    simul::Array<ImuDef> imus;             // <gazebo> 센서 중 IMU
    simul::Array<FtDef> forceTorques;      // <gazebo> 센서 중 힘/토크 센서
    int root;                            // 루트 link 인덱스
    simul::Array<int> order;              // 부모가 먼저 오도록 정렬된 joint 인덱스
    simul::String baseDir;                 // URDF 파일이 있는 폴더 (문자열로 읽었으면 빈 문자열)
    simul::Array<simul::String> warnings;

    Robot() : root(-1) {}
    void Clear();
    int FindLink(const simul::String& n) const;   // 없으면 -1
    int FindJoint(const simul::String& n) const;  // 없으면 -1
    // 관절 ji 의 부모/자식 링크 번호, mimic 대상 관절 번호 (없으면 -1). FindLink/FindJoint 와 결과는 같지만 한 번 찾은 번호를 기억한다 (빠른 반복용).
    int JointParent(size_t ji) const;
    int JointChild(size_t ji) const;
    int JointMimic(size_t ji) const;
    // 사용자가 직접 움직이는 관절 (movable 이면서 mimic 이 아닌 것)의 joint 인덱스
    simul::Array<int> ActuatedJoints() const;
    // 관절 값을 limit 으로 제한한 뒤 순기구학을 계산한다. 결과는 links 와 같은 순서의 월드 변환.
    // q 에 없는 관절은 0. mimic 관절은 대상 관절 값으로 자동 결정된다.
    // T[child] = T[parent] * origin * motion(q)
    simul::Array<dhMat> LinkTransforms(const JointValues& q, bool clampToLimits = true) const;
    // 관절의 최종 값 (mimic, 제한 반영). 관절 이름이 없으면 0.
    float EffectiveJointValue(const JointValues& q, int jointIndex, bool clampToLimits = true) const;
};

// 구조 검증과 루트/순서 계산 (LoadUrdf 가 마지막에 하는 것과 같다). 프로그램이 직접 만들거나 합친 Robot 에 쓴다.
bool FinalizeRobot(Robot* robot, simul::String* error);

struct LoadOptions {
    // package://<이름>/... 를 찾을 폴더. (이름, 그 패키지의 루트 폴더)
    simul::Array<simul::Pair<simul::String, simul::String> > packagePaths;
    // 추가 검색 폴더. 상대 경로와 package:// 경로를 이 폴더 아래에서도 찾는다.
    simul::Array<simul::String> searchDirs;
    // GLB(Y-up) 메시를 URDF(Z-up)에 맞게 X축 +90도 회전한다.
    bool glbYUpToZUp;
    LoadOptions() : glbYUpToZUp(true) {}
};

// ---- URDF 읽기. 성공하면 links/joints 의 구조가 검증된 상태다
//   검증: 이름 중복 없음, 존재하지 않는 link 참조 없음, 루트 하나, 순환 없음, 모든 link 가 루트에 연결됨
bool LoadUrdf(const simul::String& path, Robot* out, simul::String* error);
bool LoadUrdfFromString(const simul::String& xml, Robot* out, simul::String* error, const simul::String& baseDir = simul::String());

// 메시 경로를 실제 파일로 해석한다. 찾으면 true.
//   package://pkg/a/b : packagePaths -> searchDirs -> URDF 폴더와 그 상위 폴더 순서
//   file:///abs/path, 절대 경로, 상대 경로(URDF 폴더 -> searchDirs)
bool ResolveMeshPath(const simul::String& filename, const simul::String& baseDir, const LoadOptions& opt, simul::String* resolved);

// Geometry 하나를 Scene 으로 만든다. box/cylinder/sphere 는 기본 도형을 생성하고, mesh 는 파일을 읽은 뒤
// URDF 의 scale 과 축 변환(opt.glbYUpToZUp)을 적용한다. cache 를 주면 같은 파일을 다시 읽지 않는다
// (경로 -> scale 적용 전 Scene). kGeomNone 이면 false 를 돌려주고 error 는 비워 둔다.
bool LoadGeometryScene(const Geometry& g, const simul::String& baseDir, const LoadOptions& opt, Scene* out,
                       simul::String* error, simul::Map<simul::String, Scene>* cache = 0);

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
    simul::Array<LinkVisual> visuals;
    simul::Array<simul::String> warnings;  // 읽지 못한 메시 등
};

// 모든 link 의 visual 을 읽는다 (box/cylinder/sphere 는 기본 도형으로 생성, mesh 는 파일 확장자로 판별).
// 일부 메시를 읽지 못해도 나머지는 채워지며, 모두 성공했을 때만 true.
bool LoadRobotMeshes(const Robot& robot, const LoadOptions& opt, RobotMeshes* out);

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_URDF_H
