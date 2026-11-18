#ifndef CORE_CURDF_URDFWRITER_H
#define CORE_CURDF_URDFWRITER_H

#include "CBase/String.h"
#include "CURDF/Urdf.h"

// Robot -> URDF 파일. 여러 모델을 합친 장면(World::Build 의 결과)이나 프로그램이 만든 Robot 을 하나의 URDF 로 저장한다.
// 읽기(LoadUrdf)의 반대이고, 쓴 파일을 다시 읽으면 같은 구조(링크, 관절, 질량/관성, 형상)가 나온다.
// 간결하게 쓴다: 기본값과 같은 요소는 생략하고, 같은 이름의 재질은 <robot> 아래에 한 번만 정의하고 이름으로 부른다.
namespace simul {
namespace urdf {

struct WriteOptions {
    simul::String robotName;      // 비면 robot.name, 그것도 비면 "robot"
    bool relativeMeshes;          // true 면 메시 경로를 저장하는 파일 기준 상대 경로로 쓴다 (불가능하면 절대 경로 그대로)
    WriteOptions() : relativeMeshes(true) {}
};

// URDF 본문. outDir 는 저장할 폴더 (메시 상대 경로 계산용, 비어 있으면 경로를 그대로 둔다).
simul::String UrdfToString(const Robot& robot, const simul::String& outDir, const WriteOptions& opt = WriteOptions());

// 파일로 저장 (폴더는 미리 있어야 한다).
bool SaveUrdf(const Robot& robot, const simul::String& path, simul::String* error, const WriteOptions& opt = WriteOptions());

// 자세(회전 행렬) -> URDF 의 rpy (고정축 roll, pitch, yaw. R = Rz(yaw) Ry(pitch) Rx(roll)), 라디안.
void RpyFromMat(const dhMat& m, double rpy[3]);

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_URDFWRITER_H
