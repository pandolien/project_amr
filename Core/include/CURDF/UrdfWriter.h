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

// 폴더로 저장: dir/ 아래에 <폴더이름>.urdf 와 obj/ (메시와 그 부속 파일) 를 만든다. 폴더째 옮겨도 열린다 (메시 경로가 모두 상대 경로).
//   GLB/glTF  -> 바이너리 STL (obj/<이름>.stl). URDF 의 scale 과 Y-up->Z-up 회전(load.glbYUpToZUp)을 정점에 미리 적용하고 scale 은 1 로 쓴다.
//                STL 에는 색이 없으므로 시각 형상에 URDF 색이 없으면 GLB 의 기본 색을 URDF <material> 로 옮긴다. 텍스처와 정점 색은 버려지고 warnings 에 남는다.
//   STL       -> obj/<이름>.stl 로 복사
//   OBJ, DAE  -> obj/<이름>/ 폴더에 복사 (OBJ 는 mtl 과 그 안의 텍스처, DAE 는 <init_from> 의 이미지)
//   기본 도형(box, cylinder, sphere)은 파일이 없다. 같은 원본은 한 번만 만들고, 이름이 겹치면 _2, _3 을 붙인다.
//   <material><texture> 이미지는 obj/ 로 복사한다.
struct FolderSaveResult {
    simul::Array<simul::String> warnings;
    simul::String urdfPath;   // 만들어진 <dir>/<이름>.urdf
    int converted;            // GLB -> STL 로 쓴 수
    int copied;               // 그대로 복사한 메시 수
    FolderSaveResult() : converted(0), copied(0) {}
};
bool SaveUrdfFolder(const Robot& robot, const simul::String& dir, const LoadOptions& load, simul::String* error,
                    FolderSaveResult* result = 0, const WriteOptions& opt = WriteOptions());

// 자세(회전 행렬) -> URDF 의 rpy (고정축 roll, pitch, yaw. R = Rz(yaw) Ry(pitch) Rx(roll)), 라디안.
void RpyFromMat(const dhMat& m, double rpy[3]);

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_URDFWRITER_H
