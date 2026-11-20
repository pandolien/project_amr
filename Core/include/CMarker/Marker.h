// ArUco 마커와 ChArUco 보드를 만든다 (OpenCV 의 cv::aruco 와 같은 마커, 같은 보드 배치).
//
// 센서 카메라(CSensor)는 물체마다 색 하나로 그리므로, 마커는 "흰 판" 메시와 그 위에 살짝 띄운 "검은 칸" 메시 두 개로 만든다.
// 두 메시에 흰색/검은색 재질을 주면 카메라 영상(BGR)에도, 3D 화면에도 마커가 정확히 보인다. 단위 m.
//
// 좌표계 (OpenCV 와 같다):
//   ArUco   : 원점은 마커 가운데, x 는 영상의 오른쪽, y 는 영상의 위쪽, z 는 보는 사람 쪽 (판은 z <= 0, 무늬 면은 z = 0). estimatePoseSingleMarkers 의 마커 좌표계와 같다.
//   ChArUco : 원점은 보드(체스판)의 왼쪽 위 모서리, x 는 오른쪽, y 는 아래쪽, z 는 보드 안쪽 (OpenCV 4.7 이후 CharucoBoard 의 좌표계. 판은 z >= 0).
//             왼쪽 위 칸은 검은색이고, 마커는 흰 칸에 raster 순서(위에서 아래, 왼쪽에서 오른쪽)로 id 0, 1, 2 ... 가 들어간다 (legacyPattern = false).
#pragma once

#include "CBase/Array.h"
#include "CBase/String.h"

namespace simul {
namespace marker {

// 사전. 마커 한 변의 비트 수 (4x4 는 안쪽 4x4 비트 + 검은 테두리). 각 사전에 id 는 0 ~ 999 (OpenCV 의 DICT_nXn_1000 과 같다. _50, _100, _250 은 앞쪽 부분)
enum ArucoDict { kDict4x4 = 0, kDict5x5 = 1, kDict6x6 = 2, kDict7x7 = 3 };

const int kMaxMarkerId = 999;

int ArucoBitSize(ArucoDict dict);                       // 4, 5, 6, 7
// 이름 "4x4" ... "7x7" (앞뒤 "DICT_", "_50" 같은 꼬리는 무시한다: "DICT_5X5_250" 도 된다) -> 사전. 못 알아보면 false
bool ParseArucoDict(const String& name, ArucoDict* dict);
String ArucoDictName(ArucoDict dict);                   // "4x4" ...
// id 의 안쪽 비트 (bitSize x bitSize, 행 우선, 위쪽 행부터. 1 = 흰색, 0 = 검은색). bits 는 bitSize*bitSize 칸. id 가 범위 밖이면 false
bool ArucoBits(ArucoDict dict, int id, unsigned char* bits);

// 삼각형 메시 (위치 xyz 가 연속, 인덱스 3개씩)
struct MarkerMesh {
    Array<float> positions;
    Array<unsigned> indices;
    bool empty() const { return indices.empty(); }
};

struct MarkerParts {
    MarkerMesh plate;      // 흰 판 (두께가 있는 상자)
    MarkerMesh ink;        // 검은 칸들 (판 위로 inkLift 만큼 띄운 얇은 면)
    Array<int> ids;        // 들어 있는 마커 id (ChArUco 는 보드의 마커 순서대로)
    double width, height;  // 판 전체 크기 (여백 포함, m)
    MarkerParts() : width(0), height(0) {}
};

struct MarkerBuildOptions {
    double margin;       // 판의 흰 여백 (m). 음수면 자동: ArUco 는 칸 하나(= 마커 한 변 / (비트 + 2)), ChArUco 는 칸 길이의 절반
    double thickness;    // 판 두께 (m)
    double inkLift;      // 검은 칸을 판 표면에서 띄우는 높이 (m). 너무 작으면 먼 곳에서 깜빡인다
    MarkerBuildOptions() : margin(-1.0), thickness(0.004), inkLift(0.0004) {}
};

// ArUco 마커 하나. sideLength = 검은 테두리를 포함한 마커 한 변 (m)
bool BuildAruco(ArucoDict dict, int id, double sideLength, const MarkerBuildOptions& options, MarkerParts* out, String* error);

// ChArUco 보드. squaresX x squaresY 칸, squareLength = 체스판 한 칸 (m), markerLength = 마커 한 변 (검은 테두리 포함, m. squareLength 보다 작아야 한다)
// firstId = 첫 마커의 id (기본 0). 마커 수 = 흰 칸 수 = squaresX*squaresY/2 (내림)
bool BuildCharuco(ArucoDict dict, int squaresX, int squaresY, double squareLength, double markerLength, int firstId, const MarkerBuildOptions& options, MarkerParts* out, String* error);

// 이진 STL 로 쓴다
bool WriteMarkerStl(const MarkerMesh& mesh, const String& path, String* error);

// "aruco,4x4,0,0.15" 또는 "charuco,4x4,5,7,0.04,0.03[,첫id]" 같은 명세 -> 마커. name 에는 파일 이름에 쓸 이름 ("aruco_4x4_id0", "charuco_4x4_5x7")
bool BuildMarkerFromSpec(const String& spec, const MarkerBuildOptions& options, MarkerParts* out, String* name, String* error);

// 마커를 baseName_plate.stl, baseName_ink.stl 과 그것을 쓰는 baseName.urdf 로 폴더에 쓴다. urdfPath 로 URDF 경로를 돌려준다.
bool WriteMarkerFiles(const MarkerParts& parts, const String& dir, const String& baseName, String* urdfPath, String* error);

}  // namespace marker
}  // namespace simul
