// CMarker 테스트: ArUco 비트가 OpenCV 와 같은지, 메시/ChArUco 배치, STL/URDF 쓰기, 센서 카메라로 찍어서 읽기
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "CBase/Array.h"
#include "CBase/File.h"
#include "CBase/Math.h"
#include "CMarker/Marker.h"
#include "3d/dhMat.h"
#include "CSensor/Camera.h"
#include "CURDF/Urdf.h"

using namespace simul;
using namespace simul::marker;
using namespace simul::sensor;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

// OpenCV 4.x/5.x (cv::aruco::generateImageMarker) 로 뽑은 안쪽 비트 (행 우선, 1 = 흰색)
static const struct {
    int n, id;
    const char* bits;
} kGolden[] = {
    {4, 0, "1011010100110010"},
    {4, 7, "1100010011110010"},
    {4, 999, "1111011110111111"},
    {5, 3, "1000000111001010111110111"},
    {6, 250, "011101100110110000110101111001111000"},
    {7, 42, "0011010111101001001111111001111000110110011111000"},
    {7, 999, "1110011111100110100101101110001000000110100111111"},
};

void testBits() {
    for (size_t g = 0; g < sizeof(kGolden) / sizeof(kGolden[0]); g++) {
        unsigned char bits[49];
        const ArucoDict d = static_cast<ArucoDict>(kGolden[g].n - 4);
        check(ArucoBits(d, kGolden[g].id, bits), "bits available");
        bool same = true;
        for (int k = 0; k < kGolden[g].n * kGolden[g].n; k++) same = same && bits[k] == (kGolden[g].bits[k] == '1');
        check(same, "ArUco bits equal OpenCV");
    }
    unsigned char b[49];
    check(!ArucoBits(kDict4x4, -1, b) && !ArucoBits(kDict4x4, 1000, b), "id out of range is rejected");
    ArucoDict d;
    check(ParseArucoDict("DICT_5X5_250", &d) && d == kDict5x5, "dictionary name parsing");
    check(ParseArucoDict("7x7", &d) && d == kDict7x7, "dictionary short name");
    check(!ParseArucoDict("8x8", &d) && !ParseArucoDict("abc", &d), "unknown dictionary");
}

void bounds(const MarkerMesh& m, double* lo, double* hi) {
    for (int c = 0; c < 3; c++) lo[c] = 1e9, hi[c] = -1e9;
    for (size_t i = 0; i + 2 < m.positions.size() + 0; i += 3)
        for (int c = 0; c < 3; c++) {
            const double v = m.positions[i + static_cast<size_t>(c)];
            if (v < lo[c]) lo[c] = v;
            if (v > hi[c]) hi[c] = v;
        }
}

void testAruco() {
    MarkerParts p;
    String err;
    MarkerBuildOptions o;
    check(BuildAruco(kDict4x4, 7, 0.12, o, &p, &err), "BuildAruco");
    check(p.ids.size() == 1 && p.ids[0] == 7, "id recorded");
    double lo[3], hi[3];
    bounds(p.plate, lo, hi);
    const double half = 0.06 + 0.12 / 6.0;
    check(Abs(hi[0] - half) < 1e-6 && Abs(lo[0] + half) < 1e-6 && Abs(hi[2]) < 1e-9 && Abs(lo[2] + 0.004) < 1e-6, "plate size, margin one cell, surface at z = 0");
    bounds(p.ink, lo, hi);
    check(Abs(hi[0] - 0.06) < 1e-6 && Abs(lo[1] + 0.06) < 1e-6 && Abs(lo[2] - 0.0004) < 1e-7, "ink spans the marker and floats above the plate");
    check(!BuildAruco(kDict4x4, 1000, 0.1, o, &p, &err) && !err.empty(), "id 1000 rejected with a message");
    check(!BuildAruco(kDict4x4, 0, 0.0, o, &p, &err), "zero size rejected");
}

void testCharuco() {
    MarkerParts p;
    String err;
    MarkerBuildOptions o;
    check(BuildCharuco(kDict4x4, 5, 7, 0.04, 0.03, 0, o, &p, &err), "BuildCharuco");
    check(p.ids.size() == 17, "5x7 board has 17 markers");   // 35/2
    check(p.ids[0] == 0 && p.ids[16] == 16, "ids in raster order from 0");
    double lo[3], hi[3];
    bounds(p.plate, lo, hi);
    check(Abs(lo[0] + 0.02) < 1e-6 && Abs(hi[0] - 0.22) < 1e-6 && Abs(hi[1] - 0.30) < 1e-6 && Abs(lo[2]) < 1e-9, "board spans 0..W, 0..H from the top-left corner");
    check(Abs(p.width - 0.24) < 1e-9 && Abs(p.height - 0.32) < 1e-9, "outer size includes margin");
    bounds(p.ink, lo, hi);
    check(Abs(lo[2] + 0.0004) < 1e-7, "ink faces the viewer at -z");
    check(!BuildCharuco(kDict4x4, 5, 7, 0.04, 0.04, 0, o, &p, &err), "marker as large as the square is rejected");
    check(!BuildCharuco(kDict4x4, 5, 7, 0.04, 0.03, 990, o, &p, &err), "ids above 999 rejected");
    check(!BuildCharuco(kDict4x4, 1, 7, 0.04, 0.03, 0, o, &p, &err), "too few squares rejected");
}

void testSpec() {
    MarkerParts p;
    String name, err;
    MarkerBuildOptions o;
    check(BuildMarkerFromSpec("aruco,4x4,0,0.15", o, &p, &name, &err) && name == "aruco_4x4_id0", "aruco spec");
    check(BuildMarkerFromSpec("charuco,5x5,5,7,0.04,0.03,100", o, &p, &name, &err) && name == "charuco_5x5_5x7" && p.ids[0] == 100, "charuco spec with first id");
    check(!BuildMarkerFromSpec("aruco,4x4,0", o, &p, &name, &err), "short spec rejected");
    check(!BuildMarkerFromSpec("qr,4x4,0,0.1", o, &p, &name, &err), "unknown kind rejected");
}

// 센서 카메라로 마커를 위에서 내려다보며 찍어, 칸 가운데의 밝기로 비트를 다시 읽는다
void testCameraDecode() {
    MarkerParts p;
    String err;
    MarkerBuildOptions o;
    const int kId = 7;
    check(BuildAruco(kDict4x4, kId, 0.15, o, &p, &err), "build for camera");
    CameraTarget t[2];
    for (int k = 0; k < 2; k++) {
        const MarkerMesh& m = k == 0 ? p.plate : p.ink;
        t[k].mesh.positions = m.positions.data();
        t[k].mesh.vertexCount = m.positions.size() / 3;
        t[k].mesh.indices = m.indices.data();
        t[k].mesh.indexCount = m.indices.size();
        t[k].mesh.id = k;
        t[k].mesh.bvh = 0;
        const float c = k == 0 ? 0.93f : 0.03f;
        t[k].color[0] = t[k].color[1] = t[k].color[2] = c;
    }
    CameraSettings s;
    s.width = s.height = 240;
    s.noiseStdDev = 0.0f;
    s.colorNoiseStdDev = 0.0f;
    // 마커 위 0.4 m 에서 아래(-z)를 본다. 영상의 위쪽이 마커의 +y 가 되도록 센서를 돌린다
    const dhMat pose = dhMat::Trans(0.0f, 0.0f, 0.4f) * dhMat::RotY(static_cast<float>(dhPI / 2.0)) * dhMat::RotX(static_cast<float>(-dhPI / 2.0));
    Camera cam(s);
    CameraImage im;
    cam.Render(pose, t, 2, &im);
    const double fx = 0.5 * s.width / tan(0.5 * s.horizontalFov), cell = 0.15 / 6.0;
    unsigned char bits[16];
    ArucoBits(kDict4x4, kId, bits);
    int wrong = 0;
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++) {
            const double x = -0.075 + (c + 0.5) * cell, y = 0.075 - (r + 0.5) * cell;
            const int u = static_cast<int>(0.5 * s.width + fx * x / 0.4), v = static_cast<int>(0.5 * s.height - fx * y / 0.4);
            const bool white = im.bgr[(static_cast<size_t>(v) * static_cast<size_t>(s.width) + static_cast<size_t>(u)) * 3] > 128;
            const bool border = r == 0 || r == 5 || c == 0 || c == 5;
            const bool expect = !border && bits[(r - 1) * 4 + (c - 1)] == 1;
            if (white != expect) wrong++;
        }
    check(wrong == 0, "the camera image of the marker reads back to the same bits (upright, not mirrored)");
}

void testFiles() {
    MarkerParts p;
    String err, urdf;
    MarkerBuildOptions o;
    check(BuildAruco(kDict4x4, 3, 0.1, o, &p, &err), "build for files");
    const String dir = ".";
    check(WriteMarkerFiles(p, dir, "t_marker_tmp", &urdf, &err), "write files");
    urdf::Robot robot;
    check(urdf::LoadUrdf(urdf, &robot, &err), "written URDF loads");
    check(robot.links.size() == 1 && robot.links[0].visuals.size() == 2, "one link with plate and ink visuals");
    Array<unsigned char> stl;
    check(ReadAllBytes("t_marker_tmp_plate.stl", &stl, &err) && stl.size() == 84 + 12 * 50, "plate STL is a box: 12 triangles");
    RemoveFile("t_marker_tmp.urdf");
    RemoveFile("t_marker_tmp_plate.stl");
    RemoveFile("t_marker_tmp_ink.stl");
}

}  // namespace

int main() {
    testBits();
    testAruco();
    testCharuco();
    testSpec();
    testCameraDecode();
    testFiles();
    if (g_fail) {
        printf("test_cmarker: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_cmarker: all passed\n");
    return 0;
}
