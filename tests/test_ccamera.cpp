// CSensor 카메라 테스트: 깊이(z-depth), BGR 채널 순서, 영상 방향, 가림, near/far, 잡음, 자세, BVH, 내부 변수, 예외
#include <math.h>
#include <stdio.h>

#include "CBase/Array.h"
#include "CBase/Math.h"
#include "CCollision/Bvh.h"
#include "CSensor/Camera.h"

using namespace simul;
using namespace simul::sensor;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol) { return Abs(a - b) <= tol; }

// 축에 나란한 상자 [lo, hi] (정점 8개, 삼각형 12개)
struct BoxMesh {
    Array<float> pos;
    Array<uint32_t> idx;
    BoxMesh(float x0, float y0, float z0, float x1, float y1, float z1) {
        const float p[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        for (int i = 0; i < 8; i++)
            for (int k = 0; k < 3; k++) pos.push_back(p[i][k]);
        const uint32_t f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
        for (int i = 0; i < 12; i++)
            for (int k = 0; k < 3; k++) idx.push_back(f[i][k]);
    }
    CameraTarget target(int id, float r, float g, float b, const TriangleBvh* bvh = 0) const {
        CameraTarget c;
        c.mesh.positions = pos.data();
        c.mesh.vertexCount = pos.size() / 3;
        c.mesh.indices = idx.data();
        c.mesh.indexCount = idx.size();
        c.mesh.id = id;
        c.mesh.bvh = bvh;
        c.color[0] = r;
        c.color[1] = g;
        c.color[2] = b;
        return c;
    }
};

CameraSettings small(int w, int h) {
    CameraSettings s;
    s.width = w;
    s.height = h;
    return s;
}

void testIntrinsics() {
    CameraSettings s = small(640, 480);
    s.horizontalFov = static_cast<float>(90.0 * dhPI / 180.0);
    Camera c(s);
    const CameraIntrinsics k = c.Intrinsics();
    check(near(k.fx, 320, 1e-3) && near(k.fy, 320, 1e-3) && near(k.cx, 320, 1e-6) && near(k.cy, 240, 1e-6), "90 deg over 640 px: fx = 320, principal point at the center");
    check(near(c.VerticalFov(), 2.0 * atan(240.0 / 320.0), 1e-5), "vertical fov follows the aspect ratio");
    const dhVector d = c.PixelDirection(320, 240);
    check(near(d.x, 1, 1e-6) && near(d.y, 0, 1e-6) && near(d.z, 0, 1e-6), "the center pixel looks straight ahead");
    const dhVector l = c.PixelDirection(0, 240);
    check(near(l.y, 1.0, 1e-5), "the left edge looks to +y (left) with tan(45) = 1");
    const dhVector tp = c.PixelDirection(320, 0);
    check(near(tp.z, 0.75, 1e-5), "the top edge looks up (+z)");
}

// 정면 벽: 깊이는 벽까지의 x 거리 (모든 픽셀에서 같다: z-depth 이므로 광선 길이가 아니다)
void testWallDepth() {
    const BoxMesh wall(2.0f, -5.0f, -5.0f, 2.1f, 5.0f, 5.0f);
    CameraTarget t = wall.target(7, 1.0f, 0.0f, 0.0f);
    Camera cam(small(64, 48));
    CameraImage im;
    cam.Render(dhMat(), &t, 1, &im);
    check(im.width == 64 && im.height == 48 && im.bgr.size() == 64 * 48 * 3 && im.depth.size() == 64 * 48, "image sizes");
    bool allSame = true, allHit = true;
    for (int v = 0; v < 48; v++)
        for (int u = 0; u < 64; u++) {
            if (!im.IsHit(u, v)) allHit = false;
            else if (!near(im.Depth(u, v), 2.0, 1e-4)) allSame = false;
        }
    check(allHit, "the wall fills the frame");
    check(allSame, "depth is the x distance for every pixel (z-depth)");
    check(im.targetIds[0] == 7, "target id");
    dhVector p;
    check(im.PixelPoint(0, 0, &p) && near(p.x, 2.0, 1e-4) && p.y > 0 && p.z > 0, "top-left pixel is up and to the left of the axis");
    check(im.PixelPoint(63, 47, &p) && p.y < 0 && p.z < 0, "bottom-right pixel is down and to the right");
    check(!im.PixelPoint(-1, 0, &p) && !im.PixelPoint(0, 48, &p), "out of range pixel");
}

// 색 순서: 빨강 면은 BGR 로 (0, 0, 255 근처), 파랑 면은 (255, 0, 0 근처). 면이 카메라를 정면으로 보면 밝기 1.0
void testBgrOrder() {
    const BoxMesh wall(2.0f, -5.0f, -5.0f, 2.1f, 5.0f, 5.0f);
    CameraSettings s = small(8, 8);
    Camera cam(s);
    CameraImage im;
    CameraTarget red = wall.target(0, 1.0f, 0.0f, 0.0f);
    cam.Render(dhMat(), &red, 1, &im);
    const size_t c = (4 * 8 + 4) * 3;
    check(im.bgr[c] == 0 && im.bgr[c + 1] == 0 && im.bgr[c + 2] >= 250, "red surface is stored as B=0 G=0 R=255");
    CameraTarget blue = wall.target(0, 0.0f, 0.0f, 1.0f);
    cam.Render(dhMat(), &blue, 1, &im);
    check(im.bgr[c] >= 250 && im.bgr[c + 1] == 0 && im.bgr[c + 2] == 0, "blue surface is stored as B=255 G=0 R=0");
    CameraTarget g = wall.target(0, 0.0f, 1.0f, 0.0f);
    cam.Render(dhMat(), &g, 1, &im);
    check(im.bgr[c] == 0 && im.bgr[c + 1] >= 250 && im.bgr[c + 2] == 0, "green surface is stored as B=0 G=255 R=0");
}

// 영상 방향: 왼쪽(+y)에 있는 빨강 상자는 영상 왼쪽 절반에만, 위쪽(+z)의 파랑 상자는 위쪽 절반에만 보인다
void testOrientation() {
    const BoxMesh left(3.0f, 0.5f, -0.2f, 3.1f, 1.5f, 0.2f);
    const BoxMesh up(3.0f, -0.2f, 0.5f, 3.1f, 0.2f, 1.5f);
    CameraTarget t[2] = {left.target(1, 1, 0, 0), up.target(2, 0, 0, 1)};
    CameraSettings s = small(64, 48);
    Camera cam(s);
    CameraImage im;
    cam.Render(dhMat(), t, 2, &im);
    int leftHits = 0, rightHits = 0, upHits = 0, downHits = 0;
    for (int v = 0; v < 48; v++)
        for (int u = 0; u < 64; u++) {
            const int id = im.targetIds[static_cast<size_t>(v) * 64 + static_cast<size_t>(u)];
            if (id == 1) (u < 32 ? leftHits : rightHits)++;
            if (id == 2) (v < 24 ? upHits : downHits)++;
        }
    check(leftHits > 0 && rightHits == 0, "an object at +y appears in the left half of the image");
    check(upHits > 0 && downHits == 0, "an object at +z appears in the top half of the image");
}

void testOcclusionRangeAndBackground() {
    const BoxMesh front(1.0f, -0.3f, -0.3f, 1.1f, 0.3f, 0.3f);
    const BoxMesh back(3.0f, -5.0f, -5.0f, 3.1f, 5.0f, 5.0f);
    CameraTarget t[2] = {back.target(2, 0, 1, 0), front.target(1, 1, 0, 0)};  // 먼 것이 먼저 와도 가까운 것이 이긴다
    CameraSettings s = small(32, 32);
    Camera cam(s);
    CameraImage im;
    cam.Render(dhMat(), t, 2, &im);
    check(im.targetIds[16 * 32 + 16] == 1 && near(im.Depth(16, 16), 1.0, 1e-4), "the nearer box hides the wall behind it");
    check(im.targetIds[0] == 2 && near(im.Depth(0, 0), 3.0, 1e-4), "the wall is visible outside the box");
    s.farClip = 2.0f;
    cam.SetSettings(s);
    cam.Render(dhMat(), t, 2, &im);
    check(im.targetIds[0] == -1 && im.Depth(0, 0) == 0.0f, "beyond far clip: no hit, depth 0");
    const size_t c = 0;
    check(im.bgr[c] == 46 && im.bgr[c + 1] == 36 && im.bgr[c + 2] == 31, "background color is written in BGR (default 0.12 0.14 0.18 = R31 G36 B46)");
    s.farClip = 10.0f;
    s.nearClip = 1.5f;
    cam.SetSettings(s);
    cam.Render(dhMat(), t, 2, &im);
    check(im.targetIds[16 * 32 + 16] == 2, "closer than near clip is skipped, so the wall behind shows");
}

void testPoseNoiseBvh() {
    const BoxMesh wall(0.0f, -5.0f, -5.0f, 0.1f, 5.0f, 5.0f);
    TriangleBvh bvh;
    bvh.Build(wall.pos.data(), wall.pos.size() / 3, wall.idx.data(), wall.idx.size());
    CameraTarget a = wall.target(0, 0.5f, 0.5f, 0.5f);
    CameraTarget b = wall.target(0, 0.5f, 0.5f, 0.5f, &bvh);
    // 센서를 x = -3 에 놓고 z 축 둘레로 180 도 돌리면 -x 를 향한다. 벽은 x = 0 이므로 반대로 보게 만들면 안 보인다.
    dhMat pose = dhMat::Trans(-3.0f, 0.0f, 0.0f);
    Camera cam(small(16, 12));
    CameraImage im, imB;
    cam.Render(pose, &a, 1, &im);
    cam.Render(pose, &b, 1, &imB);
    check(near(im.Depth(8, 6), 3.0, 1e-4), "sensor pose is applied: wall 3 m ahead");
    bool same = true;
    for (size_t i = 0; i < im.depth.size(); i++) same = same && im.depth[i] == imB.depth[i] && im.bgr[i * 3] == imB.bgr[i * 3];
    check(same, "BVH gives the same image as the plain mesh");
    dhMat turned = pose * dhMat::RotZ(static_cast<float>(dhPI));
    cam.Render(turned, &a, 1, &im);
    check(!im.IsHit(8, 6), "a camera turned away sees nothing");

    CameraSettings s = small(16, 12);
    s.noiseStdDev = 0.01f;
    s.seed = 5;
    Camera n1(s), n2(s);
    CameraImage i1, i2;
    n1.Render(pose, &a, 1, &i1);
    n2.Render(pose, &a, 1, &i2);
    bool eq = true, differs = false;
    double sum = 0, sum2 = 0;
    for (size_t i = 0; i < i1.depth.size(); i++) {
        eq = eq && i1.depth[i] == i2.depth[i];
        const double e = i1.depth[i] - 3.0;
        sum += e;
        sum2 += e * e;
        if (Abs(e) > 1e-6) differs = true;
    }
    const double n = static_cast<double>(i1.depth.size());
    check(eq && differs, "noise is repeatable for the same seed and nonzero");
    check(near(sum / n, 0.0, 0.004) && near(sqrt(sum2 / n), 0.01, 0.004), "noise has about the requested standard deviation");
}

void testEdgeCases() {
    CameraSettings s = small(0, 10);
    Camera bad(s);
    CameraImage im;
    CameraTarget none;
    bad.Render(dhMat(), &none, 0, &im);
    check(!bad.Valid() && im.width == 0 && im.bgr.empty(), "zero width is rejected without crashing");
    s = small(4, 4);
    s.horizontalFov = 0.0f;
    check(!Camera(s).Valid(), "zero fov is invalid");
    s = small(4, 4);
    s.farClip = s.nearClip;
    check(!Camera(s).Valid(), "far must be larger than near");
    Camera ok(small(4, 4));
    ok.Render(dhMat(), &none, 1, &im);   // 빈 메시 타깃
    check(im.width == 4 && !im.IsHit(0, 0) && im.depth[0] == 0.0f, "empty target: all background");
}

}  // namespace

int main() {
    testIntrinsics();
    testWallDepth();
    testBgrOrder();
    testOrientation();
    testOcclusionRangeAndBackground();
    testPoseNoiseBvh();
    testEdgeCases();
    if (g_fail) {
        printf("test_ccamera: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_ccamera: all passed\n");
    return 0;
}
