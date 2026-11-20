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
    s.noiseStdDev = 0.0f;        // 기본값은 잡음이 있다. 정확한 값을 보는 테스트는 잡음 없이
    s.colorNoiseStdDev = 0.0f;
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


// 태양: 면이 태양을 향할수록 밝고, 가려진 곳은 그림자(주변광만), 해가 지면 어둡고, 배경은 하늘색이다
void testSun() {
    const BoxMesh ground(-10.0f, -10.0f, -0.1f, 10.0f, 10.0f, 0.0f);
    const BoxMesh blocker(-1.0f, -1.0f, 2.0f, 1.0f, 1.0f, 2.2f);
    CameraTarget t[2] = {ground.target(1, 0.8f, 0.8f, 0.8f), blocker.target(2, 0.5f, 0.2f, 0.2f)};
    const dhMat pose = dhMat::Trans(0.0f, 0.0f, 5.0f) * dhMat::RotY(static_cast<float>(dhPI / 2.0));   // 아래를 본다. 영상 위쪽 = 월드 +x, 왼쪽 = 월드 +y
    CameraSettings s = small(64, 48);
    s.horizontalFov = static_cast<float>(90.0 * dhPI / 180.0);
    Camera cam(s);
    const CameraIntrinsics k = cam.Intrinsics();
    auto pixelOf = [&](float wx, float wy, int* u, int* v) {
        *u = static_cast<int>(k.cx - k.fx * wy / 5.0f);
        *v = static_cast<int>(k.cy - k.fy * wx / 5.0f);
    };
    auto green = [&](const CameraImage& im, float wx, float wy) {
        int u, v;
        pixelOf(wx, wy, &u, &v);
        return static_cast<int>(im.bgr[(static_cast<size_t>(v) * static_cast<size_t>(im.width) + static_cast<size_t>(u)) * 3 + 1]);
    };
    SunLight sun;
    sun.enabled = true;
    sun.direction[0] = 0.70710678f;   // 태양은 +x 쪽, 45도 높이: 그림자는 -x 쪽으로 드리운다 (가림막 윗면 z = 2.2 -> 바닥에서 x = -3.2 ~ -1)
    sun.direction[1] = 0.0f;
    sun.direction[2] = 0.70710678f;
    sun.intensity = 1.0f;
    sun.ambient = 0.5f;
    sun.sky[0] = 0.2f; sun.sky[1] = 0.4f; sun.sky[2] = 0.9f;
    cam.SetSun(sun);
    CameraImage im;
    cam.Render(pose, t, 2, &im);
    const int litG = green(im, 3.0f, 0.0f), shadowG = green(im, -2.0f, 0.0f);
    check(litG > 150, "ground in the sun is bright");
    check(shadowG < litG / 2 && shadowG > 20, "ground behind the blocker is in shadow (ambient only, not black)");
    check(im.IsHit(0, 0) && im.targetIds[(24) * 64 + 32] == 2, "the blocker is seen from above");

    SunLight noShadow = sun;
    noShadow.shadows = false;
    cam.SetSun(noShadow);
    cam.Render(pose, t, 2, &im);
    check(green(im, -2.0f, 0.0f) == litG, "shadows off: the same spot is lit");

    SunLight night = sun;
    night.intensity = 0.0f;
    night.ambient = 0.1f;
    cam.SetSun(night);
    cam.Render(pose, t, 2, &im);
    check(green(im, 3.0f, 0.0f) < litG / 4, "at night everything is dark");

    // 태양이 수평선 쪽이면 윗면은 약하게 비춘다 (cos 법칙)
    SunLight low = sun;
    low.direction[0] = 0.98f; low.direction[1] = 0.0f; low.direction[2] = 0.2f;
    cam.SetSun(low);
    cam.Render(pose, t, 2, &im);
    check(green(im, 3.0f, 0.0f) < litG, "a low sun lights the ground less");

    // 아무것도 안 보이는 쪽을 보면 배경이 하늘색 (BGR 로 저장)
    cam.SetSun(sun);
    const dhMat up = dhMat::Trans(0.0f, 0.0f, 5.0f) * dhMat::RotY(static_cast<float>(-dhPI / 2.0));   // 위를 본다
    cam.Render(up, t, 2, &im);
    check(!im.IsHit(32, 24) && im.bgr[(24 * 64 + 32) * 3] == 230 && im.bgr[(24 * 64 + 32) * 3 + 1] == 102 && im.bgr[(24 * 64 + 32) * 3 + 2] == 51, "with the sun on, the background is the sky color (B=230, G=102, R=51)");
    // 태양을 끄면 원래 카메라 방향 조명으로 돌아간다
    cam.SetSun(SunLight());
    cam.Render(pose, t, 2, &im);
    check(im.IsHit(10, 10), "sun off: back to the headlight");
}

// 안티앨리어싱: 경계 픽셀이 두 색의 중간이 되고, 깊이/id 는 섞이지 않으며, 크기와 내부 변수는 그대로다
void testAntialias() {
    // 영상 가운데를 가르는 수직 경계: x = 2 의 벽 (y > 0 쪽만)  -> 영상 왼쪽 절반(센서 +y)만 흰색이고 오른쪽은 배경
    const BoxMesh wall(2.0f, 0.0f, -5.0f, 2.1f, 5.0f, 5.0f);
    CameraTarget t = wall.target(3, 1.0f, 1.0f, 1.0f);
    CameraSettings s = small(33, 8);   // 가운데 열(16)에 경계가 걸리도록 홀수 폭
    s.background[0] = s.background[1] = s.background[2] = 0.0f;
    Camera plain(s);
    CameraImage a;
    plain.Render(dhMat(), &t, 1, &a);
    s.antialias = 3;
    Camera smooth(s);
    CameraImage b;
    smooth.Render(dhMat(), &t, 1, &b);
    check(b.width == 33 && b.height == 8 && b.bgr.size() == 33 * 8 * 3 && b.depth.size() == 33 * 8, "antialiased image has the requested size");
    check(near(b.intrinsics.fx, a.intrinsics.fx, 1e-4) && near(b.intrinsics.cx, a.intrinsics.cx, 1e-4), "intrinsics are those of the final image");
    // 경계는 가운데 열 (u = 16, 중심이 정확히 y = 0): 하위 픽셀의 절반이 벽
    const size_t row = 4 * 33;
    int edgeG = -1;
    bool hasMid = false;
    for (int u = 0; u < 33; u++) {
        const int g = b.bgr[(row + static_cast<size_t>(u)) * 3 + 1];
        if (g > 0 && g < 200) {
            hasMid = true;
            edgeG = g;
        }
    }
    check(hasMid, "the edge pixel is a blend of wall and background");
    (void)edgeG;
    int aMid = 0;
    for (int u = 0; u < 33; u++) {
        const int g = a.bgr[(row + static_cast<size_t>(u)) * 3 + 1];
        if (g > 0 && g < 200) aMid++;
    }
    check(aMid == 0, "without antialiasing every pixel is either wall or background");
    // 깊이는 섞이지 않는다: 벽이면 정확히 2.0, 아니면 0
    bool depthOk = true;
    for (size_t i = 0; i < b.depth.size(); i++) depthOk = depthOk && (b.depth[i] == 0.0f || near(b.depth[i], 2.0, 1e-4));
    check(depthOk, "depth is never blended at edges");
    // 안쪽(경계에서 먼 곳)은 같은 색
    check(a.bgr[(row + 2) * 3 + 1] == b.bgr[(row + 2) * 3 + 1] && a.bgr[(row + 30) * 3 + 1] == b.bgr[(row + 30) * 3 + 1], "away from edges the image is unchanged");
    s.antialias = 5;
    check(!Camera(s).Valid(), "antialias above 4 is invalid");
    s.antialias = 0;
    check(!Camera(s).Valid(), "antialias 0 is invalid");
    s.antialias = 4;
    s.width = 5000;
    check(!Camera(s).Valid(), "the supersampled size is limited too");
    // 잡음은 안티앨리어싱과 함께여도 반복 가능하다
    s = small(16, 12);
    s.antialias = 2;
    s.noiseStdDev = 0.01f;
    s.seed = 9;
    const BoxMesh wall2(3.0f, -5.0f, -5.0f, 3.1f, 5.0f, 5.0f);
    CameraTarget t2 = wall2.target(1, 1, 1, 1);
    Camera n1(s), n2(s);
    CameraImage i1, i2;
    n1.Render(dhMat(), &t2, 1, &i1);
    n2.Render(dhMat(), &t2, 1, &i2);
    bool same = true, noisy = false;
    for (size_t i = 0; i < i1.depth.size(); i++) {
        same = same && i1.depth[i] == i2.depth[i];
        noisy = noisy || Abs(i1.depth[i] - 3.0f) > 1e-6f;
    }
    check(same && noisy, "noise with antialiasing is repeatable and nonzero");
}

void testDistortion() {
    // 1) 왜곡 -> 되돌리기 왕복
    const float sets[4][5] = {{-0.3f, 0.1f, 0, 0, 0}, {0.2f, -0.05f, 0.001f, -0.002f, 0.01f}, {0, 0, 0.01f, 0.02f, 0}, {-0.25f, 0.08f, 0.003f, 0.001f, -0.01f}};
    bool roundTrip = true;
    for (int k = 0; k < 4; k++)
        for (float x = -0.8f; x <= 0.8f; x += 0.2f)
            for (float y = -0.6f; y <= 0.6f; y += 0.2f) {
                float xd, yd, bx = 0, by = 0;
                DistortNormalized(sets[k], x, y, &xd, &yd);
                roundTrip = roundTrip && UndistortNormalized(sets[k], xd, yd, &bx, &by) && near(bx, x, 1e-4) && near(by, y, 1e-4);
            }
    check(roundTrip, "undistort inverts distort");
    const float zero[5] = {0, 0, 0, 0, 0};
    float zx = 0, zy = 0;
    DistortNormalized(zero, 0.3f, -0.2f, &zx, &zy);
    check(zx == 0.3f && zy == -0.2f, "zero coefficients leave points alone");

    // 2) 세로로 선 막대(센서 +y 쪽, 영상 왼쪽): 왜곡이 없으면 모든 행에서 같은 열. 통 모양(k1 < 0)이면 위쪽 행에서 가운데 쪽으로 휜다. 실패 모양(k1 > 0)이면 바깥쪽으로.
    const BoxMesh bar(2.0f, 0.9f, -5.0f, 2.1f, 1.0f, 5.0f);
    CameraTarget t = bar.target(5, 1.0f, 1.0f, 1.0f);
    CameraSettings s = small(120, 90);
    s.background[0] = s.background[1] = s.background[2] = 0.0f;
    struct Col {
        static int Find(const CameraImage& im, int row) {   // 막대의 가운데 열 (흰 픽셀들의 평균)
            int sum = 0, n = 0;
            for (int u = 0; u < im.width; u++)
                if (im.IsHit(u, row)) {
                    sum += u;
                    n++;
                }
            return n ? sum / n : -1;
        }
    };
    Camera plain(s);
    CameraImage a;
    plain.Render(dhMat(), &t, 1, &a);
    const int aMid = Col::Find(a, 45), aTop = Col::Find(a, 2);
    check(aMid >= 0 && aMid == aTop, "without distortion the bar is vertical in the image");
    check(!plain.Settings().HasDistortion(), "default settings have no distortion");

    s.distortion[0] = -0.4f;
    Camera barrel(s);
    CameraImage b;
    barrel.Render(dhMat(), &t, 1, &b);
    const int bMid = Col::Find(b, 45), bTop = Col::Find(b, 2);
    check(bMid >= 0 && bTop >= 0, "the bar is visible with barrel distortion");
    check(Abs(60.0 - bTop) < Abs(60.0 - bMid), "barrel distortion bends the bar toward the center at the top");
    check(near(b.distortion[0], -0.4, 1e-6) && near(b.intrinsics.fx, a.intrinsics.fx, 1e-4), "image carries the coefficients and the same K");

    s.distortion[0] = 0.3f;
    Camera pin(s);
    CameraImage c;
    pin.Render(dhMat(), &t, 1, &c);
    const int cMid = Col::Find(c, 45), cTop = Col::Find(c, 2);
    check(cMid >= 0 && (cTop < 0 || Abs(60.0 - cTop) > Abs(60.0 - cMid)), "pincushion distortion bends the bar away from the center at the top");

    // 3) 깊이는 그대로 z-depth (막대는 x = 2 평면): 왜곡을 되돌린 PixelPoint 가 막대 위의 점을 돌려준다
    s.distortion[0] = -0.4f;
    Camera barrel2(s);
    CameraImage d;
    barrel2.Render(dhMat(), &t, 1, &d);
    bool depthOk = true, pointOk = true;
    int hits = 0;
    for (int v = 0; v < d.height; v++)
        for (int u = 0; u < d.width; u++)
            if (d.IsHit(u, v)) {
                hits++;
                depthOk = depthOk && d.Depth(u, v) > 1.999f && d.Depth(u, v) < 2.101f;   // 막대는 x = 2 ~ 2.1
                dhVector p;
                pointOk = pointOk && d.PixelPoint(u, v, &p) && p.x > 1.999f && p.x < 2.101f && p.y > 0.88f && p.y < 1.02f;
            }
    check(hits > 0 && depthOk, "depth stays the distance along the sensor x axis");
    check(pointOk, "PixelPoint undoes the distortion");

    // 4) 접선 왜곡도 영상을 바꾼다
    s.distortion[0] = 0;
    s.distortion[2] = 0.05f;
    Camera tang(s);
    CameraImage e;
    tang.Render(dhMat(), &t, 1, &e);
    check(Col::Find(e, 2) != aTop || Col::Find(e, 80) != Col::Find(a, 80), "tangential distortion changes the image");

    // 5) 너무 큰 왜곡: 되돌리지 못하는 픽셀은 못 본 것으로 두고, 죽지 않는다
    s.distortion[2] = 0;
    s.distortion[0] = -5.0f;
    Camera extreme(s);
    CameraImage f;
    extreme.Render(dhMat(), &t, 1, &f);
    check(f.width == 120 && f.height == 90 && f.bgr.size() == 120 * 90 * 3, "extreme distortion still renders");

    // 6) 안티앨리어싱과 함께
    s.distortion[0] = -0.4f;
    s.antialias = 2;
    Camera aa(s);
    CameraImage g;
    aa.Render(dhMat(), &t, 1, &g);
    check(g.width == 120 && g.height == 90 && near(g.distortion[0], -0.4, 1e-6) && Col::Find(g, 2) >= 0, "distortion works with antialiasing");

    // 7) 유한하지 않은 계수는 올바르지 않다
    s.antialias = 1;
    s.distortion[1] = HUGE_VALF;
    check(!Camera(s).Valid(), "non-finite coefficients are invalid");
}

void testNoise() {
    CameraSettings d;   // 기본 설정에는 잡음이 있다
    check(d.colorNoiseStdDev > 0.0f && d.noiseStdDev > 0.0f && d.HasNoise(), "noise is on by default");
    const BoxMesh wall(2.0f, -5.0f, -5.0f, 2.1f, 5.0f, 5.0f);
    CameraTarget t = wall.target(1, 0.5f, 0.5f, 0.5f);
    CameraSettings s = small(40, 30);
    s.noiseStdDev = 0.0f;
    s.colorNoiseStdDev = 0.0f;
    Camera clean(s);
    CameraImage c0;
    clean.Render(dhMat(), &t, 1, &c0);
    // 색 잡음: 매끈한 벽이 지글거린다. 평균은 그대로, 표준편차는 설정값 근처
    s.colorNoiseStdDev = 0.02f;
    Camera noisy(s);
    CameraImage n1, n2;
    noisy.Render(dhMat(), &t, 1, &n1);
    noisy.Render(dhMat(), &t, 1, &n2);
    double sum = 0.0, sq = 0.0, sum0 = 0.0;
    size_t diff12 = 0;
    for (size_t i = 0; i < n1.bgr.size(); i++) {
        const double e = static_cast<double>(n1.bgr[i]) - static_cast<double>(c0.bgr[i]);
        sum += e;
        sq += e * e;
        sum0 += c0.bgr[i];
        if (n1.bgr[i] != n2.bgr[i]) diff12++;
    }
    const double cnt = static_cast<double>(n1.bgr.size()), mean = sum / cnt, sd = Sqrt(static_cast<float>(sq / cnt - mean * mean));
    check(Abs(mean) < 1.0 && sd > 3.0 && sd < 8.0, "color noise has about the configured spread (0.02 of 255 = 5)");
    check(diff12 > n1.bgr.size() / 4, "the noise changes from frame to frame");
    bool depthSame = true;
    for (size_t i = 0; i < c0.depth.size(); i++) depthSame = depthSame && c0.depth[i] == n1.depth[i];
    check(depthSame, "color noise leaves depth alone");
    // 깊이 잡음
    s.colorNoiseStdDev = 0.0f;
    s.noiseStdDev = 0.01f;
    Camera dn(s);
    CameraImage d1;
    dn.Render(dhMat(), &t, 1, &d1);
    double dsum = 0.0, dsq = 0.0;
    size_t hit = 0;
    for (size_t i = 0; i < d1.depth.size(); i++)
        if (d1.targetIds[i] >= 0) {
            const double e = static_cast<double>(d1.depth[i]) - static_cast<double>(c0.depth[i]);
            dsum += e;
            dsq += e * e;
            hit++;
        }
    const double dm = dsum / static_cast<double>(hit), dsd = Sqrt(static_cast<float>(dsq / static_cast<double>(hit) - dm * dm));
    check(hit > 100 && Abs(dm) < 0.003 && dsd > 0.006 && dsd < 0.015, "depth noise has about the configured spread (10 mm)");
    // 0 이면 없다, 안티앨리어싱과 함께여도 한 번만 더한다
    s.noiseStdDev = 0.0f;
    s.colorNoiseStdDev = 0.0f;
    Camera off(s);
    CameraImage o1, o2;
    off.Render(dhMat(), &t, 1, &o1);
    off.Render(dhMat(), &t, 1, &o2);
    check(o1.bgr == o2.bgr && o1.bgr == c0.bgr, "zero noise gives identical frames");
    s.colorNoiseStdDev = 0.02f;
    s.antialias = 2;
    Camera aa(s);
    CameraImage a1;
    aa.Render(dhMat(), &t, 1, &a1);
    size_t changed = 0;
    for (size_t i = 0; i < a1.bgr.size(); i++)
        if (a1.bgr[i] != c0.bgr[i]) changed++;
    check(a1.width == 40 && changed > a1.bgr.size() / 4, "noise works with antialiasing");
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
    testSun();
    testAntialias();
    testDistortion();
    testNoise();
    testEdgeCases();
    if (g_fail) {
        printf("test_ccamera: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_ccamera: all passed\n");
    return 0;
}
