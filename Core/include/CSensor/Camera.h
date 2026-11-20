#ifndef CORE_CSENSOR_CAMERA_H
#define CORE_CSENSOR_CAMERA_H

#include <stddef.h>
#include <stdint.h>

#include "3d/dhMat.h"
#include "3d/dhVector.h"
#include "CBase/Array.h"
#include "CSensor/Lidar.h"

// 핀홀 카메라 시뮬레이션 (CPU 광선 추적). 픽셀마다 광선 하나를 쏴서 가장 가까운 면의 색과 깊이를 잰다.
// 좌표계: 라이다와 같은 센서 좌표계. +x 가 앞(카메라가 보는 방향), +y 가 왼쪽, +z 가 위 (ROS body frame).
//   영상의 u 는 오른쪽(= 센서 -y), v 는 아래쪽(= 센서 -z) 으로 커진다. 원점은 영상의 왼쪽 위. 픽셀 (u, v) 의 중심은 (u + 0.5, v + 0.5).
//   OpenCV/ROS 광학 좌표계(z 가 앞, x 가 오른쪽, y 가 아래)와는 축이 다르다. 영상 자체는 OpenCV 규약 그대로다.
// 컬러 영상은 BGR 순서 8비트 (OpenCV 의 CV_8UC3 과 같은 배치: 픽셀마다 B, G, R, 행 우선, 행 사이 빈틈 없음 = stride 는 width*3).
//   cv::Mat(height, width, CV_8UC3, image.bgr.data()) 로 복사 없이 감쌀 수 있다.
// 깊이 영상은 32비트 실수 (CV_32FC1 배치), 단위 m. 값은 센서 +x 방향 거리(z-depth)이고 광선을 따라 잰 거리가 아니다.
//   아무것도 안 맞았거나 near/far 밖이면 0.
// 색: 메시마다 하나의 기본 색에 카메라 방향 조명(밝기 = 0.3 + 0.7 * |면 법선 · 광선 방향|)을 곱한다. 텍스처와 정점 색은 아직 쓰지 않는다.
//   SetSun 으로 태양을 주면 태양 조명으로 바뀐다: 밝기 = 주변광 + 태양 * max(면 법선 · 태양 방향, 0), 그림자는 태양 쪽으로 광선을 하나 더 쏴서 가려지면 태양을 뺀다 (반사 없음).
namespace simul {
namespace sensor {

struct CameraSettings {
    int width;           // 가로 픽셀 수
    int height;          // 세로 픽셀 수
    float horizontalFov; // 가로 시야각 (rad). 픽셀은 정사각형이라 세로 시야각은 width/height 비율로 정해진다
    float nearClip;      // 이보다 가까운 것은 안 보인다 (m, 깊이 기준)
    float farClip;       // 이보다 먼 것은 안 보인다 (m, 깊이 기준)
    float noiseStdDev;   // 깊이에 더할 가우시안 잡음의 표준편차 (m). 0 이면 없음. 기본 0.005 (5 mm). 물체에 맞은 픽셀에만 더한다
    float colorNoiseStdDev;  // 색에 더할 가우시안 잡음의 표준편차 (0..1, 만점 대비 비율). 0 이면 없음. 기본 0.02 (8비트로 약 5). 모든 픽셀의 B, G, R 에 따로따로 더한다
    unsigned seed;       // 잡음의 난수 시드. 같은 Camera 로 계속 찍으면 프레임마다 다른 잡음이 나온다 (실제 센서처럼)
    float background[3]; // 아무것도 안 맞은 픽셀의 색 (R, G, B 0..1)
    int antialias;       // 안티앨리어싱: 픽셀을 가로세로 antialias 배로 나눠 찍어 평균한다 (1 = 끔, 2 = 4개, 3 = 9개, 4 = 16개, 최대 4). 시간은 antialias^2 배 걸린다.
                         //   색만 평균하고, 깊이와 targetIds 는 픽셀 가운데에 가장 가까운 하위 픽셀의 값을 쓴다 (경계에서 깊이가 섞여 허공에 뜬 점이 생기지 않게).
    float distortion[5]; // 렌즈 왜곡 계수 k1, k2, p1, p2, k3 (OpenCV/ROS plumb_bob 과 같은 순서와 뜻). 모두 0 이면 왜곡 없음.
                         //   영상(왜곡된 영상)의 픽셀 -> 정규화 좌표 (x_d, y_d) = ((u-cx)/fx, (v-cy)/fy) 이고, 왜곡 없는 정규화 좌표 (x, y) 와는
                         //   r2 = x^2 + y^2, x_d = x (1 + k1 r2 + k2 r2^2 + k3 r2^3) + 2 p1 x y + p2 (r2 + 2 x^2), y_d = y (1 + ...) + p1 (r2 + 2 y^2) + 2 p2 x y 로 이어진다.
                         //   k1 < 0 은 통 모양(barrel), k1 > 0 은 실패 모양(pincushion). 카메라 행렬 K 는 왜곡된 영상 기준이라 ROS CameraInfo 의 K, D 에 그대로 쓰면 된다.
    CameraSettings()
        : width(320), height(240), horizontalFov(static_cast<float>(60.0 * dhPI / 180.0)), nearClip(0.05f), farClip(10.0f), noiseStdDev(0.005f), colorNoiseStdDev(0.02f), seed(1u), antialias(1) {
        background[0] = 0.12f;
        background[1] = 0.14f;
        background[2] = 0.18f;
        for (int i = 0; i < 5; i++) distortion[i] = 0.0f;
    }
    bool HasNoise() const { return noiseStdDev > 0.0f || colorNoiseStdDev > 0.0f; }
    bool HasDistortion() const { return distortion[0] != 0.0f || distortion[1] != 0.0f || distortion[2] != 0.0f || distortion[3] != 0.0f || distortion[4] != 0.0f; }
};

// 렌즈 왜곡 (k1, k2, p1, p2, k3): 왜곡 없는 정규화 좌표 -> 왜곡된 정규화 좌표.
void DistortNormalized(const float dist[5], float x, float y, float* xd, float* yd);
// 왜곡된 정규화 좌표 -> 왜곡 없는 정규화 좌표 (반복법, cv::undistortPoints 와 같은 방식). 수렴 못 하면(왜곡이 너무 커서 한 점에 여러 곳이 겹치는 영역 등) false.
bool UndistortNormalized(const float dist[5], float xd, float yd, float* x, float* y);

// 태양 조명 (SetSun). direction 은 월드 좌표에서 표면 -> 태양 방향. CEnv/Sun.h 의 SunState::direction 을 그대로 넣으면 된다.
struct SunLight {
    bool enabled;
    float direction[3];   // 월드, 단위 벡터
    float intensity;      // 태양 빛 세기 0..1 (해가 졌으면 0)
    float ambient;        // 주변광(하늘) 밝기 0..1
    float color[3];       // 태양 빛 색 R, G, B
    float sky[3];         // 하늘색 R, G, B: 아무것도 안 맞은 픽셀의 배경으로 쓴다 (설정의 background 대신)
    bool shadows;         // 그림자 광선 (끄면 빠르지만 그림자가 없다)
    SunLight() : enabled(false), intensity(1.0f), ambient(1.0f), shadows(true) {
        direction[0] = 0.0f; direction[1] = 0.0f; direction[2] = 1.0f;
        color[0] = color[1] = color[2] = 1.0f;
        sky[0] = 0.12f; sky[1] = 0.14f; sky[2] = 0.18f;
    }
};

// 핀홀 내부 변수 (OpenCV 카메라 행렬 K = [fx 0 cx; 0 fy cy; 0 0 1] 과 같은 뜻, 단위 픽셀)
struct CameraIntrinsics {
    float fx, fy, cx, cy;
    CameraIntrinsics() : fx(0), fy(0), cx(0), cy(0) {}
};

// 카메라가 볼 메시 하나. mesh 는 라이다와 같은 구조 (정점/인덱스/월드 변환/BVH), color 는 기본 색 R, G, B (0..1).
struct CameraTarget {
    LidarTarget mesh;
    float color[3];
    CameraTarget() { color[0] = color[1] = color[2] = 0.75f; }
};

struct CameraImage {
    int width, height;
    Array<uint8_t> bgr;     // width * height * 3, BGR
    Array<float> depth;     // width * height, m (못 본 곳은 0)
    Array<int> targetIds;   // width * height, 맞은 메시의 id (mesh.id), 못 맞으면 -1
    CameraIntrinsics intrinsics;
    float distortion[5];    // 이 영상을 찍을 때의 왜곡 계수 (k1, k2, p1, p2, k3)
    CameraImage() : width(0), height(0) { for (int i = 0; i < 5; i++) distortion[i] = 0.0f; }
    bool IsHit(int u, int v) const { return targetIds[static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u)] >= 0; }
    float Depth(int u, int v) const { return depth[static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u)]; }
    // 픽셀 (u, v) 가 본 점을 센서 좌표로 (깊이가 0 이면 false)
    bool PixelPoint(int u, int v, dhVector* p) const;
};

class Camera {
public:
    explicit Camera(const CameraSettings& settings = CameraSettings());
    const CameraSettings& Settings() const { return settings_; }
    void SetSettings(const CameraSettings& s);  // 난수도 새 시드로 다시 시작한다
    void SetSun(const SunLight& sun) { sun_ = sun; }   // 태양 조명 (enabled = false 면 카메라 방향 조명)
    const SunLight& Sun() const { return sun_; }
    bool Valid() const;                         // 크기, 시야각, near/far 가 올바른가
    CameraIntrinsics Intrinsics() const;
    float VerticalFov() const;                  // 세로 시야각 (rad)

    // 픽셀 (u, v) (소수 가능, 픽셀 중심은 +0.5) 를 지나는 센서 좌표의 광선 방향. x 성분이 1 이 되게 두어 t 가 곧 깊이다 (길이는 1 이 아님).
    // 렌즈 왜곡을 반영한다. 왜곡을 되돌릴 수 없는 픽셀이면(PixelRay 참고) 왜곡을 뺀 방향을 돌려준다.
    dhVector PixelDirection(float u, float v) const;
    // 같은 값을 주되 왜곡을 되돌리지 못하면 false (그 픽셀에서는 광선을 쏘지 않는다)
    bool PixelRay(float u, float v, dhVector* dir) const;

    // sensorPose: 센서 좌표 -> 월드. targets: 볼 메시들 (센서가 붙은 링크 자신은 빼야 자기 몸이 안 보인다).
    void Render(const dhMat& sensorPose, const CameraTarget* targets, size_t targetCount, CameraImage* out);

private:
    unsigned nextRandom();
    float gaussian();
    void addColorNoise(CameraImage* out);
    CameraSettings settings_;
    SunLight sun_;
    unsigned rng_;
};

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_CAMERA_H
