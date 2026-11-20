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
// 색: 메시마다 하나의 기본 색에 머리 위 조명 대신 카메라 방향 조명(밝기 = 0.3 + 0.7 * |면 법선 · 광선 방향|)을 곱한다.
//   텍스처와 정점 색은 아직 쓰지 않는다. 그림자/반사 없음.
namespace simul {
namespace sensor {

struct CameraSettings {
    int width;           // 가로 픽셀 수
    int height;          // 세로 픽셀 수
    float horizontalFov; // 가로 시야각 (rad). 픽셀은 정사각형이라 세로 시야각은 width/height 비율로 정해진다
    float nearClip;      // 이보다 가까운 것은 안 보인다 (m, 깊이 기준)
    float farClip;       // 이보다 먼 것은 안 보인다 (m, 깊이 기준)
    float noiseStdDev;   // 깊이에 더할 가우시안 잡음의 표준편차 (m). 0 이면 없음. 색에는 더하지 않는다
    unsigned seed;       // 잡음의 난수 시드
    float background[3]; // 아무것도 안 맞은 픽셀의 색 (R, G, B 0..1)
    CameraSettings()
        : width(320), height(240), horizontalFov(static_cast<float>(60.0 * dhPI / 180.0)), nearClip(0.05f), farClip(10.0f), noiseStdDev(0.0f), seed(1u) {
        background[0] = 0.12f;
        background[1] = 0.14f;
        background[2] = 0.18f;
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
    CameraImage() : width(0), height(0) {}
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
    bool Valid() const;                         // 크기, 시야각, near/far 가 올바른가
    CameraIntrinsics Intrinsics() const;
    float VerticalFov() const;                  // 세로 시야각 (rad)

    // 픽셀 (u, v) (소수 가능, 픽셀 중심은 +0.5) 를 지나는 센서 좌표의 광선 방향. x 성분이 1 이 되게 두어 t 가 곧 깊이다 (길이는 1 이 아님).
    dhVector PixelDirection(float u, float v) const;

    // sensorPose: 센서 좌표 -> 월드. targets: 볼 메시들 (센서가 붙은 링크 자신은 빼야 자기 몸이 안 보인다).
    void Render(const dhMat& sensorPose, const CameraTarget* targets, size_t targetCount, CameraImage* out);

private:
    unsigned nextRandom();
    float gaussian();
    CameraSettings settings_;
    unsigned rng_;
};

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_CAMERA_H
