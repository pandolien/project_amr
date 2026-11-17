#ifndef CORE_CSENSOR_LIDAR_H
#define CORE_CSENSOR_LIDAR_H

#include <stddef.h>
#include <stdint.h>

#include "3d/dhMat.h"
#include "3d/dhVector.h"
#include "CBase/Array.h"
#include "CCollision/Bvh.h"

// 2D/3D 라이다 시뮬레이션. 센서에서 광선 여러 개를 쏘고, 광선마다 메시와 가장 가까운 교차점까지의 거리를 잰다
// (광선-삼각형 교차는 CCollision/MeshRaycast 의 Möller-Trumbore).
// 광선 방향: 방위각 theta (센서 +x 에서 +z 둘레로 반시계), 고각 phi (xy 평면에서 위로)
//     d = (cos(phi) cos(theta), cos(phi) sin(theta), sin(phi))
// 2D 라이다 = 수직 채널이 1개(phi = 0), 3D 라이다 = 수직 채널이 여러 개. 단위는 m, rad (ROS LaserScan 과 같은 각도 규약).
namespace simul {
namespace sensor {

struct LidarSettings {
    float angleMin;        // 첫 광선의 방위각 (rad)
    float angleMax;        // 마지막 광선의 방위각 (rad)
    float angleIncrement;  // 방위각 사이의 간격 (rad)
    int channels;          // 수직 채널 수 (1 이면 2D 라이다)
    float elevationMin;    // 첫 채널의 고각 (rad). channels 가 1 이면 이 값 하나만 쓴다 (기본 0: 수평)
    float elevationMax;    // 마지막 채널의 고각 (rad). 채널은 이 사이에 같은 간격으로 놓인다
    float rangeMin;        // 이보다 가까운 것은 잡히지 않는다 (m)
    float rangeMax;        // 이보다 먼 것은 잡히지 않는다 (m)
    float noiseStdDev;     // 거리에 더할 가우시안 잡음의 표준편차 (m). 0 이면 잡음 없음
    unsigned seed;         // 잡음의 난수 시드 (같으면 같은 잡음)
    LidarSettings()
        : angleMin(static_cast<float>(-135.0 * dhPI / 180.0)), angleMax(static_cast<float>(135.0 * dhPI / 180.0)),
          angleIncrement(static_cast<float>(0.5 * dhPI / 180.0)), channels(1), elevationMin(0.0f), elevationMax(0.0f), rangeMin(0.05f), rangeMax(30.0f), noiseStdDev(0.0f), seed(1u) {}
};

// 라이다가 볼 메시 하나 (정점 배열은 복사하지 않고 가리키기만 한다. 스캔하는 동안 유효해야 한다).
struct LidarTarget {
    const float* positions;     // 정점마다 xyz 가 stride 개(float)씩 간격으로
    size_t vertexCount;
    const uint32_t* indices;    // 삼각형 인덱스 (3개씩)
    size_t indexCount;
    size_t stride;              // 3 = 빽빽한 xyz, 6 = 위치+법선 교차 배열
    dhMat world;                // 메시 좌표 -> 월드 (강체변환, Scale 없음)
    const TriangleBvh* bvh;     // 이 메시의 BVH (없으면 NULL). 있으면 광선 검사를 BVH 로 한다 (positions/indices 로 만든 것이어야 한다)
    int id;                     // 호출자가 붙이는 번호 (맞은 메시를 알려줄 때 돌려준다. 보통 링크 번호)
    bool hasBounds;             // 아래 경계 상자가 계산돼 있는가 (있으면 광선이 빗나가는 메시를 빨리 거른다)
    float boundsMin[3], boundsMax[3];  // 메시 좌표의 경계 상자
    LidarTarget() : positions(NULL), vertexCount(0), indices(NULL), indexCount(0), stride(3), bvh(NULL), id(-1), hasBounds(false) {
        boundsMin[0] = boundsMin[1] = boundsMin[2] = 0.0f;
        boundsMax[0] = boundsMax[1] = boundsMax[2] = 0.0f;
    }
    void ComputeBounds();  // 정점으로부터 경계 상자를 계산한다 (메시가 바뀔 때마다 한 번)
};

struct LidarScan {
    float angleMin, angleIncrement, rangeMin, rangeMax;
    size_t azimuthCount;       // 한 채널의 광선 수
    size_t channelCount;       // 수직 채널 수
    float elevationMin, elevationMax;
    // 아래 배열의 크기는 azimuthCount * channelCount. 광선 번호 i = channel * azimuthCount + 방위 번호
    Array<float> ranges;       // 광선마다 거리 (m). 아무것도 안 맞으면 무한대 (IsHit 로 확인)
    Array<int> targetIds;      // 맞은 메시의 id, 못 맞으면 -1
    Array<dhVector> points;    // 맞은 점의 월드 좌표 (못 맞은 광선은 값 없음)
    LidarScan() : angleMin(0), angleIncrement(0), rangeMin(0), rangeMax(0), azimuthCount(0), channelCount(0), elevationMin(0), elevationMax(0) {}
    size_t size() const { return ranges.size(); }
    bool IsHit(size_t i) const { return targetIds[i] >= 0; }
    size_t Channel(size_t i) const { return azimuthCount ? i / azimuthCount : 0; }
    float Angle(size_t i) const { return angleMin + angleIncrement * static_cast<float>(azimuthCount ? i % azimuthCount : 0); }  // 방위각
    float Elevation(size_t channel) const {
        return channelCount > 1 ? elevationMin + (elevationMax - elevationMin) * static_cast<float>(channel) / static_cast<float>(channelCount - 1) : elevationMin;
    }
};

class Lidar {
public:
    explicit Lidar(const LidarSettings& settings = LidarSettings());
    const LidarSettings& Settings() const { return settings_; }
    void SetSettings(const LidarSettings& s);  // 난수도 새 시드로 다시 시작한다

    size_t AzimuthCount() const;               // 한 채널의 광선 수 = (angleMax - angleMin) / angleIncrement + 1 (설정이 이상하면 0)
    size_t RayCount() const;                   // 전체 광선 수 = AzimuthCount * channels
    float RayAngle(size_t azimuth) const { return settings_.angleMin + settings_.angleIncrement * static_cast<float>(azimuth); }
    float ChannelElevation(size_t channel) const;

    // sensorPose: 센서 좌표 -> 월드. targets: 볼 메시들 (센서가 붙은 로봇 자신은 빼야 자기 몸이 안 잡힌다).
    void Scan(const dhMat& sensorPose, const LidarTarget* targets, size_t targetCount, LidarScan* out);

private:
    float gaussian();  // 표준정규분포 난수 (Box-Muller)
    unsigned nextRandom();
    LidarSettings settings_;
    unsigned rng_;
};

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_LIDAR_H
