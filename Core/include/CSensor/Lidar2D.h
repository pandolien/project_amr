#ifndef CORE_CSENSOR_LIDAR2D_H
#define CORE_CSENSOR_LIDAR2D_H

#include <stddef.h>
#include <stdint.h>

#include "3d/dhMat.h"
#include "3d/dhVector.h"
#include "CBase/Array.h"

// 2D 라이다 시뮬레이션. 센서의 수평면(센서 좌표계의 xy 평면)에서 부채꼴로 광선을 쏘고, 광선마다 메시와 가장 가까운
// 교차점까지의 거리를 잰다 (광선-삼각형 교차는 CCollision/MeshRaycast 의 Möller-Trumbore).
// 각도는 센서의 +x 축에서 시작해 +z 축 둘레로 반시계 방향 (ROS LaserScan 과 같은 규약). 단위는 m, rad.
namespace amr {
namespace sensor {

struct LidarSettings {
    float angleMin;        // 첫 광선의 각도 (rad)
    float angleMax;        // 마지막 광선의 각도 (rad)
    float angleIncrement;  // 광선 사이의 각도 (rad)
    float rangeMin;        // 이보다 가까운 것은 잡히지 않는다 (m)
    float rangeMax;        // 이보다 먼 것은 잡히지 않는다 (m)
    float noiseStdDev;     // 거리에 더할 가우시안 잡음의 표준편차 (m). 0 이면 잡음 없음
    unsigned seed;         // 잡음의 난수 시드 (같으면 같은 잡음)
    LidarSettings()
        : angleMin(static_cast<float>(-135.0 * dhPI / 180.0)), angleMax(static_cast<float>(135.0 * dhPI / 180.0)),
          angleIncrement(static_cast<float>(0.5 * dhPI / 180.0)), rangeMin(0.05f), rangeMax(30.0f), noiseStdDev(0.0f), seed(1u) {}
};

// 라이다가 볼 메시 하나 (정점 배열은 복사하지 않고 가리키기만 한다. 스캔하는 동안 유효해야 한다).
struct LidarTarget {
    const float* positions;     // 정점마다 xyz 가 stride 개(float)씩 간격으로
    size_t vertexCount;
    const uint32_t* indices;    // 삼각형 인덱스 (3개씩)
    size_t indexCount;
    size_t stride;              // 3 = 빽빽한 xyz, 6 = 위치+법선 교차 배열
    dhMat world;                // 메시 좌표 -> 월드 (강체변환, Scale 없음)
    int id;                     // 호출자가 붙이는 번호 (맞은 메시를 알려줄 때 돌려준다. 보통 링크 번호)
    bool hasBounds;             // 아래 경계 상자가 계산돼 있는가 (있으면 광선이 빗나가는 메시를 빨리 거른다)
    float boundsMin[3], boundsMax[3];  // 메시 좌표의 경계 상자
    LidarTarget() : positions(NULL), vertexCount(0), indices(NULL), indexCount(0), stride(3), id(-1), hasBounds(false) {
        boundsMin[0] = boundsMin[1] = boundsMin[2] = 0.0f;
        boundsMax[0] = boundsMax[1] = boundsMax[2] = 0.0f;
    }
    void ComputeBounds();  // 정점으로부터 경계 상자를 계산한다 (메시가 바뀔 때마다 한 번)
};

struct LidarScan {
    float angleMin, angleIncrement, rangeMin, rangeMax;
    Array<float> ranges;       // 광선마다 거리 (m). 아무것도 안 맞으면 무한대 (IsHit 로 확인)
    Array<int> targetIds;      // 맞은 메시의 id, 못 맞으면 -1
    Array<dhVector> points;    // 맞은 점의 월드 좌표 (못 맞은 광선은 값 없음)
    LidarScan() : angleMin(0), angleIncrement(0), rangeMin(0), rangeMax(0) {}
    size_t size() const { return ranges.size(); }
    bool IsHit(size_t i) const { return targetIds[i] >= 0; }
    float Angle(size_t i) const { return angleMin + angleIncrement * static_cast<float>(i); }
};

class Lidar2D {
public:
    explicit Lidar2D(const LidarSettings& settings = LidarSettings());
    const LidarSettings& Settings() const { return settings_; }
    void SetSettings(const LidarSettings& s);  // 난수도 새 시드로 다시 시작한다

    size_t RayCount() const;                   // 광선 수 = (angleMax - angleMin) / angleIncrement + 1 (설정이 이상하면 0)
    float RayAngle(size_t i) const { return settings_.angleMin + settings_.angleIncrement * static_cast<float>(i); }

    // sensorPose: 센서 좌표 -> 월드. targets: 볼 메시들 (센서가 붙은 로봇 자신은 빼야 자기 몸이 안 잡힌다).
    void Scan(const dhMat& sensorPose, const LidarTarget* targets, size_t targetCount, LidarScan* out);

private:
    float gaussian();  // 표준정규분포 난수 (Box-Muller)
    unsigned nextRandom();
    LidarSettings settings_;
    unsigned rng_;
};

}  // namespace sensor
}  // namespace amr

#endif  // CORE_CSENSOR_LIDAR2D_H
