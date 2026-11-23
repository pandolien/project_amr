#ifndef CORE_CSENSOR_ODOMETRY_H
#define CORE_CSENSOR_ODOMETRY_H

#include "CSensor/Encoder.h"

// 차동 2륜 AMR 의 휠 오도메트리: 좌우 바퀴의 회전 변화로 평면 위치 (x, y, yaw) 와 속도를 적분한다.
// 바퀴가 앞으로 구르는 방향이 + 가 되도록 EncoderSettings::invert 를 맞춰 둔다 (좌우 바퀴 축 방향이 반대인 모델이 흔하다).
// 적분은 중점법 (한 스텝 동안 직진 + 회전의 중간 방위로 이동). 바퀴 슬립은 없다고 본다 (실제와 어긋나는 것이 오도메트리 오차의 원인이다).
// 좌표계: 시작 자세 기준 평면 (x 앞, y 왼쪽, yaw 는 반시계 +, rad, (-pi, pi]).
namespace simul {
namespace sensor {

struct DiffDriveGeometry {
    double wheelRadius;  // m
    double trackWidth;   // 좌우 바퀴 중심 사이 거리 (윤거, m)
    DiffDriveGeometry() : wheelRadius(0.1), trackWidth(0.5) {}
};

struct Pose2D {
    double x, y, yaw;  // m, m, rad
    double v, w;       // 직진 속도 (m/s), 회전 속도 (rad/s). 직전 Update 구간의 평균
    Pose2D() : x(0), y(0), yaw(0), v(0), w(0) {}
};

class DiffDriveOdometry {
public:
    explicit DiffDriveOdometry(const DiffDriveGeometry& g = DiffDriveGeometry());
    const DiffDriveGeometry& Geometry() const { return geom_; }
    void Reset(double x = 0.0, double y = 0.0, double yaw = 0.0);  // 자세와 직전 바퀴 각도를 다시 시작한다
    const Pose2D& Pose() const { return pose_; }

    // 이 구간에 좌우 바퀴가 앞으로 돈 각도 (rad) 와 걸린 시간 (s)
    void Update(double dLeftAngle, double dRightAngle, double dt);

    // 엔코더 측정값 (position 은 바퀴 각도) 으로 갱신한다. 첫 호출은 기준만 잡고 움직이지 않는다.
    // 반환: 갱신했으면 true, 첫 호출이거나 측정이 유효하지 않으면 false
    bool Update(const EncoderReading& left, const EncoderReading& right, double dt);

private:
    DiffDriveGeometry geom_;
    Pose2D pose_;
    bool hasPrev_;
    double prevLeft_, prevRight_;
};

}  // namespace sensor
}  // namespace simul

#endif  // CORE_CSENSOR_ODOMETRY_H
