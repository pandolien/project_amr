#ifndef CORE_QT_LIDARPLOT_H
#define CORE_QT_LIDARPLOT_H

#include <QWidget>

#include "CSensor/Lidar.h"

// 라이다 스캔 그래프 (QPainter 만 쓴다, 추가 라이브러리 없음). 위쪽: 위에서 본 지도(맞은 점, 거리가 멀수록 진하게),
// 아래쪽: 방위각에 따른 거리. 3D 라이다면 고각이 0 에 가장 가까운 채널을 아래쪽 그래프에 그린다.
class LidarPlot : public QWidget {
    Q_OBJECT
public:
    explicit LidarPlot(QWidget* parent = nullptr);
    // origin: 센서 원점(월드), heading: 센서 +x 가 월드 xy 평면에서 향하는 각 (rad)
    void SetScan(const simul::sensor::LidarScan& scan, const dhVector& origin, float heading);
    QSize sizeHint() const override { return QSize(420, 520); }

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    simul::sensor::LidarScan scan_;
    dhVector origin_;
    float heading_;
    bool has_;
};

#endif  // CORE_QT_LIDARPLOT_H
