#ifndef CORE_QT_CAMERAVIEW_H
#define CORE_QT_CAMERAVIEW_H

#include <QImage>
#include <QWidget>

#include "CSensor/Camera.h"

// 카메라 영상 보기 (QPainter 만 쓴다). 컬러(BGR 영상을 그대로 보여준다) 또는 깊이(가까울수록 밝은 회색) 를 보여주고,
// 영상 위에서 마우스를 움직이면 그 픽셀의 (u, v), 깊이, 센서 좌표를 아래 줄에 보여준다.
class CameraView : public QWidget {
    Q_OBJECT
public:
    explicit CameraView(QWidget* parent = nullptr);
    void SetImage(const simul::sensor::CameraImage& image);
    void SetShowDepth(bool on);
    bool ShowDepth() const { return depth_; }
    void SetDepthRange(float nearClip, float farClip);   // 깊이 보기에서 밝기가 변하는 범위 (카메라의 near/far)
    QSize sizeHint() const override { return QSize(360, 300); }

    // BGR 영상 -> QImage (Format_RGB888 로 채널 순서를 바꿔 복사한다. Qt 5.12 에는 Format_BGR888 이 없다)
    static QImage ColorToQImage(const simul::sensor::CameraImage& image);
    // 깊이 영상 -> 8비트 회색 QImage: 가까울수록 밝고 (nearClip -> 255, farClip -> 1), 못 본 곳은 0
    static QImage DepthToQImage(const simul::sensor::CameraImage& image, float nearClip, float farClip);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void leaveEvent(QEvent* e) override;

private:
    QRect imageRect() const;
    void rebuild();
    simul::sensor::CameraImage image_;
    QImage shown_;
    bool depth_;
    float near_, far_;
    QString info_;
};

#endif  // CORE_QT_CAMERAVIEW_H
