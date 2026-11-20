#include "CoreQt/CameraView.h"

#include <QMouseEvent>
#include <QPainter>

#include <math.h>

using namespace simul::sensor;

CameraView::CameraView(QWidget* parent) : QWidget(parent), depth_(false), near_(0.05f), far_(10.0f) {
    setMouseTracking(true);
    setMinimumSize(160, 120);
}

QImage CameraView::ColorToQImage(const CameraImage& im) {
    if (im.width <= 0 || im.height <= 0 || im.bgr.size() != static_cast<size_t>(im.width) * static_cast<size_t>(im.height) * 3) return QImage();
    QImage out(im.width, im.height, QImage::Format_RGB888);
    for (int y = 0; y < im.height; y++) {
        uchar* d = out.scanLine(y);
        const uint8_t* s = &im.bgr[static_cast<size_t>(y) * static_cast<size_t>(im.width) * 3];
        for (int x = 0; x < im.width; x++) {
            d[x * 3] = s[x * 3 + 2];
            d[x * 3 + 1] = s[x * 3 + 1];
            d[x * 3 + 2] = s[x * 3];
        }
    }
    return out;
}

QImage CameraView::DepthToQImage(const CameraImage& im, float nearClip, float farClip) {
    if (im.width <= 0 || im.height <= 0 || im.depth.size() != static_cast<size_t>(im.width) * static_cast<size_t>(im.height)) return QImage();
    QImage out(im.width, im.height, QImage::Format_Grayscale8);
    const float span = farClip - nearClip > 1e-6f ? farClip - nearClip : 1.0f;
    for (int y = 0; y < im.height; y++) {
        uchar* d = out.scanLine(y);
        for (int x = 0; x < im.width; x++) {
            const float z = im.Depth(x, y);
            if (!(z > 0.0f)) {
                d[x] = 0;
                continue;
            }
            float t = 1.0f - (z - nearClip) / span;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            d[x] = static_cast<uchar>(1.0f + t * 254.0f);
        }
    }
    return out;
}

void CameraView::SetImage(const CameraImage& image) {
    image_ = image;
    rebuild();
    update();
}

void CameraView::SetShowDepth(bool on) {
    depth_ = on;
    rebuild();
    update();
}

void CameraView::SetDepthRange(float nearClip, float farClip) {
    near_ = nearClip;
    far_ = farClip;
    rebuild();
    update();
}

void CameraView::rebuild() {
    shown_ = depth_ ? DepthToQImage(image_, near_, far_) : ColorToQImage(image_);
}

QRect CameraView::imageRect() const {
    if (shown_.isNull()) return QRect();
    const int h = height() - 22;   // 아래 한 줄은 글자
    QSize s = shown_.size();
    s.scale(width(), h > 1 ? h : 1, Qt::KeepAspectRatio);
    return QRect((width() - s.width()) / 2, (h - s.height()) / 2, s.width(), s.height());
}

void CameraView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(24, 24, 28));
    const QRect r = imageRect();
    if (shown_.isNull()) {
        p.setPen(QColor(150, 150, 150));
        p.drawText(rect(), Qt::AlignCenter, QString::fromUtf8("영상 없음"));
        return;
    }
    p.drawImage(r, shown_);
    p.setPen(QColor(200, 200, 200));
    const QString head = QString::fromUtf8("%1 x %2  %3").arg(image_.width).arg(image_.height).arg(depth_ ? QString::fromUtf8("깊이 (밝을수록 가까움)") : QString::fromUtf8("컬러 (BGR)"));
    p.drawText(QRect(0, height() - 22, width(), 22), Qt::AlignVCenter | Qt::AlignLeft, info_.isEmpty() ? head : info_);
}

void CameraView::mouseMoveEvent(QMouseEvent* e) {
    const QRect r = imageRect();
    info_.clear();
    if (!r.isEmpty() && r.contains(e->pos()) && image_.width > 0) {
        const int u = (e->pos().x() - r.x()) * image_.width / r.width();
        const int v = (e->pos().y() - r.y()) * image_.height / r.height();
        if (u >= 0 && v >= 0 && u < image_.width && v < image_.height) {
            dhVector pt;
            if (image_.PixelPoint(u, v, &pt)) info_ = QString::fromUtf8("(%1, %2)  깊이 %3 m  점 (%4, %5, %6)").arg(u).arg(v).arg(image_.Depth(u, v), 0, 'f', 3).arg(pt.x, 0, 'f', 3).arg(pt.y, 0, 'f', 3).arg(pt.z, 0, 'f', 3);
            else info_ = QString::fromUtf8("(%1, %2)  안 보임").arg(u).arg(v);
        }
    }
    update();
}

void CameraView::leaveEvent(QEvent*) {
    info_.clear();
    update();
}
