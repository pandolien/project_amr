#include "CoreQt/LidarPlot.h"

#include <math.h>

#include <QPainter>
#include <QPainterPath>

namespace {
const QColor kSurface(0xfc, 0xfc, 0xfb);
const QColor kInk(0x0b, 0x0b, 0x0b);
const QColor kInk2(0x52, 0x51, 0x4e);
const QColor kGrid(0xe6, 0xe5, 0xe1);
const QColor kSeries(0x2a, 0x78, 0xd6);

// 거리 -> 색 (한 색조, 가까울수록 연하게)
QColor RangeColor(float t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    const int r0 = 0xbc, g0 = 0xd6, b0 = 0xf5, r1 = 0x12, g1 = 0x40, b1 = 0x7f;
    return QColor(r0 + static_cast<int>((r1 - r0) * t), g0 + static_cast<int>((g1 - g0) * t), b0 + static_cast<int>((b1 - b0) * t));
}

// 보기 좋은 눈금 간격 (1, 2, 5 x 10^n)
double NiceStep(double span, int maxTicks) {
    if (span <= 0) return 1;
    const double raw = span / maxTicks;
    const double p = pow(10.0, floor(log10(raw)));
    const double f = raw / p;
    return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p;
}
}  // namespace

LidarPlot::LidarPlot(QWidget* parent) : QWidget(parent), heading_(0.0f), has_(false) {
    setMinimumSize(280, 340);
}

void LidarPlot::SetScan(const amr::sensor::LidarScan& scan, const dhVector& origin, float heading) {
    scan_ = scan;
    origin_ = origin;
    heading_ = heading;
    has_ = true;
    update();
}

void LidarPlot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), kSurface);
    QFont f = font();
    f.setPointSize(9);
    p.setFont(f);
    if (!has_ || scan_.size() == 0) {
        p.setPen(kInk2);
        p.drawText(rect(), Qt::AlignCenter, QString::fromUtf8("라이다 스캔 없음"));
        return;
    }

    const int W = width(), H = height();
    const int mapH = static_cast<int>(H * 0.56);
    const QRectF mapBox(44, 26, W - 44 - 14, mapH - 26 - 30);
    const QRectF plotBox(44, mapH + 26, W - 44 - 14, H - mapH - 26 - 38);

    p.setPen(kInk);
    p.drawText(QPointF(10, 16), QString::fromUtf8("위에서 본 스캔 (x, y: m)"));
    p.drawText(QPointF(10, mapH + 16), QString::fromUtf8("방위각에 따른 거리"));

    // ---- 지도: 맞은 점의 범위로 축을 잡고, x, y 눈금 간격(m)을 같게 한다
    double minX = origin_.x, maxX = origin_.x, minY = origin_.y, maxY = origin_.y, maxR = 0.0;
    for (size_t i = 0; i < scan_.size(); i++) {
        if (!scan_.IsHit(i)) continue;
        const dhVector& q = scan_.points[i];
        if (q.x < minX) minX = q.x;
        if (q.x > maxX) maxX = q.x;
        if (q.y < minY) minY = q.y;
        if (q.y > maxY) maxY = q.y;
        if (scan_.ranges[i] > maxR) maxR = scan_.ranges[i];
    }
    double spanX = maxX - minX, spanY = maxY - minY;
    if (spanX < 1.0) spanX = 1.0;
    if (spanY < 1.0) spanY = 1.0;
    const double cx = (minX + maxX) * 0.5, cy = (minY + maxY) * 0.5;
    double scale = qMin(mapBox.width() / (spanX * 1.1), mapBox.height() / (spanY * 1.1));  // 픽셀/m
    if (scale <= 0) scale = 1;
    const QPointF mc = mapBox.center();
    auto toPx = [&](double x, double y) { return QPointF(mc.x() + (x - cx) * scale, mc.y() - (y - cy) * scale); };

    p.save();
    p.setClipRect(mapBox);
    const double step = NiceStep(qMax(mapBox.width(), mapBox.height()) / scale, 6);
    p.setPen(QPen(kGrid, 1));
    for (double x = ceil((cx - mapBox.width() / 2 / scale) / step) * step; x <= cx + mapBox.width() / 2 / scale; x += step)
        p.drawLine(toPx(x, cy - mapBox.height() / 2 / scale), toPx(x, cy + mapBox.height() / 2 / scale));
    for (double y = ceil((cy - mapBox.height() / 2 / scale) / step) * step; y <= cy + mapBox.height() / 2 / scale; y += step)
        p.drawLine(toPx(cx - mapBox.width() / 2 / scale, y), toPx(cx + mapBox.width() / 2 / scale, y));
    p.setPen(Qt::NoPen);
    for (size_t i = 0; i < scan_.size(); i++) {
        if (!scan_.IsHit(i)) continue;
        const float span = scan_.rangeMax - scan_.rangeMin;
        p.setBrush(RangeColor(span > 0 ? (scan_.ranges[i] - scan_.rangeMin) / (static_cast<float>(maxR) - scan_.rangeMin + 1e-6f) : 0.5f));
        p.drawEllipse(toPx(scan_.points[i].x, scan_.points[i].y), 1.8, 1.8);
    }
    // 센서: 점 + 향하는 쪽 화살표
    const QPointF s = toPx(origin_.x, origin_.y);
    p.setBrush(kInk);
    p.drawEllipse(s, 4, 4);
    p.setPen(QPen(kInk, 1.6));
    const QPointF tip(s.x() + 22 * cos(heading_), s.y() - 22 * sin(heading_));
    p.drawLine(s, tip);
    p.restore();
    p.setPen(QPen(kGrid, 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(mapBox);
    p.setPen(kInk2);
    p.drawText(QPointF(mapBox.left(), mapBox.bottom() + 14), QString::fromUtf8("격자 %1 m").arg(step));

    // ---- 방위각 - 거리 (고각 0 에 가장 가까운 채널)
    size_t ch = 0;
    float best = 1e9f;
    for (size_t c = 0; c < scan_.channelCount; c++) {
        const float d = fabsf(scan_.Elevation(c));
        if (d < best) { best = d; ch = c; }
    }
    const size_t n = scan_.azimuthCount;
    const double a0 = scan_.angleMin * 180.0 / M_PI;
    const double a1 = (scan_.angleMin + scan_.angleIncrement * static_cast<float>(n > 0 ? n - 1 : 0)) * 180.0 / M_PI;
    const double rMax = qMax(1.0, maxR * 1.1);
    auto plotPx = [&](double deg, double r) {
        const double u = a1 > a0 ? (deg - a0) / (a1 - a0) : 0.5;
        return QPointF(plotBox.left() + u * plotBox.width(), plotBox.bottom() - (r / rMax) * plotBox.height());
    };
    p.setPen(QPen(kGrid, 1));
    const double rs = NiceStep(rMax, 4);
    for (double r = 0; r <= rMax; r += rs) {
        p.drawLine(plotPx(a0, r), plotPx(a1, r));
        p.setPen(kInk2);
        p.drawText(QRectF(0, plotPx(a0, r).y() - 8, plotBox.left() - 4, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(r));
        p.setPen(QPen(kGrid, 1));
    }
    const double as = NiceStep(a1 - a0, 6);
    for (double a = ceil(a0 / as) * as; a <= a1; a += as) {
        p.drawLine(plotPx(a, 0), plotPx(a, rMax));
        p.setPen(kInk2);
        p.drawText(QRectF(plotPx(a, 0).x() - 20, plotBox.bottom() + 2, 40, 14), Qt::AlignCenter, QString::number(a));
        p.setPen(QPen(kGrid, 1));
    }
    // 맞은 구간끼리만 선으로 잇는다 (빗나간 광선에서 선을 끊는다)
    p.setPen(QPen(kSeries, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    QPainterPath path;
    bool pen = false;
    for (size_t k = 0; k < n; k++) {
        const size_t i = ch * n + k;
        if (!scan_.IsHit(i)) { pen = false; continue; }
        const QPointF q = plotPx((scan_.angleMin + scan_.angleIncrement * static_cast<float>(k)) * 180.0 / M_PI, scan_.ranges[i]);
        if (pen) path.lineTo(q);
        else path.moveTo(q);
        pen = true;
    }
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
    p.setPen(kInk2);
    p.drawText(QRectF(plotBox.left(), plotBox.bottom() + 16, plotBox.width(), 14), Qt::AlignCenter, QString::fromUtf8("방위각 (도)  /  세로: 거리 (m)"));
}
