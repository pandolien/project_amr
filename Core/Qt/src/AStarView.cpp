#include "CoreQt/AStarView.h"

#include <math.h>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

using namespace simul::sim;

// 그림이 그려지는 면. 상태는 AStarView 가 갖고 있다.
class AStarCanvas : public QWidget {
public:
    explicit AStarCanvas(AStarView* owner) : QWidget(owner), v_(owner) { setMinimumSize(260, 260); }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), QColor(24, 28, 36));
        if (!v_->has_) {
            p.setPen(QColor(170, 176, 190));
            p.drawText(rect(), Qt::AlignCenter, QString::fromUtf8("아직 길을 새로 짠 적이 없습니다\n(주민끼리 마주치면 A* 가 돈다)"));
            return;
        }
        const AStarTrace& t = v_->trace_;
        const double gw = t.nx * t.cell, gh = t.ny * t.cell;
        const double top = 26.0, bottom = 34.0, side = 10.0;
        const double scale = qMin((width() - 2 * side) / gw, (height() - top - bottom) / gh);
        const double ox = (width() - gw * scale) / 2.0, oy = top + (height() - top - bottom - gh * scale) / 2.0;
        // 격자 칸 중심 (m) -> 화면. x 는 오른쪽, y 는 위쪽
        const double half = t.cell / 2.0;
        auto sx = [&](double x) { return ox + (x - (t.minx - half)) * scale; };
        auto sy = [&](double y) { return oy + gh * scale - (y - (t.miny - half)) * scale; };

        p.setPen(Qt::NoPen);
        p.fillRect(QRectF(ox, oy, gw * scale, gh * scale), QColor(34, 40, 52));

        // 장애물 안쪽 (비싼 칸): 칸을 따로 그리지 않고 원으로 보인다
        const int n = static_cast<int>(t.expanded.size());
        const int shown = v_->shown_;
        for (int k = 0; k < shown && k < n; k++) {
            const int idx = t.expanded[static_cast<size_t>(k)];
            const int i = idx % t.nx, j = idx / t.nx;
            const double f = n > 1 ? static_cast<double>(k) / (n - 1) : 0.0;
            p.setBrush(QColor::fromHsv(210, 110 + static_cast<int>(f * 120), 245 - static_cast<int>(f * 110)));
            p.drawRect(QRectF(sx(t.minx + i * t.cell - half), sy(t.miny + j * t.cell + half), t.cell * scale, t.cell * scale));
        }
        const bool done = shown >= n;
        if (done) {   // 격자 위 경로
            p.setBrush(QColor(255, 214, 80));
            for (size_t k = 0; k < t.rawPath.size(); k++) {
                const int i = t.rawPath[k] % t.nx, j = t.rawPath[k] / t.nx;
                p.drawRect(QRectF(sx(t.minx + i * t.cell - half), sy(t.miny + j * t.cell + half), t.cell * scale, t.cell * scale));
            }
        } else if (shown > 0) {   // 지금 막 꺼낸 칸
            const int idx = t.expanded[static_cast<size_t>(shown - 1)];
            const int i = idx % t.nx, j = idx / t.nx;
            p.setBrush(QColor(255, 140, 40));
            p.drawRect(QRectF(sx(t.minx + i * t.cell - half), sy(t.miny + j * t.cell + half), t.cell * scale, t.cell * scale));
        }

        // 격자선 (칸이 충분히 클 때만)
        if (t.cell * scale >= 6.0) {
            p.setPen(QColor(255, 255, 255, 18));
            for (int i = 0; i <= t.nx; i++) p.drawLine(QPointF(ox + i * t.cell * scale, oy), QPointF(ox + i * t.cell * scale, oy + gh * scale));
            for (int j = 0; j <= t.ny; j++) p.drawLine(QPointF(ox, oy + j * t.cell * scale), QPointF(ox + gw * scale, oy + j * t.cell * scale));
        }

        // 장애물
        for (size_t k = 0; k < t.obstacles.size(); k++) {
            const Circle2& o = t.obstacles[k];
            p.setPen(QPen(QColor(255, 90, 90), 2));
            p.setBrush(QColor(255, 80, 80, 70));
            p.drawEllipse(QPointF(sx(o.x), sy(o.y)), o.r * scale, o.r * scale);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(255, 120, 120));
            p.drawEllipse(QPointF(sx(o.x), sy(o.y)), 3.0, 3.0);
        }

        // 최종 경로
        if (done && t.found) {
            QPen pen(QColor(90, 235, 120), 3);
            pen.setJoinStyle(Qt::RoundJoin);
            p.setPen(pen);
            QPointF prev(sx(t.start.x), sy(t.start.y));
            for (size_t k = 0; k < t.path.size(); k++) {
                const QPointF cur(sx(t.path[k].x), sy(t.path[k].y));
                p.drawLine(prev, cur);
                prev = cur;
            }
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(255, 255, 255));
            for (size_t k = 0; k + 1 < t.path.size(); k++) p.drawEllipse(QPointF(sx(t.path[k].x), sy(t.path[k].y)), 3.5, 3.5);
        }

        // 시작 / 목표
        const QPointF s(sx(t.start.x), sy(t.start.y)), g(sx(t.goal.x), sy(t.goal.y));
        p.setPen(QPen(QColor(255, 255, 255), 2));
        p.setBrush(QColor(60, 200, 90));
        p.drawEllipse(s, 6.0, 6.0);
        p.setBrush(QColor(240, 70, 70));
        p.drawRect(QRectF(g.x() - 6, g.y() - 6, 12, 12));
        p.setPen(QColor(235, 238, 245));
        p.drawText(QPointF(s.x() + 9, s.y() - 8), QString::fromUtf8("시작"));
        p.drawText(QPointF(g.x() + 9, g.y() - 8), QString::fromUtf8("목표"));

        // 제목과 범례
        p.setPen(QColor(235, 238, 245));
        QFont f = p.font();
        f.setBold(true);
        p.setFont(f);
        p.drawText(QPointF(10, 18), v_->title_);
        f.setBold(false);
        p.setFont(f);
        p.setPen(QColor(170, 176, 190));
        p.drawText(QPointF(10, height() - 12),
                   QString::fromUtf8("파랑 살펴본 칸 · 노랑 격자 경로 · 초록 줄인 경로 · 빨강 피할 구역 (칸 %1 cm)")
                       .arg(static_cast<int>(t.cell * 100.0 + 0.5)));
    }

private:
    AStarView* v_;
};

AStarView::AStarView(QWidget* parent) : QWidget(parent), has_(false), shown_(0), carry_(0.0) {
    canvas_ = new AStarCanvas(this);
    play_ = new QPushButton(QString::fromUtf8("일시정지"), this);
    restart_ = new QPushButton(QString::fromUtf8("처음부터"), this);
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, 0);
    speed_ = new QComboBox(this);
    speed_->addItem(QString::fromUtf8("느리게"), 0.25);
    speed_->addItem(QString::fromUtf8("보통"), 1.0);
    speed_->addItem(QString::fromUtf8("빠르게"), 4.0);
    speed_->setCurrentIndex(1);
    info_ = new QLabel(this);
    timer_ = new QTimer(this);
    timer_->setInterval(33);

    QHBoxLayout* row = new QHBoxLayout();
    row->addWidget(play_);
    row->addWidget(restart_);
    row->addWidget(speed_);
    row->addWidget(info_, 1);
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(canvas_, 1);
    lay->addWidget(slider_);
    lay->addLayout(row);

    connect(timer_, &QTimer::timeout, this, &AStarView::onTick);
    connect(play_, &QPushButton::clicked, this, &AStarView::onPlayToggled);
    connect(restart_, &QPushButton::clicked, this, &AStarView::onRestart);
    connect(slider_, &QSlider::valueChanged, this, &AStarView::onSlider);
    updateControls();
}

bool AStarView::IsPlaying() const { return timer_->isActive(); }

void AStarView::SetTrace(const AStarTrace& trace, const QString& title) {
    trace_ = trace;
    title_ = title;
    has_ = trace.nx > 0 && trace.ny > 0 && trace.cell > 0.0;
    slider_->blockSignals(true);
    slider_->setRange(0, static_cast<int>(trace_.expanded.size()));
    slider_->blockSignals(false);
    onRestart();
}

void AStarView::setShown(int n) {
    const int total = static_cast<int>(trace_.expanded.size());
    shown_ = n < 0 ? 0 : (n > total ? total : n);
    slider_->blockSignals(true);
    slider_->setValue(shown_);
    slider_->blockSignals(false);
    updateControls();
    canvas_->update();
}

void AStarView::updateControls() {
    const int total = static_cast<int>(trace_.expanded.size());
    play_->setText(timer_->isActive() ? QString::fromUtf8("일시정지") : QString::fromUtf8("재생"));
    play_->setEnabled(has_);
    restart_->setEnabled(has_);
    slider_->setEnabled(has_);
    if (!has_) {
        info_->setText(QString());
    } else if (shown_ >= total) {
        info_->setText(QString::fromUtf8("%1칸 살펴봄 → %2개 꺾임점%3")
                           .arg(total)
                           .arg(static_cast<int>(trace_.path.size()))
                           .arg(trace_.found ? QString() : QString::fromUtf8(" (길 없음)")));
    } else {
        info_->setText(QString::fromUtf8("살펴본 칸 %1 / %2").arg(shown_).arg(total));
    }
}

void AStarView::onTick() {
    const int total = static_cast<int>(trace_.expanded.size());
    const double per = (total / 120.0 > 1.0 ? total / 120.0 : 1.0) * speed_->currentData().toDouble();
    carry_ += per;
    const int add = static_cast<int>(carry_);
    carry_ -= add;
    setShown(shown_ + add);
    if (shown_ >= total) {
        timer_->stop();
        updateControls();
    }
}

void AStarView::onPlayToggled() {
    if (timer_->isActive()) {
        timer_->stop();
    } else {
        if (shown_ >= static_cast<int>(trace_.expanded.size())) setShown(0);
        timer_->start();
    }
    updateControls();
}

void AStarView::onRestart() {
    carry_ = 0.0;
    setShown(0);
    if (has_) timer_->start();
    else timer_->stop();
    updateControls();
}

void AStarView::onSlider(int value) {
    timer_->stop();
    carry_ = 0.0;
    setShown(value);
}
