#include "CoreQt/FloatingPanel.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QPainter>
#include <QVBoxLayout>

#include "CBase/Util.h"

FloatingPanel::FloatingPanel(const QString& title, QWidget* content, QWidget* parent) : QFrame(parent), title_(nullptr), topInset_(0), dragging_(false), grip_(nullptr), resizing_(false) {
    setObjectName("floatingPanel");
    setStyleSheet(
        "#floatingPanel { background: rgba(26, 29, 38, 238); border: 1px solid rgba(255,255,255,40); border-radius: 10px; }"
        "#floatingPanelTitle { background: transparent; color: #f2f4f8; font-weight: bold; font-size: 13px; padding: 8px 12px 6px 12px; }"
        "#floatingPanelClose { background: transparent; color: #9aa3b8; border: none; padding: 0 12px; font-size: 13px; }"
        "#floatingPanelClose:hover { color: white; background: rgba(255,90,90,170); border-top-right-radius: 9px; }");
    QVBoxLayout* col = new QVBoxLayout(this);
    col->setContentsMargins(0, 0, 0, 0);
    col->setSpacing(0);
    QHBoxLayout* bar = new QHBoxLayout;
    bar->setSpacing(0);
    title_ = new QLabel(title, this);
    title_->setObjectName("floatingPanelTitle");
    title_->setCursor(Qt::SizeAllCursor);
    QPushButton* close = new QPushButton(QString::fromUtf8("✕"), this);
    close->setObjectName("floatingPanelClose");
    close->setCursor(Qt::ArrowCursor);
    close->setFlat(true);
    connect(close, &QPushButton::clicked, this, &FloatingPanel::closeRequested);
    bar->addWidget(title_, 1);
    bar->addWidget(close);
    col->addLayout(bar);
    QFrame* line = new QFrame(this);   // 제목과 내용 사이 얇은 구분선
    line->setFixedHeight(1);
    line->setStyleSheet("background: rgba(255,255,255,28);");
    col->addWidget(line);
    content->setParent(this);
    col->addWidget(content, 1);
    col->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout* foot = new QHBoxLayout;
    foot->setContentsMargins(0, 0, 0, 0);
    foot->addStretch(1);
    foot->setContentsMargins(0, 0, 2, 2);
    // QSizeGrip 은 최상위 창(메인 윈도우)의 크기를 바꾸므로 쓰지 않고, 이 패널만 키우는 손잡이를 직접 둔다
    grip_ = new QWidget(this);
    grip_->setFixedSize(16, 16);
    grip_->setCursor(Qt::SizeFDiagCursor);
    grip_->setStyleSheet("background: transparent;");
    grip_->installEventFilter(this);
    foot->addWidget(grip_, 0, Qt::AlignRight | Qt::AlignBottom);
    col->addLayout(foot);
    setMinimumSize(200, 120);
    hide();
}

void FloatingPanel::SetTopInset(int px) {
    topInset_ = px;
    Clamp();
}

void FloatingPanel::PlaceDefault(const QPoint& pos, const QSize& size) {
    resize(size);
    move(pos);
    Clamp();
}

void FloatingPanel::Clamp() {
    if (!parentWidget()) return;
    const QRect area = parentWidget()->rect().adjusted(4, 4 + topInset_, -4, -4);
    const int w = simul::Min(width(), simul::Max(area.width(), minimumWidth()));
    const int h = simul::Min(height(), simul::Max(area.height(), minimumHeight()));
    if (w != width() || h != height()) resize(w, h);
    const int x = simul::Max(area.left(), simul::Min(pos().x(), area.right() - width() + 1));
    const int y = simul::Max(area.top(), simul::Min(pos().y(), area.bottom() - height() + 1));
    if (QPoint(x, y) != pos()) move(x, y);
}

// 제목줄을 잡고 끌어서 옮긴다
void FloatingPanel::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton && title_->geometry().contains(e->pos())) {
        dragging_ = true;
        grab_ = e->pos();
        raise();
        e->accept();
        return;
    }
    QFrame::mousePressEvent(e);
}

void FloatingPanel::mouseMoveEvent(QMouseEvent* e) {
    if (!dragging_) {
        QFrame::mouseMoveEvent(e);
        return;
    }
    move(mapToParent(e->pos()) - grab_);
    Clamp();
}

void FloatingPanel::mouseReleaseEvent(QMouseEvent* e) {
    dragging_ = false;
    QFrame::mouseReleaseEvent(e);
}

bool FloatingPanel::eventFilter(QObject* obj, QEvent* e) {
    if (obj != grip_) return QFrame::eventFilter(obj, e);
    if (e->type() == QEvent::Paint) {   // 점 세 개짜리 작은 삼각 표시
        QPainter p(grip_);
        p.setPen(QColor(255, 255, 255, 90));
        for (int i = 0; i < 3; i++) p.drawLine(grip_->width() - 3 - i * 4, grip_->height() - 2, grip_->width() - 2, grip_->height() - 3 - i * 4);
        return true;
    }
    QMouseEvent* m = dynamic_cast<QMouseEvent*>(e);
    if (!m) return false;
    if (e->type() == QEvent::MouseButtonPress && m->button() == Qt::LeftButton) {
        resizing_ = true;
        resizeStart_ = m->globalPos();
        sizeStart_ = size();
        return true;
    }
    if (e->type() == QEvent::MouseMove && resizing_) {
        const QPoint d = m->globalPos() - resizeStart_;
        QSize ns(sizeStart_.width() + d.x(), sizeStart_.height() + d.y());
        if (parentWidget()) {   // 부모(3D 화면) 밖으로는 키우지 않는다
            ns.setWidth(simul::Min(ns.width(), parentWidget()->width() - 4 - x()));
            ns.setHeight(simul::Min(ns.height(), parentWidget()->height() - 4 - y()));
        }
        ns = ns.expandedTo(minimumSize());
        resize(ns);
        return true;
    }
    if (e->type() == QEvent::MouseButtonRelease) {
        resizing_ = false;
        return true;
    }
    return false;
}
