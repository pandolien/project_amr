#include "CoreQt/FloatingPanel.h"

#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QSizeGrip>
#include <QVBoxLayout>

#include "CBase/Util.h"

FloatingPanel::FloatingPanel(const QString& title, QWidget* content, QWidget* parent) : QFrame(parent), title_(nullptr), topInset_(0), dragging_(false) {
    setObjectName("floatingPanel");
    setStyleSheet(
        "#floatingPanel { background: rgba(250, 250, 252, 240); border: 1px solid rgba(0,0,0,110); border-radius: 6px; }"
        "#floatingPanelTitle { background: rgba(28, 30, 38, 235); color: #e8eaf0; font-weight: bold; padding: 4px 8px; border-top-left-radius: 5px; }"
        "#floatingPanelClose { background: rgba(28, 30, 38, 235); color: #e8eaf0; border: none; padding: 0 8px; border-top-right-radius: 5px; }"
        "#floatingPanelClose:hover { background: #c0392b; }");
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
    content->setParent(this);
    col->addWidget(content, 1);
    QHBoxLayout* foot = new QHBoxLayout;
    foot->setContentsMargins(0, 0, 0, 0);
    foot->addStretch(1);
    foot->addWidget(new QSizeGrip(this), 0, Qt::AlignRight | Qt::AlignBottom);   // 오른쪽 아래 모서리로 크기 바꾸기
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
