#include "CoreQt/SelectionOverlay.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include "CBase/Math.h"
#include "CoreQt/QtString.h"
#include "CBase/Util.h"
#include "3d/Units.h"

SelectionOverlay::SelectionOverlay(QWidget* parent) : QFrame(parent), title_(nullptr), info_(nullptr), rowsBox_(nullptr) {
    setObjectName("selectionOverlay");
    setStyleSheet(
        "#selectionOverlay { background: rgba(28, 30, 38, 225); border: 1px solid rgba(255,255,255,70); border-radius: 8px; }"
        "#selectionOverlay QLabel { color: #e8eaf0; background: transparent; }"
        "#selectionOverlay QSlider::groove:horizontal { height: 4px; background: rgba(255,255,255,60); border-radius: 2px; }"
        "#selectionOverlay QSlider::handle:horizontal { background: #6aa9ff; width: 12px; margin: -5px 0; border-radius: 6px; }");
    setFixedWidth(270);
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(10, 8, 10, 8);
    lay->setSpacing(4);
    title_ = new QLabel(this);
    title_->setStyleSheet("font-weight: bold; font-size: 13px;");
    info_ = new QLabel(this);
    info_->setTextFormat(Qt::RichText);
    info_->setStyleSheet("font-size: 11px; color: #b8bdca;");
    rowsBox_ = new QWidget(this);
    new QVBoxLayout(rowsBox_);
    rowsBox_->layout()->setContentsMargins(0, 4, 0, 0);
    lay->addWidget(title_);
    lay->addWidget(info_);
    lay->addWidget(rowsBox_);
    hide();
}

void SelectionOverlay::SetContent(const QString& title, const QString& info, const QVector<Joint>& joints) {
    title_->setText(title);
    info_->setText(info);
    rows_.clear();
    delete rowsBox_->layout();
    const QList<QWidget*> old = rowsBox_->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
    for (int i = 0; i < old.size(); i++) delete old[i];
    QVBoxLayout* lay = new QVBoxLayout(rowsBox_);
    lay->setContentsMargins(0, 4, 0, 0);
    lay->setSpacing(2);
    for (int i = 0; i < joints.size(); i++) {
        Row r;
        r.joint = joints[i];
        QLabel* name = new QLabel(joints[i].name, rowsBox_);
        name->setStyleSheet("font-size: 11px;");
        r.slider = new QSlider(Qt::Horizontal, rowsBox_);
        r.slider->setRange(0, kSteps);
        r.value = new QLabel(rowsBox_);
        r.value->setMinimumWidth(64);
        r.value->setAlignment(Qt::AlignRight);
        r.value->setStyleSheet("font-size: 11px;");
        QWidget* line = new QWidget(rowsBox_);
        QHBoxLayout* h = new QHBoxLayout(line);
        h->setContentsMargins(0, 0, 0, 0);
        h->addWidget(r.slider, 1);
        h->addWidget(r.value);
        lay->addWidget(name);
        lay->addWidget(line);
        rows_.push_back(r);
    }
    for (int i = 0; i < rows_.size(); i++) {
        setSlider(rows_[i], rows_[i].joint.value);
        updateLabel(rows_[i]);
        const int index = i;
        connect(rows_[i].slider, &QSlider::valueChanged, this, [this, index]() {
            Row& r = rows_[index];
            updateLabel(r);
            emit valueEdited(r.joint.name, valueOf(r));
        });
    }
    adjustSize();
}

void SelectionOverlay::SetInfo(const QString& info) {
    info_->setText(info);
}

double SelectionOverlay::valueOf(const Row& r) const {
    return r.joint.lower + (r.joint.upper - r.joint.lower) * static_cast<double>(r.slider->value()) / kSteps;
}

void SelectionOverlay::setSlider(Row& r, double v) {
    const double span = r.joint.upper - r.joint.lower;
    const double c = simul::Max(r.joint.lower, simul::Min(r.joint.upper, v));
    const bool blocked = r.slider->blockSignals(true);
    r.slider->setValue(span > 0 ? static_cast<int>(simul::Lround((c - r.joint.lower) / span * kSteps)) : 0);
    r.slider->blockSignals(blocked);
}

void SelectionOverlay::updateLabel(Row& r) {
    const double v = valueOf(r);
    r.value->setText(r.joint.angular ? QString("%1°").arg(simul::units::RadToDeg(v), 0, 'f', 1) : QString("%1 m").arg(v, 0, 'f', 3));
}

void SelectionOverlay::ShowValues(const simul::urdf::JointValues& q) {
    for (int i = 0; i < rows_.size(); i++) {
        Row& r = rows_[i];
        const simul::urdf::JointValues::const_iterator it = q.find(simulqt::FromQ(r.joint.name));
        if (it == q.end() || r.slider->isSliderDown()) continue;
        setSlider(r, static_cast<double>(it->second));
        updateLabel(r);
    }
}

void SelectionOverlay::PlaceNear(const QPoint& anchor) {
    if (!parentWidget()) return;
    adjustSize();
    const QRect area = parentWidget()->rect().adjusted(6, 6, -6, -6);
    int x = anchor.x() + 28;
    int y = anchor.y() - height() / 2;
    if (x + width() > area.right()) x = anchor.x() - 28 - width();   // 오른쪽이 모자라면 왼쪽에 놓는다
    x = simul::Max(area.left(), simul::Min(x, area.right() - width()));
    y = simul::Max(area.top(), simul::Min(y, area.bottom() - height()));
    if (pos() != QPoint(x, y)) move(x, y);
    raise();
}
