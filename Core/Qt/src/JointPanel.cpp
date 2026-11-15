#include "CoreQt/JointPanel.h"

#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace {
const double kPi = 3.14159265358979323846;
}

JointPanel::JointPanel(QWidget* parent) : QWidget(parent), body_(nullptr) {
    QVBoxLayout* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);
    QScrollArea* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    body_ = new QWidget;
    new QVBoxLayout(body_);
    scroll->setWidget(body_);
    outer->addWidget(scroll, 1);
    QPushButton* reset = new QPushButton(QString::fromUtf8("모두 0 으로"), this);
    connect(reset, SIGNAL(clicked()), this, SLOT(onReset()));
    outer->addWidget(reset);
    setMinimumWidth(240);
}

void JointPanel::SetRobot(const amr::urdf::Robot& robot) {
    rows_.clear();
    // 기존 위젯 제거
    delete body_->layout();
    const QList<QWidget*> old = body_->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
    for (int i = 0; i < old.size(); i++) delete old[i];
    QVBoxLayout* lay = new QVBoxLayout(body_);

    const std::vector<int> act = robot.ActuatedJoints();
    for (size_t k = 0; k < act.size(); k++) {
        const amr::urdf::Joint& j = robot.joints[static_cast<size_t>(act[k])];
        Row r;
        r.name = j.name;
        r.angular = j.type != amr::urdf::kJointPrismatic;
        if (j.type == amr::urdf::kJointContinuous || !j.hasLimit) {
            r.lower = r.angular ? -kPi : -1.0;
            r.upper = r.angular ? kPi : 1.0;
        } else {
            r.lower = static_cast<double>(j.lower);
            r.upper = static_cast<double>(j.upper);
        }
        QLabel* name = new QLabel(QString::fromStdString(j.name), body_);
        r.slider = new QSlider(Qt::Horizontal, body_);
        r.slider->setRange(0, kSteps);
        r.value = new QLabel(body_);
        r.value->setMinimumWidth(70);
        r.value->setAlignment(Qt::AlignRight);
        QWidget* line = new QWidget(body_);
        QHBoxLayout* h = new QHBoxLayout(line);
        h->setContentsMargins(0, 0, 0, 0);
        h->addWidget(r.slider, 1);
        h->addWidget(r.value);
        lay->addWidget(name);
        lay->addWidget(line);
        rows_.push_back(r);
    }
    lay->addStretch(1);
    for (size_t i = 0; i < rows_.size(); i++) {
        // 0 이 범위 안이면 0 위치, 아니면 가장 가까운 끝
        const Row& r = rows_[i];
        double zero = 0.0;
        zero = std::max(r.lower, std::min(r.upper, zero));
        const double span = r.upper - r.lower;
        rows_[i].slider->setValue(span > 0 ? static_cast<int>(std::lround((zero - r.lower) / span * kSteps)) : 0);
        updateLabel(rows_[i]);
        connect(rows_[i].slider, SIGNAL(valueChanged(int)), this, SLOT(onSlider()));
    }
}

double JointPanel::valueOf(const Row& r) const {
    return r.lower + (r.upper - r.lower) * static_cast<double>(r.slider->value()) / kSteps;
}

void JointPanel::updateLabel(Row& r) {
    const double v = valueOf(r);
    r.value->setText(r.angular ? QString("%1°").arg(v * 180.0 / kPi, 0, 'f', 1) : QString("%1 m").arg(v, 0, 'f', 3));
}

amr::urdf::JointValues JointPanel::Values() const {
    amr::urdf::JointValues q;
    for (size_t i = 0; i < rows_.size(); i++) q[rows_[i].name] = static_cast<float>(valueOf(rows_[i]));
    return q;
}

void JointPanel::SetValue(const QString& jointName, double value) {
    for (size_t i = 0; i < rows_.size(); i++) {
        if (QString::fromStdString(rows_[i].name) != jointName) continue;
        Row& r = rows_[i];
        const double span = r.upper - r.lower;
        const double c = std::max(r.lower, std::min(r.upper, value));
        r.slider->setValue(span > 0 ? static_cast<int>(std::lround((c - r.lower) / span * kSteps)) : 0);
    }
}

void JointPanel::onSlider() {
    for (size_t i = 0; i < rows_.size(); i++) updateLabel(rows_[i]);
    emit valuesChanged();
}

void JointPanel::onReset() {
    for (size_t i = 0; i < rows_.size(); i++) SetValue(QString::fromStdString(rows_[i].name), 0.0);
}
