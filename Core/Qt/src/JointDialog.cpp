#include "CoreQt/JointDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

#include "3d/dhMat.h"
#include "CURDF/World.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/QtString.h"

namespace {

QDoubleSpinBox* MakeSpin(double lo, double hi, double step, int decimals, const QString& suffix) {
    QDoubleSpinBox* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setSingleStep(step);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    s->setKeyboardTracking(false);
    return s;
}

// 모델 m 이 child 에 (직접 또는 간접으로) 붙어 있는가: 그 모델의 링크에 child 를 연결하면 순환이 된다
bool HangsFrom(const ModelViewer* v, int m, int child) {
    for (int guard = 0; m >= 0 && guard <= v->ModelCount(); guard++) {
        if (m == child) return true;
        m = v->ModelParent(m);
    }
    return false;
}

}  // namespace

JointDialog::JointDialog(ModelViewer* viewer, int child, int preferredParentLink, QWidget* parentWidget) : QDialog(parentWidget), viewer_(viewer), child_(child) {
    setWindowTitle(QString::fromUtf8("관절로 연결"));
    setModal(true);

    QFormLayout* form = new QFormLayout;
    form->addRow(QString::fromUtf8("연결할 모델"), new QLabel(viewer_->ModelName(child_)));

    parent_ = new QComboBox;
    const simul::urdf::Robot& r = viewer_->Robot();
    for (size_t i = 0; i < r.links.size(); i++) {
        const int m = viewer_->ModelOfLink(static_cast<int>(i));
        if (m < 0 || HangsFrom(viewer_, m, child_)) continue;   // world 링크, 자기 모델, 자기에게 붙은 모델은 후보가 아니다
        parent_->addItem(viewer_->ModelName(m) + QString::fromUtf8("  ›  ") + simulqt::ToQ(r.links[i].name), static_cast<int>(i));
    }
    form->addRow(QString::fromUtf8("부모 링크"), parent_);
    SetParentLink(preferredParentLink);

    kind_ = new QComboBox;
    kind_->addItem(QString::fromUtf8("고정 (fixed)"));
    kind_->addItem(QString::fromUtf8("회전, 한계 있음 (revolute)"));
    kind_->addItem(QString::fromUtf8("회전, 한계 없음 (continuous)"));
    kind_->addItem(QString::fromUtf8("직선 이동 (prismatic)"));
    form->addRow(QString::fromUtf8("관절 종류"), kind_);

    QHBoxLayout* ax = new QHBoxLayout;
    const char* an[3] = {"x ", "y ", "z "};
    for (int i = 0; i < 3; i++) {
        axis_[i] = MakeSpin(-1000.0, 1000.0, 0.1, 3, QString());
        axis_[i]->setValue(i == 2 ? 1.0 : 0.0);
        ax->addWidget(new QLabel(QString::fromUtf8(an[i])));
        ax->addWidget(axis_[i]);
    }
    form->addRow(QString::fromUtf8("관절 축 (관절 좌표계)"), ax);

    keep_ = new QRadioButton(QString::fromUtf8("지금 자리 그대로 (관절 위치를 자동 계산)"));
    manual_ = new QRadioButton(QString::fromUtf8("직접 입력 (부모 링크 기준)"));
    keep_->setChecked(true);
    form->addRow(QString::fromUtf8("관절 위치"), keep_);
    form->addRow(QString(), manual_);
    const char* on[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
    QHBoxLayout* o1 = new QHBoxLayout;
    QHBoxLayout* o2 = new QHBoxLayout;
    for (int i = 0; i < 6; i++) {
        origin_[i] = MakeSpin(-10000.0, 10000.0, i < 3 ? 0.05 : 5.0, i < 3 ? 3 : 1, i < 3 ? QString::fromUtf8(" m") : QString::fromUtf8("°"));
        (i < 3 ? o1 : o2)->addWidget(new QLabel(QString::fromUtf8(on[i])));
        (i < 3 ? o1 : o2)->addWidget(origin_[i]);
    }
    form->addRow(QString(), o1);
    form->addRow(QString(), o2);

    lower_ = MakeSpin(-100000.0, 100000.0, 5.0, 3, QString());
    upper_ = MakeSpin(-100000.0, 100000.0, 5.0, 3, QString());
    effort_ = MakeSpin(0.0, 1e6, 1.0, 2, QString());
    velocity_ = MakeSpin(0.0, 1e6, 0.5, 2, QString());
    effort_->setValue(10.0);
    velocity_->setValue(1.0);
    limitUnit_ = new QLabel;
    QHBoxLayout* lim = new QHBoxLayout;
    lim->addWidget(new QLabel(QString::fromUtf8("최소")));
    lim->addWidget(lower_);
    lim->addWidget(new QLabel(QString::fromUtf8("최대")));
    lim->addWidget(upper_);
    lim->addWidget(limitUnit_);
    form->addRow(QString::fromUtf8("관절 한계"), lim);
    QHBoxLayout* eff = new QHBoxLayout;
    eff->addWidget(new QLabel(QString::fromUtf8("힘/토크")));
    eff->addWidget(effort_);
    eff->addWidget(new QLabel(QString::fromUtf8("속도")));
    eff->addWidget(velocity_);
    form->addRow(QString(), eff);

    name_ = new QLineEdit;
    name_->setPlaceholderText(viewer_->ModelName(child_) + QStringLiteral("_attach"));
    form->addRow(QString::fromUtf8("관절 이름"), name_);

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    box->button(QDialogButtonBox::Ok)->setText(QString::fromUtf8("연결"));
    box->button(QDialogButtonBox::Cancel)->setText(QString::fromUtf8("취소"));
    connect(box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->addLayout(form);
    lay->addWidget(box);

    connect(kind_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &JointDialog::kindChanged);
    connect(keep_, &QRadioButton::toggled, this, &JointDialog::keepChanged);
    kindChanged();
    keepChanged();
}

int JointDialog::ParentLink() const { return parent_->currentIndex() < 0 ? -1 : parent_->currentData().toInt(); }

int JointDialog::ParentLinkCount() const { return parent_->count(); }

bool JointDialog::SetParentLink(int link) {
    const int i = parent_->findData(link);
    if (i < 0) return false;
    parent_->setCurrentIndex(i);
    return true;
}

int JointDialog::JointKind() const { return kind_->currentIndex(); }

void JointDialog::SetJointKind(int kind) { kind_->setCurrentIndex(kind); }

void JointDialog::SetAxis(double x, double y, double z) {
    axis_[0]->setValue(x);
    axis_[1]->setValue(y);
    axis_[2]->setValue(z);
}

void JointDialog::SetLimits(double lower, double upper) {
    lower_->setValue(lower);
    upper_->setValue(upper);
}

void JointDialog::SetJointName(const QString& name) { name_->setText(name); }

void JointDialog::SetKeepPose(bool keep) {
    keep_->setChecked(keep);
    manual_->setChecked(!keep);
}

void JointDialog::SetOrigin(const double v[6]) {
    for (int i = 0; i < 6; i++) origin_[i]->setValue(v[i]);
}

void JointDialog::kindChanged() {
    const int k = kind_->currentIndex();
    const bool moves = k != 0;
    const bool limited = k == 1 || k == 3;
    for (int i = 0; i < 3; i++) axis_[i]->setEnabled(moves);
    lower_->setEnabled(limited);
    upper_->setEnabled(limited);
    effort_->setEnabled(moves);
    velocity_->setEnabled(moves);
    limitUnit_->setText(k == 3 ? QString::fromUtf8("m") : QString::fromUtf8("도"));
    if (limited) {   // 처음 고른 한계가 비어 있으면 알맞은 기본값을 넣는다
        if (lower_->value() == 0.0 && upper_->value() == 0.0) {
            lower_->setValue(k == 3 ? 0.0 : -90.0);
            upper_->setValue(k == 3 ? 0.5 : 90.0);
        }
    }
}

void JointDialog::keepChanged() {
    for (int i = 0; i < 6; i++) origin_[i]->setEnabled(manual_->isChecked());
}

bool JointDialog::Apply(QString* error) const {
    simul::urdf::AttachSpec spec;
    const int k = kind_->currentIndex();
    spec.type = k == 1 ? simul::urdf::kJointRevolute : k == 2 ? simul::urdf::kJointContinuous : k == 3 ? simul::urdf::kJointPrismatic : simul::urdf::kJointFixed;
    spec.axis = dhVector(static_cast<float>(axis_[0]->value()), static_cast<float>(axis_[1]->value()), static_cast<float>(axis_[2]->value()));
    if (k == 1 || k == 3) {
        spec.hasLimit = true;
        const double toSi = k == 1 ? dhRad(1.0) : 1.0;   // 회전 한계는 도로 입력받는다
        spec.lower = static_cast<float>(lower_->value() * toSi);
        spec.upper = static_cast<float>(upper_->value() * toSi);
    }
    if (k != 0) {
        spec.effort = static_cast<float>(effort_->value());
        spec.velocity = static_cast<float>(velocity_->value());
    }
    spec.jointName = simulqt::FromQ(name_->text().trimmed());
    const int link = ParentLink();
    if (link < 0) {
        if (error) *error = QString::fromUtf8("연결할 부모 링크가 없습니다 (다른 모델이 필요합니다)");
        return false;
    }
    if (manual_->isChecked()) {
        const dhMat m = dhMat::Trans(static_cast<float>(origin_[0]->value()), static_cast<float>(origin_[1]->value()), static_cast<float>(origin_[2]->value())) *
                        dhMat::RotZ(static_cast<float>(dhRad(origin_[5]->value()))) * dhMat::RotY(static_cast<float>(dhRad(origin_[4]->value()))) *
                        dhMat::RotX(static_cast<float>(dhRad(origin_[3]->value())));
        return viewer_->ConnectModel(child_, link, spec, &m, error);
    }
    return viewer_->ConnectModel(child_, link, spec, nullptr, error);
}
