#include "CoreQt/PropertiesPanel.h"

#include <QFormLayout>

#include "CoreQt/ModelViewer.h"

PropertiesPanel::PropertiesPanel(ModelViewer* viewer, QWidget* parent) : QWidget(parent), viewer_(viewer), name_(nullptr), model_(-1), updating_(false) {
    for (int i = 0; i < 7; i++) spin_[i] = nullptr;
    QFormLayout* form = new QFormLayout(this);
    name_ = new QLabel(this);
    QFont bold = name_->font();
    bold.setBold(true);
    name_->setFont(bold);
    form->addRow(QString::fromUtf8("모델"), name_);
    const char* labels[6] = {"위치 x (m)", "위치 y (m)", "위치 z (m)", "회전 roll (°)", "회전 pitch (°)", "회전 yaw (°)"};
    for (int i = 0; i < 6; i++) {
        QDoubleSpinBox* sp = new QDoubleSpinBox(this);
        sp->setKeyboardTracking(false);   // Enter 나 다른 칸으로 옮길 때, 또는 화살표를 누를 때만 적용한다
        if (i < 3) {
            sp->setRange(-100000.0, 100000.0);
            sp->setDecimals(3);
            sp->setSingleStep(0.05);
        } else {
            sp->setRange(-180.0, 180.0);
            sp->setDecimals(1);
            sp->setSingleStep(5.0);
            sp->setWrapping(true);
        }
        spin_[i] = sp;
        form->addRow(QString::fromUtf8(labels[i]), sp);
        connect(sp, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            if (updating_ || model_ < 0) return;
            float v[6];
            for (int k = 0; k < 6; k++) v[k] = static_cast<float>(spin_[k]->value());
            viewer_->SetModelPlacement6(model_, v);   // 엔진 제어: 모델의 월드 위치/회전
        });
    }
    {   // 크기 배율: 모델 전체(길이, 관절 위치, 도형, 메시, 질량)를 원래 크기의 몇 배로 할지
        QDoubleSpinBox* sp = new QDoubleSpinBox(this);
        sp->setKeyboardTracking(false);
        sp->setRange(0.001, 1000.0);
        sp->setDecimals(3);
        sp->setSingleStep(0.1);
        sp->setValue(1.0);
        sp->setToolTip(QString::fromUtf8("원래 크기의 몇 배인지. 길이와 질량이 함께 바뀝니다 (질량은 배율의 세제곱)"));
        spin_[6] = sp;
        form->addRow(QString::fromUtf8("크기 배율 (x)"), sp);
        connect(sp, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this](double s) {
            if (updating_ || model_ < 0) return;
            viewer_->SetModelScale(model_, s);   // 엔진 제어: 모델 전체 크기
        });
    }
    // 엔진 쪽 변화를 따라간다
    connect(viewer_, &ModelViewer::selectionChanged, this, &PropertiesPanel::refresh);
    connect(viewer_, &ModelViewer::selectionPicked, this, &PropertiesPanel::refresh);
    connect(viewer_, &ModelViewer::robotChanged, this, &PropertiesPanel::refresh);
    connect(viewer_, &ModelViewer::modelChanged, this, &PropertiesPanel::refresh);
    connect(viewer_, &ModelViewer::objectMoved, this, &PropertiesPanel::refresh);
    refresh();
}

void PropertiesPanel::refresh() {
    int model = -1;
    if (viewer_->SelectedLink() >= 0) model = viewer_->ModelOfLink(viewer_->SelectedLink());
    if (model >= viewer_->ModelCount()) model = -1;
    model_ = model;
    updating_ = true;
    if (model < 0) {
        name_->setText(QString::fromUtf8("(선택 없음)"));
        for (int i = 0; i < 6; i++) spin_[i]->setValue(0.0);
        spin_[6]->setValue(1.0);
    } else {
        float v[6];
        viewer_->ModelPlacement6(model, v);
        name_->setText(viewer_->ModelName(model));
        for (int i = 0; i < 6; i++)
            if (!spin_[i]->hasFocus()) spin_[i]->setValue(static_cast<double>(v[i]));   // 입력 중인 칸은 건드리지 않는다
        if (!spin_[6]->hasFocus()) spin_[6]->setValue(viewer_->ModelScale(model));
    }
    for (int i = 0; i < 7; i++) spin_[i]->setEnabled(model >= 0);
    updating_ = false;
}
