#include "CoreQt/ShapeDialog.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "CoreQt/ModelViewer.h"

static QDoubleSpinBox* MakeSpin(double lo, double hi, double step, int decimals, const QString& suffix) {
    QDoubleSpinBox* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setSingleStep(step);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    s->setKeyboardTracking(false);
    return s;
}

ShapeDialog::ShapeDialog(QWidget* parent, Shape shape, const double* pos) : QDialog(parent), zAuto_(true) {
    setWindowTitle(QString::fromUtf8("도형 추가"));
    setModal(true);
    color_ = QColor(120, 160, 220);

    shape_ = new QComboBox;
    shape_->addItem(QString::fromUtf8("박스"));
    shape_->addItem(QString::fromUtf8("원통"));
    shape_->addItem(QString::fromUtf8("구"));
    name_ = new QLineEdit;

    QFormLayout* form = new QFormLayout;
    form->addRow(QString::fromUtf8("종류"), shape_);
    form->addRow(QString::fromUtf8("이름"), name_);
    for (int i = 0; i < 3; i++) {
        sizeLabel_[i] = new QLabel;
        size_[i] = MakeSpin(0.001, 1000.0, 0.05, 3, " m");
        size_[i]->setValue(0.5);
        form->addRow(sizeLabel_[i], size_[i]);
        connect(size_[i], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ShapeDialog::sizeChanged);
    }
    const char* posNames[3] = {"위치 x", "위치 y", "위치 z"};
    for (int i = 0; i < 3; i++) {
        pos_[i] = MakeSpin(-10000.0, 10000.0, 0.1, 3, " m");
        pos_[i]->setValue(pos ? pos[i] : 0.0);
        form->addRow(QString::fromUtf8(posNames[i]), pos_[i]);
    }
    // z 를 직접 고치면 더는 크기에 맞춰 따라가지 않는다
    connect(pos_[2], static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this]() {
        if (pos_[2]->hasFocus()) zAuto_ = false;
    });
    mass_ = MakeSpin(0.001, 100000.0, 1.0, 3, " kg");
    mass_->setValue(5.0);
    form->addRow(QString::fromUtf8("질량"), mass_);
    colorBtn_ = new QPushButton;
    colorBtn_->setMinimumHeight(24);
    connect(colorBtn_, &QPushButton::clicked, this, &ShapeDialog::pickColor);
    form->addRow(QString::fromUtf8("색"), colorBtn_);

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    box->button(QDialogButtonBox::Ok)->setText(QString::fromUtf8("추가"));
    box->button(QDialogButtonBox::Cancel)->setText(QString::fromUtf8("취소"));
    connect(box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);

    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->addLayout(form);
    lay->addWidget(box);

    connect(shape_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ShapeDialog::shapeChanged);
    shape_->setCurrentIndex(static_cast<int>(shape));
    shapeChanged();
    if (!pos) pos_[2]->setValue(baseHeight());
    updateColorButton();
}

ShapeDialog::Shape ShapeDialog::CurrentShape() const { return static_cast<Shape>(shape_->currentIndex()); }
void ShapeDialog::SetShape(Shape s) { shape_->setCurrentIndex(static_cast<int>(s)); }
QString ShapeDialog::Name() const { return name_->text().trimmed(); }
void ShapeDialog::SetName(const QString& name) { name_->setText(name); }
double ShapeDialog::Size(int i) const { return size_[i]->value(); }
void ShapeDialog::SetSize(int i, double v) { size_[i]->setValue(v); }
double ShapeDialog::Position(int i) const { return pos_[i]->value(); }
void ShapeDialog::SetPosition(int i, double v) {
    pos_[i]->setValue(v);
    if (i == 2) zAuto_ = false;
}
void ShapeDialog::SetColor(const QColor& c) {
    if (!c.isValid()) return;
    color_ = c;
    updateColorButton();
}
double ShapeDialog::Mass() const { return mass_->value(); }
void ShapeDialog::SetMass(double kg) { mass_->setValue(kg); }

double ShapeDialog::baseHeight() const {
    switch (CurrentShape()) {
        case kBox: return size_[2]->value() * 0.5;
        case kCylinder: return size_[1]->value() * 0.5;   // 길이의 절반 (원통 축은 z)
        default: return size_[0]->value();                 // 구는 반지름
    }
}

void ShapeDialog::shapeChanged() {
    const Shape s = CurrentShape();
    const char* names[3][3] = {{"가로 (x)", "세로 (y)", "높이 (z)"}, {"반지름", "길이", ""}, {"반지름", "", ""}};
    const int used = s == kBox ? 3 : (s == kCylinder ? 2 : 1);
    for (int i = 0; i < 3; i++) {
        sizeLabel_[i]->setText(QString::fromUtf8(names[s][i]));
        sizeLabel_[i]->setVisible(i < used);
        size_[i]->setVisible(i < used);
    }
    // 종류별 기본 이름/크기 (이름은 사용자가 고치지 않은 기본값일 때만 바꾼다)
    const char* defNames[3] = {"box", "cylinder", "sphere"};
    const QString cur = name_->text();
    if (cur.isEmpty() || cur == "box" || cur == "cylinder" || cur == "sphere") name_->setText(defNames[s]);
    if (s == kBox) { size_[0]->setValue(0.5); size_[1]->setValue(0.5); size_[2]->setValue(0.5); }
    else if (s == kCylinder) { size_[0]->setValue(0.25); size_[1]->setValue(0.5); }
    else size_[0]->setValue(0.25);
    sizeChanged();
}

void ShapeDialog::sizeChanged() {
    if (zAuto_) pos_[2]->setValue(baseHeight());
}

void ShapeDialog::pickColor() {
    const QColor c = QColorDialog::getColor(color_, this, QString::fromUtf8("도형 색"));
    if (c.isValid()) SetColor(c);
}

void ShapeDialog::updateColorButton() {
    colorBtn_->setText(color_.name());
    const QColor fg = color_.lightness() > 128 ? Qt::black : Qt::white;
    colorBtn_->setStyleSheet(QString("QPushButton { background: %1; color: %2; }").arg(color_.name(), fg.name()));
}

int ShapeDialog::Apply(ModelViewer* viewer, QString* error) const {
    if (!viewer) return -1;
    const float pose[6] = {static_cast<float>(Position(0)), static_cast<float>(Position(1)), static_cast<float>(Position(2)), 0, 0, 0};
    const QString name = Name().isEmpty() ? QString("shape") : Name();
    switch (CurrentShape()) {
        case kBox: return viewer->AddBox(name, Size(0), Size(1), Size(2), pose, color_, Mass(), error);
        case kCylinder: return viewer->AddCylinder(name, Size(0), Size(1), pose, color_, Mass(), error);
        default: return viewer->AddSphere(name, Size(0), pose, color_, Mass(), error);
    }
}
