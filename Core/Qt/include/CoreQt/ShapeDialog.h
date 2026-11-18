#pragma once
// 기본 도형(박스, 원통, 구)을 장면에 올리기 전에 이름, 크기, 위치, 색, 질량을 입력받는 창.
// SimulView 의 "추가" 메뉴와 3D 화면 오른쪽 클릭 메뉴가 쓴다. 호스트가 자기 UI 에서 그대로 띄워도 된다.
#include <QColor>
#include <QDialog>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class ModelViewer;

class ShapeDialog : public QDialog {
    Q_OBJECT
public:
    enum Shape { kBox = 0, kCylinder = 1, kSphere = 2 };

    // pos 는 놓을 자리(m). 비우면 원점 옆. z 는 크기를 바꾸면 "바닥에 닿는 높이"로 따라간다 (직접 고치면 멈춘다).
    explicit ShapeDialog(QWidget* parent = nullptr, Shape shape = kBox, const double* pos = nullptr);

    Shape CurrentShape() const;
    void SetShape(Shape s);
    QString Name() const;
    void SetName(const QString& name);
    // 박스: 가로, 세로, 높이 / 원통: 반지름, 길이 / 구: 반지름 (모두 m)
    double Size(int i) const;
    void SetSize(int i, double v);
    double Position(int i) const;      // x, y, z (m)
    void SetPosition(int i, double v);
    QColor Color() const { return color_; }
    void SetColor(const QColor& c);
    double Mass() const;
    void SetMass(double kg);

    // 입력한 값으로 viewer 에 도형을 올린다. 성공하면 모델 번호, 실패하면 -1 (error 에 이유)
    int Apply(ModelViewer* viewer, QString* error = nullptr) const;

private slots:
    void shapeChanged();
    void sizeChanged();
    void pickColor();

private:
    double baseHeight() const;   // 바닥에 닿게 하려면 z 가 이 값
    void updateColorButton();

    QComboBox* shape_;
    QLineEdit* name_;
    QLabel* sizeLabel_[3];
    QDoubleSpinBox* size_[3];
    QDoubleSpinBox* pos_[3];
    QDoubleSpinBox* mass_;
    QPushButton* colorBtn_;
    QColor color_;
    bool zAuto_;
};
