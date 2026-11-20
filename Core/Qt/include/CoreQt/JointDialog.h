#pragma once
// 모델을 다른 모델의 링크에 관절로 연결하는 창 (관절로 연결하기).
// 부모 링크, 관절 종류(fixed/revolute/continuous/prismatic), 축, 관절 위치(지금 자리 유지 또는 직접 입력), 한계, 이름을 입력받아
// ModelViewer::ConnectModel 로 적용한다. 호스트가 자기 UI 에서 그대로 띄워도 된다.
#include <QDialog>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QRadioButton;
class ModelViewer;

class JointDialog : public QDialog {
    Q_OBJECT
public:
    // child: 연결할 모델 번호. preferredParentLink: 부모 링크 칸의 처음 선택 (합친 Robot 의 링크 번호, 연결할 수 없는 링크이면 첫 번째)
    JointDialog(ModelViewer* viewer, int child, int preferredParentLink = -1, QWidget* parent = nullptr);

    int ParentLink() const;            // 선택한 부모 링크 (합친 Robot 의 링크 번호), 후보가 없으면 -1
    bool SetParentLink(int link);      // 후보에 있으면 선택하고 true
    int ParentLinkCount() const;       // 후보 링크 수 (다른 모델의 링크. 이 모델에 붙어 있는 모델의 링크는 순환이 되므로 빠진다)
    int JointKind() const;             // 0 fixed, 1 revolute, 2 continuous, 3 prismatic
    void SetJointKind(int kind);
    void SetAxis(double x, double y, double z);
    void SetLimits(double lower, double upper);   // 회전은 도, 이동은 m
    void SetJointName(const QString& name);
    void SetKeepPose(bool keep);       // true: 연결해도 모델이 그 자리에 그대로 (기본). false: 아래 위치/자세 칸의 값을 관절 origin 으로 쓴다
    void SetOrigin(const double v[6]); // x, y, z (m), roll, pitch, yaw (도). SetKeepPose(false) 일 때 쓰인다

    // 입력한 값으로 연결한다. 성공하면 true, 실패하면 false 와 이유
    bool Apply(QString* error = nullptr) const;

private slots:
    void kindChanged();
    void keepChanged();

private:
    ModelViewer* viewer_;
    int child_;
    QComboBox* parent_;
    QComboBox* kind_;
    QDoubleSpinBox* axis_[3];
    QDoubleSpinBox* origin_[6];
    QRadioButton* keep_;
    QRadioButton* manual_;
    QDoubleSpinBox* lower_;
    QDoubleSpinBox* upper_;
    QDoubleSpinBox* effort_;
    QDoubleSpinBox* velocity_;
    QLabel* limitUnit_;
    QLineEdit* name_;
};
