#ifndef CORECQT_JOINTPANEL_H
#define CORECQT_JOINTPANEL_H

#include <QLabel>
#include <QSlider>
#include <QWidget>
#include "CBase/Map.h"
#include "CBase/String.h"
#include "CBase/Array.h"

#include "CURDF/Urdf.h"

// URDF 의 움직이는 관절마다 슬라이더를 만든다 (revolute/continuous = 도, prismatic = m).
// 값이 바뀌면 valuesChanged() 를 내보내므로 Values() 를 ModelViewer::SetJointValues 에 넘기면 된다.
class JointPanel : public QWidget {
    Q_OBJECT
public:
    explicit JointPanel(QWidget* parent = nullptr);

    void SetRobot(const amr::urdf::Robot& robot);   // 슬라이더를 새로 만든다 (모두 0 으로)
    amr::urdf::JointValues Values() const;          // 관절 이름 -> rad/m
    void SetValue(const QString& jointName, double value);  // rad/m 로 지정
    // 슬라이더의 표시만 바꾼다 (시그널을 내지 않는다). 시뮬레이션처럼 프로그램이 값을 계속 바꿀 때 쓴다.
    // 사용자가 지금 잡고 있는 슬라이더는 건드리지 않는다. 슬라이더 눈금 때문에 값이 조금 반올림되어 보인다.
    void ShowValues(const amr::urdf::JointValues& q);
    bool IsEmpty() const { return rows_.empty(); }

    // 관절별 토크(회전 N·m, 이동 N)를 슬라이더 아래에 표시한다. 관절의 effort 한계를 넘으면 빨갛게 보인다.
    // 이름 -> 값 (amr::dynamics::JointTorques 와 같은 형식). 비어 있으면 표시를 지운다.
    void SetTorques(const amr::Map<amr::String, double>& torques);

signals:
    void valuesChanged();
    void valueEdited(const QString& jointName, double value);  // 슬라이더가 바뀐 관절 하나 (SetValue 로 바꿔도 나온다)

private slots:
    void onSlider();
    void onReset();

private:
    struct Row {
        amr::String name;
        bool angular;
        double lower, upper;   // rad 또는 m
        QSlider* slider;
        QLabel* value;
        QLabel* torque;        // 토크 표시 (SetTorques)
        double effort;         // URDF limit 의 effort (0 이면 한계 정보 없음)
        QString torqueUnit;    // "N*m" 또는 "N"
    };
    static const int kSteps = 1000;
    double valueOf(const Row& r) const;
    void updateLabel(Row& r);

    amr::Array<Row> rows_;
    QWidget* body_;
};

#endif  // CORECQT_JOINTPANEL_H
