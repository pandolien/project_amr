#ifndef CORECQT_JOINTPANEL_H
#define CORECQT_JOINTPANEL_H

#include <QLabel>
#include <QSlider>
#include <QWidget>
#include <vector>

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
    bool IsEmpty() const { return rows_.empty(); }

signals:
    void valuesChanged();

private slots:
    void onSlider();
    void onReset();

private:
    struct Row {
        std::string name;
        bool angular;
        double lower, upper;   // rad 또는 m
        QSlider* slider;
        QLabel* value;
    };
    static const int kSteps = 1000;
    double valueOf(const Row& r) const;
    void updateLabel(Row& r);

    std::vector<Row> rows_;
    QWidget* body_;
};

#endif  // CORECQT_JOINTPANEL_H
