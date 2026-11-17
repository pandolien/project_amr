#ifndef CORECQT_SELECTIONOVERLAY_H
#define CORECQT_SELECTIONOVERLAY_H

#include <QFrame>
#include <QLabel>
#include <QPoint>
#include <QSlider>
#include <QString>
#include <QVector>

#include "CURDF/Urdf.h"

// 3D 화면 위에 떠서, 클릭한 물체의 정보와 그 물체를 움직이는 관절 슬라이더를 보여주는 작은 패널.
// 부모(3D 화면)의 자식 위젯으로 만들고, SetContent 로 내용을 채운 뒤 PlaceNear 로 물체 옆에 놓는다.
// 슬라이더를 움직이면 valueEdited 가 나온다 (값은 rad/m).
class SelectionOverlay : public QFrame {
    Q_OBJECT
public:
    struct Joint {
        QString name;
        bool angular;          // true 면 도(degree)로, 아니면 m 로 보여준다
        double lower, upper;   // rad 또는 m
        double value;          // rad 또는 m
    };

    explicit SelectionOverlay(QWidget* parent = nullptr);

    void SetContent(const QString& title, const QString& info, const QVector<Joint>& joints);
    void SetInfo(const QString& info);
    // 이름이 같은 슬라이더의 위치만 바꾼다 (시그널 없음). 잡고 있는 슬라이더는 건드리지 않는다.
    void ShowValues(const simul::urdf::JointValues& q);
    void PlaceNear(const QPoint& anchor);   // anchor(부모 좌표) 오른쪽에 놓고, 부모 안에 들어오게 민다
    bool HasJoints() const { return !rows_.isEmpty(); }

signals:
    void valueEdited(const QString& jointName, double value);

private:
    struct Row {
        Joint joint;
        QSlider* slider;
        QLabel* value;
    };
    static const int kSteps = 1000;
    double valueOf(const Row& r) const;
    void setSlider(Row& r, double v);
    void updateLabel(Row& r);

    QLabel* title_;
    QLabel* info_;
    QWidget* rowsBox_;
    QVector<Row> rows_;
};

#endif  // CORECQT_SELECTIONOVERLAY_H
