#ifndef CORECQT_SELECTIONOVERLAY_H
#define CORECQT_SELECTIONOVERLAY_H

#include <QFrame>
#include <QLabel>
#include <QMouseEvent>
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
    void SetTopInset(int px) { topInset_ = px; }   // 위쪽에 이만큼은 비워 둔다 (메뉴줄 아래로 들어가지 않게)
    void ResetOffset() { moved_ = false; }   // 끌어서 옮긴 자리를 잊고 기본 자리(물체 오른쪽)로 돌아간다

signals:
    void valueEdited(const QString& jointName, double value);

protected:
    // 슬라이더 말고 패널의 아무 곳이나 잡고 끌면 옮길 수 있다. 옮긴 자리는 물체 기준 상대 위치로 기억해서 물체를 계속 따라다닌다.
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

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

    int topInset_;
    bool moved_;           // 사용자가 끌어서 옮겼는가
    QPoint offset_;        // 옮겼다면 물체 위치 기준 상대 위치
    QPoint anchor_;        // 마지막으로 받은 물체 위치
    QPoint grab_;          // 끌기 시작 때 패널 안에서 잡은 점
    bool dragging_;
    QLabel* title_;
    QLabel* info_;
    QWidget* rowsBox_;
    QVector<Row> rows_;
};

#endif  // CORECQT_SELECTIONOVERLAY_H
