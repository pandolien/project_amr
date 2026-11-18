#ifndef CORECQT_PROPERTIESPANEL_H
#define CORECQT_PROPERTIESPANEL_H

#include <QDoubleSpinBox>
#include <QLabel>
#include <QWidget>

class ModelViewer;

// 엔진(ModelViewer) 밖에 두는 속성 위젯. 3D 화면 안에 들어가지 않고, 호스트가 도크, 사이드 패널, 별도 창 어디든 놓는다.
// 엔진의 공개 함수와 신호만 쓴다: 선택한 링크가 속한 모델의 이름, 위치(x, y, z m), 회전(roll, pitch, yaw 도)을 보여 주고,
// 숫자를 고치면 ModelViewer::SetModelPlacement6 로 엔진을 제어한다. 크기 배율 칸은 ModelViewer::SetModelScale 로 모델 전체 크기(질량 포함)를 바꾼다. 엔진 쪽 변화(드래그, 선택)는 신호로 받아 따라간다.
class PropertiesPanel : public QWidget {
    Q_OBJECT
public:
    explicit PropertiesPanel(ModelViewer* viewer, QWidget* parent = nullptr);

    int Model() const { return model_; }                       // 지금 보여 주는 모델 (없으면 -1)
    QDoubleSpinBox* Field(int i) const { return spin_[i]; }    // 0..2 위치, 3..5 회전, 6 크기 배율 (테스트/호스트 확장용)

private slots:
    void refresh();

private:
    ModelViewer* viewer_;
    QLabel* name_;
    QDoubleSpinBox* spin_[7];
    int model_;
    bool updating_;
};

#endif  // CORECQT_PROPERTIESPANEL_H
