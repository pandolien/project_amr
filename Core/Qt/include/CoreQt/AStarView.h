#ifndef CORE_QT_ASTARVIEW_H
#define CORE_QT_ASTARVIEW_H

#include <QWidget>

#include "CSimul/AStar.h"

class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

// A* 탐색 과정을 위에서 내려다본 그림으로 보여 준다 (QPainter 만 쓴다).
// 파란 칸 = 꺼내서 살펴본 칸 (꺼낸 순서대로 진해진다), 빨간 원 = 피할 장애물, 노란 칸 = 격자 위 최단 경로,
// 초록 선 = 직선으로 줄인 최종 경로. 재생/일시정지, 처음부터, 슬라이더로 한 칸씩 돌려 볼 수 있다.
class AStarView : public QWidget {
    Q_OBJECT
public:
    explicit AStarView(QWidget* parent = nullptr);
    // 새 탐색 기록을 올리고 처음부터 재생한다. title 은 그림 위에 쓴다 (예: 주민 이름).
    void SetTrace(const simul::sim::AStarTrace& trace, const QString& title);
    bool HasTrace() const { return has_; }
    bool IsPlaying() const;                // 재생 중이면 true (끝났거나 멈췄으면 false)
    int Shown() const { return shown_; }   // 지금까지 보여 준 탐색 칸 수
    QSize sizeHint() const override { return QSize(460, 520); }

private slots:
    void onTick();
    void onPlayToggled();
    void onRestart();
    void onSlider(int value);

private:
    friend class AStarCanvas;
    void setShown(int n);
    void updateControls();

    simul::sim::AStarTrace trace_;
    QString title_;
    bool has_;
    int shown_;
    double carry_;   // 틱마다 보여 줄 칸 수의 소수 부분

    QWidget* canvas_;
    QPushButton* play_;
    QPushButton* restart_;
    QSlider* slider_;
    QComboBox* speed_;
    QLabel* info_;
    QTimer* timer_;
};

#endif  // CORE_QT_ASTARVIEW_H
