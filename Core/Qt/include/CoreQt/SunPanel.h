#ifndef CORE_QT_SUNPANEL_H
#define CORE_QT_SUNPANEL_H

#include <QElapsedTimer>
#include <QWidget>

class QCheckBox;
class QDateTimeEdit;
class QDoubleSpinBox;
class QLabel;
class QTimer;
class ModelViewer;

// 태양 설정 패널: 켜기, 날짜/시각, 위도/경도/UTC 오프셋/월드 +y 의 방위, "지금" 버튼, 시간 흐름(초당 몇 분),
// 그리고 지금 태양의 방위각/고도와 그날의 일출/일몰. 바꾸는 즉시 ModelViewer 에 반영되고, 뷰어 쪽에서 바뀌면(sunChanged) 패널도 따라간다.
class SunPanel : public QWidget {
    Q_OBJECT
public:
    explicit SunPanel(ModelViewer* viewer, QWidget* parent = nullptr);
    QSize sizeHint() const override { return QSize(300, 360); }
    bool TimeLapseRunning() const;

private slots:
    void applyFromWidgets();
    void refreshFromViewer();
    void setNow();
    void tick();

private:
    void updateInfo();
    ModelViewer* viewer_;
    QCheckBox* enabled_;
    QCheckBox* shadows_;
    QDateTimeEdit* time_;
    QDoubleSpinBox *lat_, *lon_, *utc_, *north_, *speed_;
    QCheckBox* lapse_;
    QLabel* info_;
    QTimer* timer_;
    QElapsedTimer clock_;
    bool updating_;
};

#endif  // CORE_QT_SUNPANEL_H
