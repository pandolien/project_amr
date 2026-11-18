#ifndef CORECQT_SIMULVIEW_H
#define CORECQT_SIMULVIEW_H

#include <QStringList>
#include <QTimer>
#include <QWidget>

#include "CoreQt/ModelViewer.h"
#include "CoreQt/SelectionOverlay.h"

// 다른 Qt 프로그램(호스트)에 붙여 쓰는 3D 뷰 부품.
//   - ModelViewer(3D 화면 + 조작)와, 클릭한 물체 옆에 뜨는 선택 패널을 하나로 묶었다.
//   - 호스트가 Features 로 무엇을 켤지 고른다. 메뉴, 도크, 로그 같은 바깥 UI 는 전혀 만들지 않는다 (필요하면 호스트가 viewer() 의 시그널을 받아 만든다).
//   - 모든 기능은 viewer() 로 접근한다 (모델 추가/제거, 위치/자세, 관절 값, 카메라, 표시 옵션, 시그널 전부).
//
//   최소 사용:   SimulView* v = new SimulView(parent);  v->viewer()->AddFile("arm.urdf");
class SimulView : public QWidget {
    Q_OBJECT
public:
    enum Feature {
        kSelectionPanel = 1,   // 클릭한 물체 옆의 정보 + 관절 슬라이더 패널
        kMouseEdit = 2,        // 좌표계 화살표 이동, 회전 링, IK 끌기 (끄면 보기와 선택만)
        kDropFiles = 4,        // 파일을 끌어다 놓아 추가
        kAll = kSelectionPanel | kMouseEdit | kDropFiles
    };

    explicit SimulView(QWidget* parent = nullptr, unsigned features = kAll);

    ModelViewer* viewer() const { return viewer_; }
    SelectionOverlay* selectionPanel() const { return overlay_; }

    unsigned Features() const { return features_; }
    void SetFeatures(unsigned features);
    void EnableFeature(Feature f, bool on) { SetFeatures(on ? (features_ | f) : (features_ & ~static_cast<unsigned>(f))); }
    bool HasFeature(Feature f) const { return (features_ & f) != 0; }

signals:
    // 선택 패널의 슬라이더로 관절 값을 바꿨다 (값은 이미 viewer() 에 반영되었다. 호스트의 다른 UI 를 맞출 때 쓴다)
    void jointEdited(const QString& jointName, double value);
    // kDropFiles 가 켜진 채 파일이 놓였다. 받는 쪽이 없으면 viewer()->AddFile 로 바로 추가한다.
    void filesDropped(const QStringList& paths);

protected:
    void dragEnterEvent(QDragEnterEvent* e) override;
    void dropEvent(QDropEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

private slots:
    void refreshPanel();   // 선택이 바뀌었을 때: 내용을 새로 채운다
    void syncPanel();      // 같은 선택에서 자세/관절 값이 바뀌었을 때: 숫자와 슬라이더만 갱신
    void placePanel();     // 카메라가 움직여도 물체 옆을 따라다닌다

private:
    QString panelInfo(int link) const;

    ModelViewer* viewer_;
    SelectionOverlay* overlay_;
    QTimer* placeTimer_;
    unsigned features_;
    int panelLink_;   // 패널이 지금 보여주는 링크 (-1 이면 없음)
};

#endif  // CORECQT_SIMULVIEW_H
